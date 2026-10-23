using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;

namespace Navigation
{
    /// <summary>
    /// 寻路编排系统。只做三件事：
    /// 1. 每帧把网格节点数据拷成 NativeArray（供算法使用）；
    /// 2. 调度 A*（具体逻辑在 AStar/AStarPathfinder.cs，本系统不含搜索逻辑）；
    /// 3. 同帧 Complete，保证下游 AgentMovementSystem 能读到结果。
    /// </summary>
    [UpdateAfter(typeof(PathfindingRequestSystem))]
    partial struct PathfindingSystem : ISystem
    {
        private EntityTypeHandle EntityTypeHandle;

        // 操作数据（只读）
        [NativeDisableContainerSafetyRestriction, ReadOnly]
        public ComponentLookup<PathfindingOperation> SystemAllOperations;

        // 操作结果（读写：算法写入 PathFound / FinishedSearch）
        [NativeDisableContainerSafetyRestriction]
        public ComponentLookup<PathfindingResult> SystemAllResults;

        // 路径点缓冲区（读写：算法写入规划出的路径）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<PathBuffer> SystemAllPathPoints;

        // 网格节点缓冲区（只读：算法在自己的副本上修改 G/H）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<GridNodesBuffer> SystemAllGridNodes;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            EntityTypeHandle = SystemAPI.GetEntityTypeHandle();
            SystemAllOperations = state.GetComponentLookup<PathfindingOperation>(true);
            SystemAllResults = state.GetComponentLookup<PathfindingResult>();
            SystemAllPathPoints = state.GetBufferLookup<PathBuffer>();
            SystemAllGridNodes = state.GetBufferLookup<GridNodesBuffer>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // 每帧刷新所有 Lookup（结构性变更后不刷新会读到过期指针）
            SystemAllOperations.Update(ref state);
            SystemAllResults.Update(ref state);
            SystemAllPathPoints.Update(ref state);
            SystemAllGridNodes.Update(ref state);
            EntityTypeHandle.Update(ref state);

            // 查询所有同时拥有 PathfindingResult 和 PathfindingOperation 的 Entity（寻路操作 Entity）
            NativeArray<ComponentType> types = new NativeArray<ComponentType>(2, Allocator.Temp);
            types[0] = ComponentType.ReadOnly<PathfindingResult>();
            types[1] = ComponentType.ReadOnly<PathfindingOperation>();

            var query = state.GetEntityQuery(types);
            var chunks = query.ToArchetypeChunkArray(Allocator.Temp);

            // 遍历所有 Grid Entity（通常只有一个）
            foreach (var (grid, gridLocalTransform, gridEntity) in SystemAPI
                         .Query<Grid, LocalTransform>().WithEntityAccess())
            {
                // 将网格节点从 DynamicBuffer 复制到 NativeArray（供 Job 使用）
                NativeArray<GridNodesBuffer> nodesBufferComponents = SystemAllGridNodes[gridEntity].ToNativeArray(Allocator.Temp);
                NativeArray<GridNode> nodes = new NativeArray<GridNode>(nodesBufferComponents.Length, Allocator.TempJob);

                for (int i = 0; i < nodesBufferComponents.Length; i++)
                {
                    nodes[i] = nodesBufferComponents[i].Node;
                }

                // 对每个 Archetype Chunk 处理其中的所有寻路操作
                foreach (var archetypeChunk in chunks)
                {
                    AStarPathfinder.Schedule(archetypeChunk, grid, gridLocalTransform, nodes,
                            SystemAllOperations, SystemAllResults, SystemAllPathPoints, EntityTypeHandle, new JobHandle())
                        .Complete(); // 同帧完成寻路
                }

                nodesBufferComponents.Dispose();
                nodes.Dispose();
            }

            types.Dispose();
            chunks.Dispose();
        }
    }
}
