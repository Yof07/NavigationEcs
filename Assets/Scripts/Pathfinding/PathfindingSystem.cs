using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;
using UnityEngine.Profiling;

namespace Navigation
{
    /// <summary>
    /// 寻路编排系统只做三件事：
    /// 1. 每帧把网格节点数据拷成NativeArray（供算法使用）；
    /// 2. 调度A*（调度入口在AStar/AStarPathfinder.cs，搜索算法在AStar/AStarSearch.cs，本系统不含搜索逻辑）；
    /// 3. 同帧Complete，保证下游AgentMovementSystem能读到结果
    /// </summary>
    [UpdateAfter(typeof(PathfindingRequestSystem))]
    partial struct PathfindingSystem : ISystem
    {
        private EntityTypeHandle EntityTypeHandle;

        // 操作数据（只读）
        [NativeDisableContainerSafetyRestriction, ReadOnly]
        public ComponentLookup<PathfindingOperation> SystemAllOperations;

        // 操作结果（读写：算法写入PathFound / FinishedSearch）
        [NativeDisableContainerSafetyRestriction]
        public ComponentLookup<PathfindingResult> SystemAllResults;

        // 路径点缓冲区（读写：算法写入规划出的路径）
        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<PathBuffer> SystemAllPathPoints;

        // 网格节点缓冲区（只读：算法在自己的副本上修改G/H）
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
            // 每帧刷新所有Lookup（结构性变更后不刷新会读到过期指针）
            SystemAllOperations.Update(ref state);
            SystemAllResults.Update(ref state);
            SystemAllPathPoints.Update(ref state);
            SystemAllGridNodes.Update(ref state);
            EntityTypeHandle.Update(ref state);
            
            // 查询所有同时拥有PathfindingResult和PathfindingOperation的Entity（寻路操作Entity）
            NativeArray<ComponentType> types = new NativeArray<ComponentType>(2, Allocator.Temp);
            types[0] = ComponentType.ReadOnly<PathfindingResult>();
            types[1] = ComponentType.ReadOnly<PathfindingOperation>();
            var query = state.GetEntityQuery(types);
            var chunks = query.ToArchetypeChunkArray(Allocator.Temp);
            
            // 遍历所有Grid Entity（通常只有一个Grid，后续可能会出现多个带网格的Ecs场景则依然适用）
            foreach (var (grid, gridLocalTransform, gridEntity) in SystemAPI
                         .Query<Grid, LocalTransform>().WithEntityAccess())
            {
                // 将网格节点从DynamicBuffer复制到NativeArray（供Job使用）
                NativeArray<GridNodesBuffer> nodesBufferComponents = SystemAllGridNodes[gridEntity].ToNativeArray(Allocator.Temp);
                NativeArray<GridNode> nodes = new NativeArray<GridNode>(nodesBufferComponents.Length, Allocator.TempJob);

                // 复制nodesBufferComponents数据到nodes中
                for (int i = 0; i < nodesBufferComponents.Length; i++)
                {
                    nodes[i] = nodesBufferComponents[i].Node;
                }

                // 对每个Archetype Chunk内部的所有寻路操作Entities都用AStarPathfinder.Schedule来调度A*
                Profiler.BeginSample("PathfindingSystem: Schedule A*");
                foreach (var archetypeChunk in chunks)
                {
                    AStarPathfinder.Schedule(archetypeChunk, grid, gridLocalTransform, nodes,
                            SystemAllOperations, SystemAllResults, SystemAllPathPoints, EntityTypeHandle, new JobHandle())
                        .Complete(); // 同帧完成寻路
                }
                Profiler.EndSample();

                nodesBufferComponents.Dispose();
                nodes.Dispose();
            }
            

            types.Dispose();
            chunks.Dispose();
        }
    }
}
