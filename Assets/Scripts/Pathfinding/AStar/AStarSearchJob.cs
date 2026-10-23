using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Navigation
{
    /// <summary>
    /// 并行 A* 搜索 Job。
    /// 为 Chunk 内每个寻路操作 Entity 各启动一个 AStarSearch 执行 A* 算法。
    /// BurstCompile 标注使此 Job 被 Burst 编译为原生代码，性能大幅提升
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    public struct AStarSearchJob : IJobParallelFor
    {
        // 本 Chunk 中的操作 Entity 列表（只读）
        [ReadOnly]
        public NativeArray<Entity> OperationEntities;

        // 各种组件/缓冲区查找（NativeDisableContainerSafetyRestriction 允许并行访问）
        [NativeDisableContainerSafetyRestriction, ReadOnly]
        public ComponentLookup<PathfindingOperation> JobAllOperations;

        [NativeDisableContainerSafetyRestriction]
        public ComponentLookup<PathfindingResult> JobAllResults;

        [NativeDisableContainerSafetyRestriction]
        public BufferLookup<PathBuffer> JobAllPathPoints;

        // 每个线程共享同一份节点数组副本（并行写入不同位置，安全）
        [NativeDisableParallelForRestriction, DeallocateOnJobCompletion]
        public NativeArray<GridNode> JobNodes;

        public Grid JobGrid;
        public LocalTransform JobGridLocalTransform;

        public void Execute(int index)
        {
            Entity e = OperationEntities[index];
            PathfindingResult res = JobAllResults[e];

            // 只处理尚未完成搜索的操作（避免重复计算）
            if (!res.FinishedSearch)
            {
                AStarSearch parallelSearch = new AStarSearch()
                {
                    OperationEntity = e,
                    AllOperations = JobAllOperations,
                    AllResults = JobAllResults,
                    AllPathPoints = JobAllPathPoints,
                    Nodes = JobNodes,
                    Grid = JobGrid,
                    GridLocalTransform = JobGridLocalTransform,
                };

                parallelSearch.Execute(); // 执行 A* 算法
            }
        }
    }
}
