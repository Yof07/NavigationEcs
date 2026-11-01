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
    /// A* 算法的对外入口：为一批寻路操作调度并行搜索Job
    /// 具体搜索逻辑在AStarSearch里，本类只负责装配与调度
    /// </summary>
    public static class AStarPathfinder
    {
        /// <summary>
        /// 为一个Chunk内的所有寻路操作调度并行A*
        /// </summary>
        public static JobHandle Schedule(
            in ArchetypeChunk _chunk,
            in Grid _grid,
            in LocalTransform _gridLocalTransform,
            NativeArray<GridNode> _nodes,
            ComponentLookup<PathfindingOperation> _operations,
            ComponentLookup<PathfindingResult> _results,
            BufferLookup<PathBuffer> _pathPoints,
            EntityTypeHandle _entityTypeHandle,
            JobHandle _inputDeps)
        {
            // 每个Job需要独立的节点数组副本（并行Job间不能共享写入）
            NativeArray<GridNode> newN = new NativeArray<GridNode>(_nodes.Length, Allocator.TempJob);
            NativeArray<GridNode>.Copy(_nodes, newN);

            AStarSearchJob searchJob = new AStarSearchJob()
            {
                OperationEntities = _chunk.GetNativeArray(_entityTypeHandle), // 本Chunk中的操作Entity列表
                JobAllOperations = _operations,
                JobAllResults = _results,
                JobAllPathPoints = _pathPoints,
                JobNodes = newN,
                JobGrid = _grid,
                JobGridLocalTransform = _gridLocalTransform,
            };
            
            // 第二个参数 innerloopBatchCount：每个调度单元内循环处理的元素数。
            // 2 = 每两个 op 打成一个包，由工作线程逐个抢包执行（1 才是最小粒度）。
            return searchJob.Schedule(_chunk.Count, 2, _inputDeps);
        }
    }

    /// <summary>
    /// 并行A* 搜索Job（AStarPathfinder的Job壳）
    /// 为Chunk内每个寻路操作Entity各启动一个AStarSearch执行A* 算法
    /// BurstCompile标注使此Job被Burst编译为原生代码，性能大幅提升
    /// </summary>
    [BurstCompile(CompileSynchronously = true)]
    public struct AStarSearchJob : IJobParallelFor
    {
        // 本Chunk中的操作Entity列表（只读）
        [ReadOnly]
        public NativeArray<Entity> OperationEntities;

        // 各种组件/缓冲区查找（NativeDisableContainerSafetyRestriction允许并行访问）
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

                parallelSearch.Execute(); // 执行A* 算法
            }
        }
    }
}
