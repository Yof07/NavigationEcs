using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using KNN;
using KNN.Jobs;

namespace Navigation
{
    /// <summary>
    /// AgentMovementSystem 的阶段 3：邻居查询。
    /// 只负责「把谁的邻居算出来、写进 NeighbourBuffer」，
    /// 具体用 KdTree 还是均匀网格由 NeighbourQuerySettings.Method 决定；
    /// 阶段 4（ORCA）对这些邻居的用法不受影响。
    /// </summary>
    partial struct AgentMovementSystem
    {
        /// <summary>
        /// KNN 分支：每帧重建 KD-Tree，批量查询每个避障代理的最近 k 个代理，
        /// 过滤后写入各自的 NeighbourBuffer。
        /// 临时内存（results / knnContainer）由本方法自己分配、自己释放。
        /// </summary>
        private void WriteNeighboursWithKdTree(
            NativeList<float3> _avoidanceAgentsPositions,
            int _kNeighbours,
            NativeArray<Entity> _avAgEnts)
        {
            // 构建 KD-Tree 并执行批量 K 近邻查询
            var results = new NativeArray<int>(_avoidanceAgentsPositions.Length * _kNeighbours, Allocator.TempJob);
            var knnContainer = new KnnContainer(_avoidanceAgentsPositions.AsArray(), false, Allocator.TempJob);

            var rebuildJob = new KnnRebuildJob(knnContainer);
            rebuildJob.Schedule().Complete(); // 构建 KD-Tree

            // 并行查询每个代理的 K 近邻（结果存储在 results 数组中）
            // batchSize 至少为 1：代理数 < 32 时 Length / 32 会算出 0，ScheduleBatch 不接受 0
            var batchQueryJob = new QueryKNearestBatchJob(knnContainer, _avoidanceAgentsPositions.AsArray(), results);
            batchQueryJob.ScheduleBatch(_avoidanceAgentsPositions.Length, math.max(1, _avoidanceAgentsPositions.Length / 32)).Complete();

            // 将 KNN 查询结果写入每个代理的邻居缓冲区（过滤掉自身）
            for (int i = 0; i < _avAgEnts.Length; i++)
            {
                Entity e = _avAgEnts[i];
                DynamicBuffer<NeighbourBuffer> neighbours = AllAgentsNeighbours[e];
                neighbours.Clear();

                // 可能出现周围代理数量不足 k 个的情况
                int n = math.min(_kNeighbours, _avAgEnts.Length);

                // 遍历代理周围的要避开的代理
                for (int j = 0; j < n; j++)
                {
                    Entity otherEntity = _avAgEnts[results[i * _kNeighbours + j]];

                    if (IsValidNeighbour(e, otherEntity))
                    {
                        neighbours.Add(new NeighbourBuffer() { Agent = otherEntity });
                    }
                }
            }

            results.Dispose();
            knnContainer.Dispose();
        }

        /// <summary>
        /// 均匀网格分支：每帧把避障代理按 cell 分桶，查询每个代理半径内的邻居，
        /// 过滤后写入各自的 NeighbourBuffer。
        /// 临时内存（cellStart / cellItems / bestIndices / bestDistSq）由本方法分配并释放。
        /// </summary>
        void WriteNeighboursWithUniformGrid(
            in Grid _grid,
            in LocalTransform _gridTransform,
            in UniformGridQuerySettings _settings,
            int _maxNeighbours,
            NativeArray<Entity> _avAgEnts,
            NativeList<float3> _positions)
        {
            int agentCount = _avAgEnts.Length;

            // 导航网格覆盖的世界范围与左下角原点
            float worldWidth = _grid.NodeXCount * _grid.NodeSize;
            float worldHeight = _grid.NodeYCount * _grid.NodeSize;
            float3 origin = _gridTransform.Position
                - new float3(worldWidth * 0.5f, 0f, worldHeight * 0.5f);

            NeighbourGridLayout layout = AgentNeighbourBuckets.CreateLayout(
                origin, worldWidth, worldHeight, _settings.CellSize);

            NativeArray<int> cellStart = new NativeArray<int>(layout.TotalCellCount + 1, Allocator.Temp);
            NativeArray<int> cellItems = new NativeArray<int>(agentCount, Allocator.Temp);
            NativeArray<int> bestIndices = new NativeArray<int>(_maxNeighbours, Allocator.Temp);
            NativeArray<float> bestDistSq = new NativeArray<float>(_maxNeighbours, Allocator.Temp);

            NativeArray<float3> positions = _positions.AsArray();
            AgentNeighbourBuckets.Build(positions, layout, cellStart, cellItems);

            for (int i = 0; i < agentCount; i++)
            {
                int neighbourCount = AgentNeighbourBuckets.Query(
                    positions, layout, cellStart, cellItems,
                    i, _settings.QueryRadius, bestIndices, bestDistSq);

                Entity self = _avAgEnts[i];
                DynamicBuffer<NeighbourBuffer> neighbours = AllAgentsNeighbours[self];
                neighbours.Clear();

                for (int j = 0; j < neighbourCount; j++)
                {
                    Entity other = _avAgEnts[bestIndices[j]];

                    if (IsValidNeighbour(self, other))
                    {
                        neighbours.Add(new NeighbourBuffer() { Agent = other });
                    }
                }
            }

            cellStart.Dispose();
            cellItems.Dispose();
            bestIndices.Dispose();
            bestDistSq.Dispose();
        }

        /// <summary>
        /// 邻居过滤的唯一定义：KNN 与均匀网格桶两个分支都必须调用它，
        /// 这样 A/B 对比时唯一的变量是“候选从哪来”，而不是筛选条件也不同。
        /// 规则：
        /// 1. 排除自身；
        /// 2. 排除“已到达目的地且配置了 IgnoreAvoidanceWhenDestReached”的静止代理
        ///    （它们不再参与避障，也不该占用别人的邻居名额）。
        /// </summary>
        bool IsValidNeighbour(Entity _self, Entity _other)
        {
            if (_self == _other)
            {
                return false;
            }

            return !(AllPathFollowers[_other].DestinationReached &&
                     AllAgents[_other].IgnoreAvoidanceWhenDestReached);
        }
    }
}
