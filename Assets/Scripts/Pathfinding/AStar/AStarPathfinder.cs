using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Navigation
{
    /// <summary>
    /// A* 算法的对外入口：为一批寻路操作调度并行搜索 Job。
    /// 具体搜索逻辑在 AStarSearch 里，本类只负责装配与调度。
    /// </summary>
    public static class AStarPathfinder
    {
        /// <summary>
        /// 为一个 Chunk 内的所有寻路操作调度并行 A*
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
            // 每个 Job 需要独立的节点数组副本（并行 Job 间不能共享写入）
            NativeArray<GridNode> newN = new NativeArray<GridNode>(_nodes.Length, Allocator.TempJob);
            NativeArray<GridNode>.Copy(_nodes, newN);

            AStarSearchJob searchJob = new AStarSearchJob()
            {
                OperationEntities = _chunk.GetNativeArray(_entityTypeHandle), // 本 Chunk 中的操作 Entity 列表
                JobAllOperations = _operations,
                JobAllResults = _results,
                JobAllPathPoints = _pathPoints,
                JobNodes = newN,
                JobGrid = _grid,
                JobGridLocalTransform = _gridLocalTransform,
            };

            // 调度并行 Job（每个操作分配到独立线程，批次大小 2）
            return searchJob.Schedule(_chunk.Count, 2, _inputDeps);
        }
    }

    /// <summary>
    /// 核心 A* 寻路算法实现。
    ///
    /// 算法流程：
    /// 1. 将起点和终点世界坐标转换为网格节点
    /// 2. 若终点不可通行，使用 NodeSnapping 寻找最近的可通行节点作为替代终点
    /// 3. 标准 A* 主循环：
    ///    a. 从 Open Set（最小堆）中取出 F 代价最小的节点
    ///    b. 若到达终点，回溯路径并返回
    ///    c. 遍历 8 方向邻居，计算 G 代价并更新 Open Set
    /// 4. 若未找到路径，返回最接近终点的节点的路径
    /// 5. 路径回溯：通过 PreviousNodeIndex 链从终点追溯到起点，反转后输出
    /// </summary>
    public struct AStarSearch
    {
        public Entity OperationEntity;

        public ComponentLookup<PathfindingOperation> AllOperations;
        public ComponentLookup<PathfindingResult> AllResults;
        public BufferLookup<PathBuffer> AllPathPoints;

        // A* Open Set（最小堆，按 F 代价排序）
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion]
        public NativeHashSet<GridNode> closedNodes;

        // A* Closed Set（哈希集合，O(1) 查找）
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion]
        public NativeMinHeap<GridNode> openNodes;

        public Grid Grid;
        public LocalTransform GridLocalTransform;

        // 工作节点数组（从主数组复制，A* 过程中修改 G/H/PreviousNodeIndex 值）
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion] public NativeArray<GridNode> Nodes;
        [NativeDisableContainerSafetyRestriction, DeallocateOnJobCompletion] public NativeArray<GridNode> NewNodes;

        public void Execute()
        {
            // 复制节点数组（每次寻路使用独立副本，不污染原始数据）
            NewNodes = new NativeArray<GridNode>(Nodes.Length, Allocator.Temp);
            NativeArray<GridNode>.Copy(Nodes, NewNodes);

            PathfindingOperation operation = AllOperations[OperationEntity];
            PathfindingResult operationResult = AllResults[OperationEntity];
            DynamicBuffer<PathBuffer> pathBuffer = AllPathPoints[OperationEntity];

            operationResult.PathFound = false;
            operationResult.FinishedSearch = false;

            // 将起点/终点世界坐标转换为网格节点
            GridNode startNode = GridUtility.WorldPosToNode(operation.StartPoint, Grid, GridLocalTransform, NewNodes);
            GridNode targetNode = GridUtility.WorldPosToNode(operation.TargetPoint, Grid, GridLocalTransform, NewNodes);

            // 记录搜索过程中距目标最近的节点（用于无法到达时回退）
            GridNode closestNode = new GridNode();
            int closestNodeDistFromGoal = int.MaxValue;
            bool targetFound = false;

            // ===== 终点不可通行时：BFS 寻找最近可通行节点 =====
            if (!targetNode.IsAccessible)
            {
                if (NodeSnapping.TryFindAccessibleTarget(targetNode, startNode, Grid, NewNodes, out GridNode newTargetNode))
                {
                    targetNode = newTargetNode; // 使用 BFS 找到的替代终点
                }
                else
                {
                    // 完全无法找到可通行的替代终点，终止搜索
                    operationResult.PathFound = false;
                    operationResult.FinishedSearch = true;
                    AllResults[OperationEntity] = operationResult;
                    return;
                }
            }

            // ===== 初始化 A* 数据结构 =====
            openNodes = new NativeMinHeap<GridNode>(Grid.NodeXCount * Grid.NodeYCount, Allocator.Temp) { };
            closedNodes = new NativeHashSet<GridNode>(0, Allocator.Temp);
            openNodes.Add(startNode); // 将起点加入 Open Set

            int d = Grid.NodeXCount > Grid.NodeYCount ? Grid.NodeXCount : Grid.NodeYCount;

            // ===== A* 主循环 =====
            while (openNodes.Length > 0)
            {
                // 取出 F 代价最小的节点（最小堆堆顶）
                GridNode currentNode = openNodes.PopFirstItem();
                closedNodes.Add(currentNode); // 加入 Closed Set

                // 到达终点：回溯路径并返回
                if (currentNode.X == targetNode.X && currentNode.Y == targetNode.Y)
                {
                    operationResult.PathFound = true;
                    operationResult.FinishedSearch = true;

                    RetracePath(startNode, currentNode, Grid, GridLocalTransform, NewNodes, pathBuffer, OperationEntity);
                    targetFound = true;
                    break;
                }

                // 记录距目标最近的节点（用于 GoToClosest 策略）
                int currNodeDistanceFromGoal = GridUtility.NodeDistance(currentNode, targetNode);
                if (currNodeDistanceFromGoal < closestNodeDistFromGoal)
                {
                    closestNodeDistFromGoal = currNodeDistanceFromGoal;
                    closestNode = currentNode;
                }

                // ===== 遍历 8 方向邻居 =====
                NativeArray<int2> neighboursOffsetIndecies = GridUtility.NodePossibleNeighbours(Allocator.Temp);

                foreach (int2 neighbourIndexOffset in neighboursOffsetIndecies)
                {
                    // 边界检查
                    if (neighbourIndexOffset.x + currentNode.X < 0 ||
                        neighbourIndexOffset.x + currentNode.X >= Grid.NodeXCount ||
                        neighbourIndexOffset.y + currentNode.Y < 0 ||
                        neighbourIndexOffset.y + currentNode.Y >= Grid.NodeYCount) continue;

                    // 计算邻居节点的一维索引
                    int neighbourIndex;
                    if (Grid.NodeXCount > Grid.NodeYCount)
                        neighbourIndex = neighbourIndexOffset.x + currentNode.X + ((neighbourIndexOffset.y + currentNode.Y) * d);
                    else neighbourIndex = neighbourIndexOffset.y + currentNode.Y + ((neighbourIndexOffset.x + currentNode.X) * d);

                    GridNode neighbour = NewNodes[neighbourIndex];

                    // 检查邻居是否在 Closed Set 中（已探索过）
                    bool neighbourInClosed = false;
                    foreach (GridNode closedNodeItem in closedNodes)
                    {
                        if (closedNodeItem.X == neighbour.X && closedNodeItem.Y == neighbour.Y)
                        {
                            neighbourInClosed = true;
                        }
                    }

                    // 跳过不可通行节点和已关闭节点
                    if (!neighbour.IsAccessible || neighbourInClosed)
                        continue;

                    // 计算经过当前节点到达邻居的代价（G = 父G + 移动代价 + 地形惩罚）
                    int movementCost = currentNode.GCost + GridUtility.NodeDistance(currentNode, neighbour) + neighbour.Penalty;

                    // 若新代价更低，或邻居尚未在 Open Set 中，更新邻居数据
                    if (movementCost < neighbour.GCost || !openNodes.Contains(neighbour))
                    {
                        neighbour.GCost = movementCost;
                        neighbour.HCost = GridUtility.NodeDistance(neighbour, targetNode);

                        // 记录父节点索引（路径回溯用）
                        int currentNodeIndex;
                        if (Grid.NodeXCount > Grid.NodeYCount)
                            currentNodeIndex = currentNode.X + currentNode.Y * d;
                        else currentNodeIndex = currentNode.Y + currentNode.X * d;

                        neighbour.PreviousNodeIndex = currentNodeIndex;

                        // 更新 Open Set 中已有的邻居节点（代价降低需要重新堆化）
                        for (int i = 0; i < openNodes.Length; i++)
                        {
                            if (openNodes.Items[i].Equals(neighbour))
                            {
                                int nodeHeapIndex = openNodes.Items[i].HeapIndex;
                                neighbour.HeapIndex = nodeHeapIndex;
                                openNodes.SetItemAt(neighbour, i);
                                break;
                            }
                        }

                        if (!openNodes.Contains(neighbour))
                        {
                            openNodes.Add(neighbour); // 新节点加入 Open Set
                        }
                        else
                        {
                            openNodes.UpdateItem(neighbour); // 已有节点更新优先级
                        }

                        NewNodes[neighbourIndex] = neighbour; // 更新工作节点数组
                    }
                }

                neighboursOffsetIndecies.Dispose();
            }

            // ===== 未找到路径：返回距目标最近的节点的路径 =====
            if (!targetFound)
            {
                operationResult.PathFound = true;
                operationResult.FinishedSearch = true;
                RetracePath(startNode, closestNode, Grid, GridLocalTransform, NewNodes, pathBuffer, OperationEntity);
            }

            AllResults[OperationEntity] = operationResult;
        }

        /// <summary>
        /// 路径回溯：从终点通过 PreviousNodeIndex 链追溯到起点，
        /// 然后反转顺序（起点→终点），将路径点世界坐标写入缓冲区
        /// </summary>
        void RetracePath(
            GridNode _startNode,
            GridNode _targetNode,
            Grid _grid,
            LocalTransform _gridLocalTransform,
            NativeArray<GridNode> _nodes,
            DynamicBuffer<PathBuffer> _pathPoints,
            Entity _agentEntity)
        {
            _pathPoints.Clear();

            GridNode pathCursor = _targetNode;
            NativeList<GridNode> pathNodes = new NativeList<GridNode>(Allocator.Temp);

            // 从终点向起点回溯（通过父节点链）
            while (!(pathCursor.X == _startNode.X && pathCursor.Y == _startNode.Y))
            {
                pathNodes.Add(pathCursor);
                pathCursor = _nodes[pathCursor.PreviousNodeIndex]; // 跳到父节点
            }

            // 反转路径（从起点到终点的正向顺序），转换为世界坐标
            for (int i = pathNodes.Length - 1; i >= 0; i--)
            {
                _pathPoints.Add(new PathBuffer()
                {
                    Point = GridUtility.GridCoordsToWorld(pathNodes[i].X, pathNodes[i].Y, _grid, _gridLocalTransform)
                });
            }

            // 最后追加精确的目标世界坐标（替代最近节点中心，减少最终误差）
            _pathPoints.Add(new PathBuffer() { Point = AllOperations[_agentEntity].TargetPoint });
        }
    }
}
