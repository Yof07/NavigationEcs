using Unity.Collections;

namespace Navigation
{
    /// <summary>
    /// 节点吸附：目标落在不可通行节点时，从目标向外 BFS，寻找范围内最近的可通行节点。
    /// A* 与流场共用这一份逻辑（原寄居在 AStarSearch.ExpandInaccessibleBFS）。
    /// </summary>
    public static class NodeSnapping
    {
        /// <summary>
        /// 从 _targetNode 向外 BFS，在 TargetInaccessibleSearchTolerance * 10 的节点距离内
        /// 找到距 _startingNode 最近的可通行节点。
        /// 找不到返回 false，_closestNode 无意义。
        /// </summary>
        public static bool TryFindAccessibleTarget(
            GridNode _targetNode,
            GridNode _startingNode,
            in Grid _grid,
            NativeArray<GridNode> _nodes,
            out GridNode _closestNode)
        {
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;
            NativeArray<bool> visited = new NativeArray<bool>(_grid.NodeXCount * _grid.NodeYCount, Allocator.Temp);
            NativeQueue<(int, int)> nodeQueue = new NativeQueue<(int, int)>(Allocator.Temp);
            nodeQueue.Enqueue((_targetNode.X, _targetNode.Y));

            bool foundClosestTarget = false;
            int closestDistance = int.MaxValue;
            _closestNode = new GridNode();

            while (nodeQueue.Count > 0)
            {
                (int, int) currCorrds = nodeQueue.Dequeue();

                int currIndex;
                if (_grid.NodeXCount > _grid.NodeYCount)
                    currIndex = currCorrds.Item1 + currCorrds.Item2 * d;
                else currIndex = currCorrds.Item2 + currCorrds.Item1 * d;

                // 边界检查 + 搜索范围限制（TargetInaccessibleSearchTolerance * 10 个节点距离）
                if (currCorrds.Item1 < 0 || currCorrds.Item1 >= _grid.NodeXCount ||
                    currCorrds.Item2 < 0 || currCorrds.Item2 >= _grid.NodeYCount ||
                    GridUtility.NodeDistance(_targetNode, _nodes[currIndex]) > _grid.TargetInaccessibleSearchTolerance * 10) continue;

                if (visited[currIndex]) continue;

                // 找到可通行节点：记录距起点最近的一个
                if (_nodes[currIndex].IsAccessible)
                {
                    foundClosestTarget = true;
                    int dist = GridUtility.NodeDistance(_startingNode, _nodes[currIndex]);
                    if (dist < closestDistance)
                    {
                        _closestNode = _nodes[currIndex];
                        closestDistance = dist;
                    }
                    continue; // 不继续扩展（可通行节点是终点）
                }

                visited[currIndex] = true;

                // 向4方向扩展（只扩展不可通行节点内部，寻找边界上的可通行节点）
                nodeQueue.Enqueue((currCorrds.Item1 + 1, currCorrds.Item2));
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 + 1));
                nodeQueue.Enqueue((currCorrds.Item1 - 1, currCorrds.Item2));
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 - 1));
            }

            return foundClosestTarget;
        }
    }
}
