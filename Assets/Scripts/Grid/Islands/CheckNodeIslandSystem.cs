using Unity.Burst;
using Unity.Entities;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Physics;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;

namespace Navigation
{
    /// <summary>
    /// 网格连通区域（岛屿）分析系统
    ///
    /// 职责：
    /// 将网格中所有相互连通的可通行节点划分为同一个"岛屿"（Island），
    /// 并为每个节点记录其所属岛屿的ID
    ///
    /// 算法：BFS（广度优先搜索）洪水填充
    /// - 遍历所有节点，对每个未访问的可通行节点启动一次BFS
    /// - BFS扩展到所有4方向相邻的可通行节点，赋予相同的IslandID
    /// - 每完成一次BFS将IslandID递增
    ///
    /// 用途：PathfindingRequestSystem在提交寻路请求前检查起点和终点的IslandID，
    /// 若不同则根据DifferentIslandsPolicy决定是否寻路
    ///
    /// 触发条件：Grid.RefreshIslands = true（障碍物扫描完成后自动触发）
    ///
    /// —— 关键设计说明 ——
    /// · 与CheckObstaclesSystem的初值握手是刻意的：本系统mHasCheckedForIslands
    ///   初值为true（第一帧故意不跑），必须等CheckObstaclesSystem扫描完障碍物、
    ///   置起RefreshIslands后才开始工作若初值反了（第一帧就跑），障碍物尚未标记、
    ///   整张网格会被算成一个完整岛屿，岛屿数据从第一帧起就是错的
    ///   即CheckNodeIslandSystem依赖CheckObstaclesSystem把节点是否可通行标注出来，
    ///   才能在此基础上进行岛屿的划分，所以CheckObstaclesSystem完成工作后，
    ///   通过grid.RefreshIslands这个字段来允许更新
    /// · 用显式NativeQueue做BFS而非递归：大网格下递归洪水填充会栈溢出
    /// · 不可通行节点在BFS触达时也会被写入当前IslandID（相当于岛屿的边界墙），
    ///   但不继续向四周扩展可通行节点靠visited保证只被写入一次，不受影响
    /// </summary>
    [UpdateAfter(typeof(CheckObstaclesSystem))]
    partial struct CheckNodeIslandSystem : ISystem
    {
        // 是否已完成岛屿分析（避免重复计算）
        private bool mHasCheckedForIslands;

        /// <summary>
        /// 系统创建时初始化：把岛屿分析标志初值设为true（第一帧不执行）
        ///
        /// 初值true是刻意的跨系统握手约定，与CheckObstaclesSystem的
        /// mHasCheckedForObstacles = false恰好相反：
        /// 障碍物扫描第一帧无条件先跑一遍，扫完把Grid.RefreshIslands置起，
        /// 本系统在那一帧检测到标志后才开始第一次岛屿分析——保证BFS看到的
        /// 是已经标记好可通行性的节点数据
        /// </summary>
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            // 初始化时标记为已分析（等待RefreshIslands触发才执行）
            mHasCheckedForIslands = true;
        }

        /// <summary>
        /// 每帧执行：在网格需要时对所有节点做一次BFS洪水填充，重新划分岛屿
        ///
        /// 执行时机：
        /// 1. 未完成过分析（mHasCheckedForIslands == false）才执行；
        /// 2. CheckObstaclesSystem扫描完成后置起RefreshIslands，
        ///    本系统检测到后清除标志并开始重算
        ///
        /// 结果写入网格实体上的NodeIslandBuffer缓冲区，
        /// 与GridNodesBuffer按相同的一维索引一一对应
        /// </summary>
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            int currentIslandID = 0; // 岛屿ID从0开始递增

            foreach (var (grid, entity) in SystemAPI.Query<RefRW<Grid>>().WithEntityAccess())
            {
                // 检测到RefreshIslands请求时，重置分析状态
                if (grid.ValueRO.RefreshIslands)
                {
                    grid.ValueRW.RefreshIslands = false;
                    mHasCheckedForIslands = false;
                }

                if (!mHasCheckedForIslands)     // 防止重复计算
                {
                    DynamicBuffer<GridNodesBuffer> nodes = SystemAPI.GetBuffer<GridNodesBuffer>(entity);
                    DynamicBuffer<NodeIslandBuffer> nodesIDs = SystemAPI.GetBuffer<NodeIslandBuffer>(entity);
                    int nodeXCount = grid.ValueRO.NodeXCount;
                    int nodeYCount = grid.ValueRO.NodeYCount;

                    // 确保IslandID缓冲区大小与节点数一致
                    // （Baker创建时初始为空，这里第一次运行时扩容到位）
                    nodesIDs.Length = nodeXCount * nodeYCount;

                    int d = nodeXCount > nodeYCount ? nodeXCount : nodeYCount;
                    // 访问标记数组，防止BFS重复访问同一节点
                    // Allocator.Temp：一次性的帧内临时分配，分析结束自动随帧销毁
                    NativeArray<bool> visited = new NativeArray<bool>(nodeXCount * nodeYCount, Allocator.Temp);

                    // 遍历所有节点，对每个未访问的可通行节点启动一次BFS
                    for (int x = 0; x < nodeXCount; x++)
                    {
                        for (int y = 0; y < nodeYCount; y++)
                        {
                            // 计算节点的一维索引（全工程统一约定）
                            int index;
                            if (nodeXCount > nodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            // 只对"未访问 + 可通行"的节点启动新的BFS（发现新岛屿）
                            if (!visited[index] && nodes[index].Node.IsAccessible)
                            {
                                CheckIslandDFS(x, y, nodeXCount, nodeYCount, d, nodes, nodesIDs, visited, currentIslandID);
                                currentIslandID++; // 每个连通区域分配唯一ID
                            }
                        }
                    }

                    mHasCheckedForIslands = true;
                }
            }
        }

        /// <summary>
        /// BFS洪水填充：从 (_x, _y) 出发，将所有与其连通的可通行节点
        /// 标记为相同的IslandID（_currentIslandID）
        ///
        /// 实现细节：
        /// - 不可通行节点也赋予当前IslandID（用于边界标记），但不继续扩展
        /// - 4方向扩展（上下左右），不考虑对角连通性
        ///
        /// 注意：方法名沿用了原实现的CheckIslandDFS命名，但实际是BFS
        /// （显式队列逐层扩展），不是递归DFS
        /// </summary>
        private void CheckIslandDFS(int _x, int _y, int _nodeXCount, int _nodeYCount, int _d,
            DynamicBuffer<GridNodesBuffer> _nodes,
            DynamicBuffer<NodeIslandBuffer> _nodesIDs,
            NativeArray<bool> _visited, int _currentIslandID)
        {
            // 使用显式队列实现BFS（避免递归深度溢出）
            NativeQueue<(int, int)> nodeQueue = new NativeQueue<(int, int)>(Allocator.Temp);
            nodeQueue.Enqueue((_x, _y));

            while (nodeQueue.Count > 0)
            {
                (int, int) currCorrds = nodeQueue.Dequeue();

                // 边界检查（超出网格范围的坐标直接跳过）
                if (currCorrds.Item1 < 0 || currCorrds.Item1 >= _nodeXCount ||
                    currCorrds.Item2 < 0 || currCorrds.Item2 >= _nodeYCount) continue;

                // 计算一维索引
                int currIndex;
                if (_nodeXCount > _nodeYCount)
                    currIndex = currCorrds.Item1 + currCorrds.Item2 * _d;
                else currIndex = currCorrds.Item2 + currCorrds.Item1 * _d;

                if (_visited[currIndex]) continue; // 已访问，跳过

                // 不可通行节点：赋予IslandID但不继续扩展（边界墙壁）
                if (!_nodes[currIndex].Node.IsAccessible)
                {
                    _nodesIDs[currIndex] = new NodeIslandBuffer() { IslandID = _currentIslandID };
                    continue;
                }

                // 标记当前节点已访问并赋予IslandID
                _visited[currIndex] = true;
                _nodesIDs[currIndex] = new NodeIslandBuffer() { IslandID = _currentIslandID };

                // 向4方向扩展（BFS）
                nodeQueue.Enqueue((currCorrds.Item1 + 1, currCorrds.Item2)); // 右
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 + 1)); // 上
                nodeQueue.Enqueue((currCorrds.Item1 - 1, currCorrds.Item2)); // 左
                nodeQueue.Enqueue((currCorrds.Item1, currCorrds.Item2 - 1)); // 下
            }
        }
    }
}
