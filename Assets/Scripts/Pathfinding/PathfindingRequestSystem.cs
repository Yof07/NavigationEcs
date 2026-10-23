using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Collections;
using UnityEngine;

namespace Navigation
{
    /// <summary>
    /// 寻路请求调度系统，负责收集代理的寻路请求并以受控频率提交给 PathfindingSystem。
    ///
    /// 工作流程：
    /// 1. 检查是否有新的寻路请求（PathfindingRequester.RequestGiven == false）
    /// 2. 若配置了 DontPath 策略，检查起点和终点是否在同一岛屿，不同则跳过
    /// 3. 将合法请求加入全局操作队列（PathfindingRequestQueue）
    /// 4. 每帧从队列头部取出最多 MaxPathfindingOpsPerFrame 个操作，
    ///    创建对应的寻路操作 Entity（含 PathfindingOperation + PathfindingResult + PathBuffer）
    ///
    /// 系统顺序：在 PathfindingSystem 之前执行（队列生产者）
    /// </summary>
    [UpdateAfter(typeof(CheckNodeIslandSystem))]
    partial struct PathfindingRequestSystem : ISystem
    {
        // 组件查找缓存
        ComponentLookup<PathfindingRequester> AllRequesters;
        ComponentLookup<LocalTransform> AllLocalTransforms;
        ComponentLookup<PathfindingResult> AllPathfindingResults;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            AllRequesters = state.GetComponentLookup<PathfindingRequester>();
            AllPathfindingResults = state.GetComponentLookup<PathfindingResult>(true); // 只读
            AllLocalTransforms = state.GetComponentLookup<LocalTransform>(true);         // 只读
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            AllRequesters.Update(ref state);
            AllLocalTransforms.Update(ref state);
            AllPathfindingResults.Update(ref state);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            // 获取全局单例：寻路操作队列、节点岛屿ID、节点数组
            DynamicBuffer<PathfindingRequestQueue> operationsQueue = SystemAPI.GetSingletonBuffer<PathfindingRequestQueue>();
            DynamicBuffer<NodeIslandBuffer> nodeIslandIDs = SystemAPI.GetSingletonBuffer<NodeIslandBuffer>();
            DynamicBuffer<GridNodesBuffer> nodes = SystemAPI.GetSingletonBuffer<GridNodesBuffer>();

            PathfindingParameters pathfindingParameters = new PathfindingParameters();

            // 遍历所有同时拥有 Grid + LocalTransform + PathfindingParameters 三个组件的网格 entity
            // （不要求"仅拥有"这三个组件；工程中通常只有一个网格 entity，本循环实际只执行一次）
            // 每轮迭代解构出的是"当前匹配 entity"身上这三个组件的只读引用包装（RefRO），
            // 用 .ValueRO 读值，如需写回则要用 RefRW + .ValueRW
            foreach (var (grid, gridLocalTransform, pathfindingParams) in SystemAPI
                         .Query<RefRO<Grid>, RefRO<LocalTransform>, RefRO<PathfindingParameters>>())
            {
                pathfindingParameters = pathfindingParams.ValueRO;

                int d = grid.ValueRO.NodeXCount > grid.ValueRO.NodeYCount ? grid.ValueRO.NodeXCount : grid.ValueRO.NodeYCount;

                // DontPath 策略需要岛屿数据已就绪，否则等待
                if (grid.ValueRO.DifferentIslandsPolicy == DifferentIslandsPolicy.DontPath)
                {
                    // 如果不是每个节点的岛屿 id 都明确，即岛屿数据还被准备好，则退出
                    /*
                     * 因为岛屿系统的节点数据是在第二帧才有的，所以有可能第一帧执行这个函数的时候数据还没准备好，
                     * 因而要检测，否则 nodeIslandIDs 这个数组为空，后续读数组下标会越界
                     */
                    // 上面说的是"第一帧数据还没就绪"的正常情形；除此之外，
                    // 岛屿 id 数与节点数必须一一对应 —— 长度对不上就是数据坏了，再拿它做下标读
                    // 会越界或读到过期数据。所以下面这条判断改成有声：条件用 !=（多出来同样是坏），
                    // 命中就报错再 return，不静默放过。
                    if (nodeIslandIDs.Length != nodes.Length)
                    {
                        Debug.LogError($"[PathfindingRequestSystem] 岛屿数据长度 {nodeIslandIDs.Length} 与节点数 {nodes.Length} 不一致，本帧跳过请求处理");
                        return;
                    }
                }

                // 遍历所有有寻路请求的代理
                foreach (var (requester, localTransform, entity) in SystemAPI
                             .Query<RefRW<PathfindingRequester>, RefRO<LocalTransform>>().WithEntityAccess())
                {
                    
                    if (!requester.ValueRO.RequestGiven) // RequestGiven == false 表示有新请求
                    {
                        // ===== 岛屿连通性检查（DontPath 策略）=====
                        if (grid.ValueRO.DifferentIslandsPolicy == DifferentIslandsPolicy.DontPath)
                        {
                            // 将代理当前位置和目标位置转换为网格节点
                            GridNode startNode = GridUtility.WorldPosToNode(localTransform.ValueRO.Position, grid.ValueRO,
                                gridLocalTransform.ValueRO, nodes);

                            GridNode targetNode = GridUtility.WorldPosToNode(requester.ValueRO.Destination, grid.ValueRO,
                                gridLocalTransform.ValueRO, nodes);

                            // 计算起始节点和目标节点在数组中的一维索引
                            int startIndex = 0;
                            if (grid.ValueRO.NodeXCount > grid.ValueRO.NodeYCount)
                                startIndex = startNode.X + startNode.Y * d;
                            else startIndex = startNode.Y + startNode.X * d;

                            int targetIndex = 0;
                            if (grid.ValueRO.NodeXCount > grid.ValueRO.NodeYCount)
                                targetIndex = targetNode.X + targetNode.Y * d;
                            else targetIndex = targetNode.Y + targetNode.X * d;

                            // 如果起点和终点的岛屿 ID 不同，跳过此次寻路请求
                            if (nodeIslandIDs[startIndex].IslandID != nodeIslandIDs[targetIndex].IslandID)
                            {
                                requester.ValueRW.RequestGiven = true; // 标记为已处理（防止反复检测）
                                continue;
                            }
                        }
 
                        // ===== 添加或更新操作队列 =====
                        bool entityHasRequest = false;

                        // 检查队列中是否已有此代理的请求（更新而非重复添加）
                        for (int i = 0; i < operationsQueue.Length; i++)
                        {
                            if (operationsQueue[i].RequestingAgentEntity == entity)
                            {
                                // 更新已有请求的起点（代理可能已移动）
                                operationsQueue[i] = new PathfindingRequestQueue()
                                {
                                    RequestingAgentEntity = entity,
                                    Operation = new PathfindingOperation()
                                    {
                                        StartPoint = localTransform.ValueRO.Position,
                                        TargetPoint = requester.ValueRO.Destination
                                    }
                                };
                                entityHasRequest = true;
                                break;
                            }
                        }

                        // 队列中没有此代理的请求，新增
                        if (!entityHasRequest)
                        {
                            operationsQueue.Add(new PathfindingRequestQueue()
                            {
                                RequestingAgentEntity = entity,
                                Operation = new PathfindingOperation()
                                {
                                    StartPoint = localTransform.ValueRO.Position,
                                    TargetPoint = requester.ValueRO.Destination
                                }
                            });
                        }

                        requester.ValueRW.RequestGiven = true; // 标记请求已提交到队列
                    }
                }
            }

            // ===== 时间分片：每帧最多取出 MaxPathfindingOpsPerFrame 个操作执行 =====
            int k = 0;

            while (k < pathfindingParameters.MaxPathfindingOpsPerFrame && operationsQueue.Length > 0)
            {
                // 获取队列头部的请求（FIFO 先进先出）
                PathfindingRequester requester = AllRequesters[operationsQueue[0].RequestingAgentEntity];

                if (SystemAPI.Exists(requester.RequestEntity))
                {
                    // 操作 Entity 已存在：检查是否还在处理中，若是则更新（代理位置可能已变化）
                    PathfindingResult result = AllPathfindingResults[requester.RequestEntity];
                    if (!result.FinishedSearch)
                    {
                        // 尚未完成，更新起点坐标（使用当前实时位置）
                        ecb.SetComponent(requester.RequestEntity, new PathfindingResult());
                        ecb.SetComponent(requester.RequestEntity, new PathfindingOperation()
                        {
                            StartPoint = AllLocalTransforms[operationsQueue[0].RequestingAgentEntity].Position,
                            TargetPoint = requester.Destination
                        });
                    }
                }
                else
                {
                    // 操作 Entity 不存在：创建新 Entity 并添加寻路所需组件
                    Entity e = state.EntityManager.CreateEntity();
                    requester.RequestEntity = e; // 记录操作 Entity 引用到代理

                    ecb.AddComponent<PathfindingResult>(e);  // 结果标志（初始 false/false）
                    ecb.AddComponent<PathfindingOperation>(e, new PathfindingOperation()
                    {
                        StartPoint = AllLocalTransforms[operationsQueue[0].RequestingAgentEntity].Position,
                        TargetPoint = requester.Destination
                    });
                    ecb.AddBuffer<PathBuffer>(e);   // 路径点缓冲区
                }

                // 将更新后的 requester（含新的 RequestEntity）写回代理
                AllRequesters[operationsQueue[0].RequestingAgentEntity] = requester;

                operationsQueue.RemoveAt(0); // 从队列移除已处理的请求
                k++;
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
        }
    }
}
