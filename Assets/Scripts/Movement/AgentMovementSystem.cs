using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Rendering;
using KNN;
using KNN.Jobs;
using Unity.Jobs;
using Unity.Physics;
using UnityEngine;

namespace Navigation
{
    /// <summary>
    /// 代理移动系统，系统的核心执行层，每帧负责：
    ///
    /// 1. 路径结果同步：
    ///    检查各代理对应的寻路操作 Entity 是否完成，若完成则将路径点复制到代理自身缓冲区
    ///
    /// 2. 目标追踪（ConstantTargetFollower）：
    ///    检测目标 Entity 位置变化，动态触发重新寻路
    ///
    /// 3. 无避障代理的移动：
    ///    简单沿路径点前进，到达路径点后切换到下一个，并通过物理射线防止穿越障碍物
    ///
    /// 4. RVO 避障代理的移动（核心算法）：
    ///    a. 使用 KD-Tree（KNN）查找每个代理的 K 近邻
    ///    b. 为每个邻居计算 ORCA（最优互惠碰撞避免）速度约束线
    ///    c. 通过线性规划在所有约束下找最优速度
    ///    d. 物理射线检测阻止穿越墙壁
    /// </summary>
    [UpdateAfter(typeof(PathfindingSystem))]
    partial struct AgentMovementSystem : ISystem
    {
        // 各组件的查找缓存（在 OnCreate 中初始化，每帧在 OnUpdate 中 Update）
        public ComponentLookup<PathfindingResult> AllResults;
        public BufferLookup<PathBuffer> AllPathPoints;
        public ComponentLookup<ConstantTargetFollower> AllConstantFollowers;

        public ComponentLookup<Agent> AllAgents;
        public ComponentLookup<LocalTransform> AllLocalTransforms;
        public ComponentLookup<PathFollower> AllPathFollowers;
        public BufferLookup<NeighbourBuffer> AllAgentsNeighbours;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            AllResults = state.GetComponentLookup<PathfindingResult>();
            AllPathPoints = state.GetBufferLookup<PathBuffer>();
            AllConstantFollowers = state.GetComponentLookup<ConstantTargetFollower>();

            AllAgents = state.GetComponentLookup<Agent>();
            AllLocalTransforms = state.GetComponentLookup<LocalTransform>();
            AllPathFollowers = state.GetComponentLookup<PathFollower>();
            AllAgentsNeighbours = state.GetBufferLookup<NeighbourBuffer>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // 每帧更新所有 Lookup（确保访问最新数据）
            AllResults.Update(ref state);
            AllPathPoints.Update(ref state);
            AllConstantFollowers.Update(ref state);
            AllAgents.Update(ref state);
            AllLocalTransforms.Update(ref state);
            AllPathFollowers.Update(ref state);
            AllAgentsNeighbours.Update(ref state);

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);
            NativeList<Entity> allAgents = new NativeList<Entity>(Allocator.Temp);

            // 获取 Unity Physics World 单例（用于射线检测）
            EntityQueryBuilder builder = new EntityQueryBuilder(Allocator.Temp).WithAll<PhysicsWorldSingleton>();
            EntityQuery singletonQuery = state.EntityManager.CreateEntityQuery(builder);
            var physicsWorld = singletonQuery.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;

            // 获取 Grid 参数（节点大小等）
            builder = new EntityQueryBuilder(Allocator.Temp).WithAll<Grid, LocalTransform, NeighbourQuerySettings, UniformGridQuerySettings>();
            EntityQuery gridQuery = state.EntityManager.CreateEntityQuery(builder);
            var grid = gridQuery.GetSingleton<Grid>();
            var gridTransform = gridQuery.GetSingleton<LocalTransform>();
            var neighbourQuerySettings = gridQuery.GetSingleton<NeighbourQuerySettings>();
            var uniformGridQuerySettings = gridQuery.GetSingleton<UniformGridQuerySettings>();

            singletonQuery.Dispose();
            gridQuery.Dispose();

            // =============================================
            // 阶段 1：路径结果同步 + 目标追踪
            // =============================================
            foreach (var (pathfindingRequester, pathFollower, agentEntity) in SystemAPI
                .Query<RefRW<PathfindingRequester>, RefRW<PathFollower>>()
                .WithAll<PathBuffer>().WithEntityAccess())
            {
                allAgents.Add(agentEntity);

                // ----- 1a. 检查寻路操作是否完成，完成则复制路径 -----
                if (pathfindingRequester.ValueRO.RequestEntity != Entity.Null &&
                    SystemAPI.Exists(pathfindingRequester.ValueRO.RequestEntity))
                {
                    PathfindingResult reqResult = AllResults[pathfindingRequester.ValueRO.RequestEntity];

                    if (reqResult.FinishedSearch)
                    {
                        if (reqResult.PathFound)
                        {
                            // 从操作 Entity 的路径缓冲区复制路径点到代理的路径缓冲区
                            DynamicBuffer<PathBuffer> reqPathPoints = AllPathPoints[pathfindingRequester.ValueRO.RequestEntity];
                            DynamicBuffer<PathBuffer> agentPathPoints = state.EntityManager.GetBuffer<PathBuffer>(agentEntity);

                            pathFollower.ValueRW.PathAvailable = true;
                            pathFollower.ValueRW.TargetIndex = 0;       // 从路径起点开始追踪
                            pathFollower.ValueRW.DestinationReached = false;

                            agentPathPoints.Clear();
                            for (int i = 0; i < reqPathPoints.Length; i++)
                            {
                                agentPathPoints.Add(new PathBuffer() { Point = reqPathPoints[i].Point });
                            }
                        }

                        // 路径已读取，销毁临时操作 Entity（清理内存）
                        ecb.DestroyEntity(pathfindingRequester.ValueRO.RequestEntity);
                    }
                }

                // ----- 1b. 持续目标追踪：检测目标位置变化，触发重新寻路 -----
                if (AllConstantFollowers.HasComponent(agentEntity))
                {
                    ConstantTargetFollower constantTargetFollower = AllConstantFollowers[agentEntity];

                    if (constantTargetFollower.Target != Entity.Null &&
                        SystemAPI.Exists(constantTargetFollower.Target))
                    {
                        if (AllLocalTransforms.HasComponent(constantTargetFollower.Target))
                        {
                            if (constantTargetFollower.RecalculateOnStationary)
                            {
                                // 无论目标是否移动都重新寻路
                                pathfindingRequester.ValueRW.RequestGiven = false;
                                pathfindingRequester.ValueRW.Destination = AllLocalTransforms[constantTargetFollower.Target].Position;

                                constantTargetFollower.TargetLastPosition = AllLocalTransforms[constantTargetFollower.Target].Position;
                                AllConstantFollowers[agentEntity] = constantTargetFollower;
                            }
                            else
                            {
                                // 仅在目标移动超过阈值（0.01 单位距离²）时重新寻路
                                if (math.distancesq(AllLocalTransforms[constantTargetFollower.Target].Position,
                                    constantTargetFollower.TargetLastPosition) > 0.01f)
                                {
                                    pathfindingRequester.ValueRW.RequestGiven = false;
                                    pathfindingRequester.ValueRW.Destination = AllLocalTransforms[constantTargetFollower.Target].Position;

                                    constantTargetFollower.TargetLastPosition = AllLocalTransforms[constantTargetFollower.Target].Position;
                                    AllConstantFollowers[agentEntity] = constantTargetFollower;
                                }
                            }
                        }
                    }
                }
            }

            // =============================================
            // 阶段 2：无避障代理的移动（WithAbsent<AvoidanceAgent>）
            // =============================================
            foreach (var (agent, 
                         pathFollower, 
                         agentTransform, 
                         pathRequester, 
                         agentEntity) in SystemAPI
                .Query<RefRW<Agent>, 
                    RefRW<PathFollower>, 
                    RefRW<LocalTransform>, 
                    RefRO<PathfindingRequester>>()
                .WithAll<PathBuffer>().WithAbsent<AvoidanceAgent>().WithEntityAccess())
            {
                if (!pathFollower.ValueRO.PathAvailable || pathFollower.ValueRO.DestinationReached) continue;

                // 强制 Y 轴固定在代理中心高度（防止 Y 轴漂移）
                PathFollowing.ClampToAgentHeight(ref agentTransform.ValueRW, agent.ValueRO);

                DynamicBuffer<PathBuffer> agentPath = state.EntityManager.GetBuffer<PathBuffer>(agentEntity);

                // ----- 路径点切换逻辑（最后一个点用 StoppingDistance 阈值）-----
                PathFollowing.AdvanceIndex(ref pathFollower.ValueRW, agent.ValueRO, agentPath,
                    agentTransform.ValueRO.Position, grid.NodeSize,
                    agent.ValueRO.Radius + agent.ValueRO.StoppingDistance);

                // 索引超出路径长度：到达终点
                if (pathFollower.ValueRO.TargetIndex > agentPath.Length - 1)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                // 检查是否足够接近最终目的地（精确停止）
                if (PathFollowing.IsFinalDestinationReached(pathRequester.ValueRO, agent.ValueRO, agentTransform.ValueRO.Position))
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                // 计算朝向当前路径点的速度
                float3 destination = PathFollowing.CurrentPoint(agentPath, agent.ValueRO, pathFollower.ValueRO.TargetIndex);
                float3 destinationDir = math.normalize(destination - agentTransform.ValueRO.Position);
                agent.ValueRW.CurrentVelocity = destinationDir * agent.ValueRO.MaxSpeed;

                // ----- 障碍物射线检测（防止穿墙）-----
                var collFilter = new CollisionFilter()
                {
                    BelongsTo = ~0u,
                    CollidesWith = (uint)grid.InaccessibleLMIndex, // 只检测不可通行层
                    GroupIndex = 0
                };

                bool obsAhead = false;

                // 左中右三条平行射线：只有正前方一条时，斜向贴墙仍会挤进去
                float3 rayOrigin = PathFollowing.ObstacleRayOrigin(agentTransform.ValueRO.Position, agent.ValueRO);
                float3 sideOffset = agentTransform.ValueRO.Right() * (agent.ValueRO.Radius * 0.5f);

                for (int i = -1; i <= 1; i++)
                {
                    float3 currentOffset = sideOffset * i;

                    obsAhead = physicsWorld.CastRay(new RaycastInput()
                    {
                        Start = rayOrigin + currentOffset,
                        End = rayOrigin + (destinationDir * agent.ValueRO.Radius) + currentOffset,
                        Filter = collFilter
                    });

                    if (obsAhead) break;
                }

                if (!obsAhead)
                {
                    // 无障碍：更新位置
                    agentTransform.ValueRW.Position += agent.ValueRW.CurrentVelocity * SystemAPI.Time.DeltaTime;

                    // 平滑旋转朝向移动方向
                    PathFollowing.ApplyRotation(ref agentTransform.ValueRW, destinationDir,
                        agent.ValueRO.TurnSpeed, SystemAPI.Time.DeltaTime);
                }
            }

            // =============================================
            // 阶段 3：邻居查询（仅限有避障代理时）
            //
            // 输出产物（供阶段 4 消费）：每个避障代理的 NeighbourBuffer
            //   - 内容：该代理的邻居代理 Entity，数量上限为 NeighbourQuerySettings.MaxNeighbours
            //   - 已排除自身；已过滤"到达目的地且 IgnoreAvoidanceWhenDestReached"的静止代理
            //   - 每次整体 Clear 后重写，不跨帧累积；邻居的位置需阶段 4 自行从
            //     AllLocalTransforms 取（缓冲区里只存 Entity，不存坐标）
            //   - 实现由 NeighbourQuerySettings.Method 选择：
            //     KdTree = 精确最近 k 个；UniformGrid = 半径内最多 k 个
            // =============================================

            // 要被 Job 线程用，内存活到 Job 线程用完
            NativeList<float3> avoidanceAgentsPositions = new NativeList<float3>(0, Allocator.TempJob);    

            EntityQuery avoidanceAgents = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<LocalTransform, AvoidanceAgent>()
                .Build(ref state);

            // 如果没有任何避障代理，跳过邻居查询阶段
            if (avoidanceAgents.IsEmpty)
            {
                avoidanceAgentsPositions.Dispose();
                ecb.Playback(state.EntityManager);
                ecb.Dispose();
                allAgents.Dispose();
                return;
            }

            // 收集所有避障代理的世界坐标
            NativeArray<Entity> avAgEnts = avoidanceAgents.ToEntityArray(Allocator.Temp);
            for (int i = 0; i < avAgEnts.Length; i++)
            {
                avoidanceAgentsPositions.Add(AllLocalTransforms[avAgEnts[i]].Position);
            }

            // 使用KdTree 或 UniformGrid 查找邻居
            if (neighbourQuerySettings.Method == NeighbourQueryMethod.KdTree)
            {
                int kNeighbours = neighbourQuerySettings.MaxNeighbours + 1; // KNN 结果含自身
                WriteNeighboursWithKdTree(avoidanceAgentsPositions, kNeighbours, avAgEnts);
            }
            else
            {
                WriteNeighboursWithUniformGrid(
                    grid,
                    gridTransform,
                    uniformGridQuerySettings,
                    neighbourQuerySettings.MaxNeighbours,
                    avAgEnts,
                    avoidanceAgentsPositions);
            }

            // =============================================
            // 阶段 4：RVO 避障代理的移动
            // =============================================
            foreach (var (agent, 
                         pathFollower, 
                         agentAvoidance, 
                         agentTransform, 
                         pathRequester, 
                         agentEntity) in SystemAPI
                .Query<RefRW<Agent>, 
                    RefRW<PathFollower>, 
                    RefRW<AvoidanceAgent>, 
                    RefRW<LocalTransform>, 
                    RefRO<PathfindingRequester>>()
                .WithAll<PathBuffer>().WithEntityAccess())
            {
                if (!pathFollower.ValueRO.PathAvailable || pathFollower.ValueRO.DestinationReached) continue;

                // Y 轴固定
                PathFollowing.ClampToAgentHeight(ref agentTransform.ValueRW, agent.ValueRO);

                DynamicBuffer<NeighbourBuffer> agentNeighbours = state.EntityManager.GetBuffer<NeighbourBuffer>(agentEntity);
                DynamicBuffer<PathBuffer> agentPath = state.EntityManager.GetBuffer<PathBuffer>(agentEntity);

                // ----- 路径点切换逻辑（最后一个点只用 Radius，与阶段 2 不同）-----
                PathFollowing.AdvanceIndex(ref pathFollower.ValueRW, agent.ValueRO, agentPath,
                    agentTransform.ValueRO.Position, grid.NodeSize, agent.ValueRO.Radius);

                if (pathFollower.ValueRO.TargetIndex > agentPath.Length - 1)
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                if (PathFollowing.IsFinalDestinationReached(pathRequester.ValueRO, agent.ValueRO, agentTransform.ValueRO.Position))
                {
                    pathFollower.ValueRW.DestinationReached = true;
                    continue;
                }

                float3 destination = PathFollowing.CurrentPoint(agentPath, agent.ValueRO, pathFollower.ValueRO.TargetIndex);

                float2 newVelocity = float2.zero;
                NativeList<OrcaLine> orcaLines = new NativeList<OrcaLine>(Allocator.Temp);

                // 理想速度方向（单位向量）：朝向当前路径点
                float2 optVel = OrcaSolver.V2(math.normalize((destination - agentTransform.ValueRO.Position)));

                // 执行 ORCA 速度计算（算法本体在 Movement/Avoidance/OrcaSolver.cs）
                OrcaSolver.CalculateFinalVelocity(orcaLines, AllAgents, AllLocalTransforms, agent.ValueRW,
                    agentAvoidance.ValueRW, agentTransform.ValueRW,
                    destination, agentNeighbours, SystemAPI.Time.DeltaTime, optVel, ref newVelocity);

                float3 newVel3 = new float3(newVelocity.x, 0, newVelocity.y);
                float3 destinationDir = math.normalize(newVel3);

                // ----- 障碍物射线检测 -----
                var collFilter = new CollisionFilter()
                {
                    BelongsTo = ~0u,
                    CollidesWith = (uint)grid.InaccessibleLMIndex,
                    GroupIndex = 0
                };

                bool obsAhead = false;

                if (math.length(agent.ValueRO.CurrentVelocity) > 0)
                {
                    // 左中右三条平行射线（同阶段 2）
                    float3 rayOrigin = PathFollowing.ObstacleRayOrigin(agentTransform.ValueRO.Position, agent.ValueRO);
                    float3 sideOffset = agentTransform.ValueRO.Right() * (agent.ValueRO.Radius * 0.5f);
                    float3 moveDir = math.normalize(agent.ValueRW.CurrentVelocity);

                    for (int i = -1; i <= 1; i++)
                    {
                        float3 currentOffset = sideOffset * i;

                        obsAhead = physicsWorld.CastRay(new RaycastInput()
                        {
                            Start = rayOrigin + currentOffset,
                            End = rayOrigin + (moveDir * agent.ValueRO.Radius) + currentOffset,
                            Filter = collFilter
                        });

                        if (obsAhead) break;
                    }
                }

                if (obsAhead)
                {
                    newVel3 = float3.zero; // 前方有障碍物，停止移动
                }
                else
                {
                    // 平滑旋转朝向移动方向
                    PathFollowing.ApplyRotation(ref agentTransform.ValueRW, destinationDir,
                        agent.ValueRO.TurnSpeed, SystemAPI.Time.DeltaTime);
                }

                // 更新位置和速度
                agentTransform.ValueRW.Position += newVel3 * SystemAPI.Time.DeltaTime;
                agent.ValueRW.CurrentVelocity = newVel3;

                orcaLines.Dispose();
            }

            // 释放所有临时内存
            avoidanceAgentsPositions.Dispose();
            avAgEnts.Dispose();

            ecb.Playback(state.EntityManager);
            ecb.Dispose();
            allAgents.Dispose();
        }
    }
}
