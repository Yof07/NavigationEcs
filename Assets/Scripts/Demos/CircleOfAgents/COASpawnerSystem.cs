using Unity.Burst;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Rendering;

namespace Navigation.Demos
{
    /// <summary>
    /// COA（Circle Of Agents）场景的代理生成系统。
    /// 游戏启动后仅执行一次：将代理按同心圆排列生成在场景中，
    /// 每个代理的目标点设置为圆心对面方向，形成对穿效果以测试大规模避障。
    /// </summary>
    //RequireMatchingQueriesForUpdate 是一个应用于 SystemBase 或 ISystem 的特性（Attribute）。
    //它的核心作用是进行性能优化：如果该系统管理的所有查询（EntityQuery）都找不到任何匹配的实体，
    //就自动跳过该系统的 OnUpdate 方法，不执行这一帧的更新逻辑
    [RequireMatchingQueriesForUpdate]
    [UpdateAfter(typeof(CheckObstaclesSystem))]
    [UpdateBefore(typeof(PathfindingRequestSystem))]
    partial struct COASpawnerSystem : ISystem
    {
        /// <summary>防止重复生成的标志，OnUpdate 只执行一次生成逻辑</summary>
        bool hasSpawned;

        //[BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            if (hasSpawned)
            {
                return;
            }

            EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

            Entity agentPrefab = Entity.Null;

            // 从场景中读取由 COASpawner 烘焙好的 Prefab Entity 引用
            foreach (var agentPrefHolder in SystemAPI.Query<AgentPrefabComponent>())
            {
                agentPrefab = agentPrefHolder.Value;
            }

            // Prefab 未就绪时跳过，等下一帧再试
            if (agentPrefab == Entity.Null)
            {
                ecb.Dispose();
                return;
            }

            // 到达目的地后停止参与避障，减少静止代理的计算开销
            Agent ag = state.EntityManager.GetComponentData<Agent>(agentPrefab);
            ag.IgnoreAvoidanceWhenDestReached = true;
            ecb.SetComponent(agentPrefab, ag);

            hasSpawned = true;

            // 环配置来自 COASpawner 的 Inspector 数组，Baker 烘成 buffer
            DynamicBuffer<SpawnRing> rings = SystemAPI.GetSingletonBuffer<SpawnRing>();

            // 出生点过滤用的网格快照（本系统排在 CheckObstacles 之后，节点已扫过）
            Grid grid = default;
            LocalTransform gridTransform = default;

            foreach (var (gridComponent, gridLocalTransform) in SystemAPI.Query<Grid, LocalTransform>())
            {
                grid = gridComponent;
                gridTransform = gridLocalTransform;
            }

            // 当恰好有一个实体拥有这个  DynamicBuffer<GridNodesBuffer> 时，才会被查询出来并返回，否则只会抛错
            DynamicBuffer<GridNodesBuffer> nodes = SystemAPI.GetSingletonBuffer<GridNodesBuffer>();

            int totalAgents = 0;
            int skippedAgents = 0;

            for (int j = 0; j < rings.Length; j++)
            {
                int radius = rings[j].Radius;
                int count = rings[j].Count;

                for (int i = 0; i < count; i++)
                {
                    // 将索引 i 均匀映射到 [0, 360) 度，用于圆周均布和 HSV 着色
                    float ang = (math.PI * 2) / count * i;
                    ang = math.degrees(ang);
                    ang = ang < 0 ? ang + 360 : ang;

                    // 用角度作为色相（H），生成彩虹色以区分不同代理
                    float3 newColor = new(ang / 360, 1, 1);
                    Color rgbColor = Color.HSVToRGB(newColor.x, newColor.y, newColor.z);
                    rgbColor.a = 1;

                    // 代理在圆周上的生成位置，Y=1 避免陷入地面
                    float3 spawnPos = new float3(
                        math.cos((math.PI * 2) / count * i) * radius,
                        1,
                        math.sin((math.PI * 2) / count * i) * radius);

                    // 出生点落在障碍里就跳过：否则代理一出生就在红格中，寻路也救不回来
                    if (!GridUtility.WorldPosToNode(spawnPos, grid, gridTransform, nodes).IsAccessible)
                    {
                        skippedAgents++;
                        continue;
                    }

                    // 面朝圆心（-spawnPos 方向），用于初始朝向
                    LocalTransform locTransform = new LocalTransform()
                    {
                        Position = spawnPos,
                        Rotation = quaternion.LookRotationSafe(-spawnPos, math.up()),
                        Scale = 1
                    };

                    // 目标点 = 当前位置沿正前方延伸 2 倍半径，即穿过圆心到对面
                    PathfindingRequester requester = new PathfindingRequester()
                    {
                        Destination = spawnPos + (locTransform.Forward() * radius * 2),
                        RequestEntity = Entity.Null,
                        RequestGiven = false,   // false = 立即触发寻路请求
                    };

                    // 设置代理的渲染颜色（通过 Entities Graphics 的 MaterialProperty）
                    URPMaterialPropertyBaseColor color = new URPMaterialPropertyBaseColor()
                    {
                        Value = new float4(rgbColor.r, rgbColor.g, rgbColor.b, rgbColor.a)
                    };

                    Entity e = ecb.Instantiate(agentPrefab);
                    ecb.SetComponent(e, locTransform);
                    ecb.SetComponent(e, requester);
                    ecb.AddComponent(e, color);
                    totalAgents++;
                }
            }

            ecb.Playback(state.EntityManager);
            ecb.Dispose();

            // 打印本次生成的代理总数
            Debug.Log($"[COA] 已生成代理数量：{totalAgents}（因出生点落在障碍上跳过 {skippedAgents} 个）");

            // 若需要查询场景中实际存活的 Agent Entity 数量，使用以下方式：
            // var query = state.GetEntityQuery(ComponentType.ReadOnly<Agent>());
            // Debug.Log($"[COA] 当前场景 Agent 数量：{query.CalculateEntityCount()}");
        }
    }
}
