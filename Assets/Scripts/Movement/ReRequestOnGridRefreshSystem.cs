using Unity.Burst;
using Unity.Entities;

namespace Navigation
{
    /// <summary>
    /// 「网格数据变了」的订阅者：把未到达代理的 RequestGiven 置 false，让它们重新排队。
    ///
    /// 定位：排在 CheckObstaclesSystem（①，发布 Grid.Version）之后、
    /// PathfindingRequestSystem（④，收集请求）之前，所以**同帧**生效；
    /// 而它自己属于 Movement 模块，能直接读 PathFollower，不需要 Grid 反向依赖上层。
    ///
    /// 为什么是独立系统而不是塞进别的系统：
    /// - 放进 CheckObstacles（Grid）：会让 Grid 引用 Pathfinding/Movement 的类型；
    /// - 放进 Request（Pathfinding）：需要跨层读 PathFollower；
    /// - 放进 AgentMovement（⑥）：Request 已经跑过，晚一帧。
    /// 后续若有别的「网格变更」订阅者（流场表作废、UI、调试可视化），也在这里接。
    /// </summary>
    [UpdateAfter(typeof(CheckObstaclesSystem))]
    [UpdateBefore(typeof(PathfindingRequestSystem))]
    partial struct ReRequestOnGridRefreshSystem : ISystem
    {
        // 已处理到的网格数据版本。
        private int m_LastConsumedVersion;

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            foreach (var grid in SystemAPI.Query<RefRO<Grid>>())
            {
                int version = grid.ValueRO.Version;

                // 版本没变：网格没变，跳过
                if (version == m_LastConsumedVersion)
                {
                    continue;
                }

                m_LastConsumedVersion = version;

                if (!grid.ValueRO.RecalcPathsAfterGridRefresh)
                {
                    continue;
                }

                foreach (var (requester, follower) in SystemAPI
                             .Query<RefRW<PathfindingRequester>, RefRO<PathFollower>>())
                {
                    if (!follower.ValueRO.DestinationReached)
                    {
                        requester.ValueRW.RequestGiven = false; // 重新排队
                    }
                }
            }
        }
    }
}
