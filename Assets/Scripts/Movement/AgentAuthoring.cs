using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace Navigation
{
    /// <summary>
    /// 寻路代理的 Authoring 组件（挂载在 Unity GameObject 上）。
    /// 在 SubScene 烘焙（Bake）阶段，由 AgentBaker 将 Inspector 参数转换为 ECS 组件。
    /// 支持可选的初始目的地和 RVO 避障功能
    /// </summary>
    public class AgentAuthoring : MonoBehaviour
    {
        /// <summary>代理移动速度（世界单位/秒）</summary>
        public float MaxSpeed;

        /// <summary>代理物理半径（用于路径点判断和 RVO 碰撞）</summary>
        public float Radius;

        /// <summary>代理高度（代理 Y 轴中心 = Height/2）</summary>
        public float Height;

        /// <summary>
        /// 转向速度（度/秒）
        /// </summary>
        [Tooltip("Rotation speed of the agent")]
        public float TurnSpeed;

        /// <summary>停止距离：与最终目的地的距离小于此值时视为到达</summary>
        [Tooltip("Suffecient distance from target to stop")]
        public float StoppingDistance;

        /// <summary>
        /// 路径中间点切换距离阈值。
        /// 负值 = 自动使用 Grid.NodeSize
        /// </summary>
        [Tooltip("Min distance for next target point to count as reached. Setting this to a negative value will keep it at Grid.NodeSize")]
        public float NextTargetTouchDistance;

        /// <summary>
        /// 是否启用 RVO 避障。
        /// true = 添加 AvoidanceAgent 和 NeighbourBuffer 组件
        /// </summary>
        [Tooltip("Activate avoidance for this agent. If deactivated, will not avoid and will not be avoided")]
        public bool ActivateAvoidance;

        /// <summary>到达目的地后其他代理不再绕避本代理（优化停止状态的性能）</summary>
        [Tooltip("Other agents will not avoid this when it's desination has been reached")]
        public bool IgnoreAvoidanceWhenDestReached;

        /// <summary>是否有初始目的地（在烘焙时自动提交第一次寻路请求）</summary>
        public bool HasIntitalDestination;

        /// <summary>初始目的地世界坐标（仅当 HasIntitalDestination = true 时有效）</summary>
        public Vector3 InitialDestination;

        /// <summary>
        /// Baker 内部类：将 AgentAuthoring 的参数烘焙为 ECS 组件。
        /// 触发条件：AgentAuthoring 挂载在 Agent.prefab ，是场景外的资产，不会被 SubScene 自动枚举；
        ///   也就是说不会被自动烘焙，当 GTSpawnerBaker 通过 GetEntity(mAgentPrefab) 引用它时，
        ///   烘焙系统在本次 SubScene 烘焙流程内对该 prefab 做惰性烘焙，此时本 Baker 才执行。
        /// </summary>
        public class AgentBaker : Baker<AgentAuthoring>
        {
            public override void Bake(AgentAuthoring authoring)
            {
                // 获取或创建对应的 Entity（Dynamic = 运行时可移动）
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                // 核心代理参数
                Agent agent = new Agent()
                {
                    MaxSpeed = authoring.MaxSpeed,
                    Radius = authoring.Radius,
                    Height = authoring.Height,
                    TurnSpeed = authoring.TurnSpeed,
                    NextTargetTouchDistance = authoring.NextTargetTouchDistance,
                    IgnoreAvoidanceWhenDestReached = authoring.IgnoreAvoidanceWhenDestReached,
                    StoppingDistance = authoring.StoppingDistance
                };

                // 路径跟随状态初始化（尚无路径）
                PathFollower follower = new PathFollower()
                {
                    PathAvailable = false,
                    TargetIndex = 0
                };

                // 寻路请求初始化
                // RequestGiven = !HasInitialDestination 意思：
                //   有初始目的地 → RequestGiven=false → 立即发起请求
                //   无初始目的地 → RequestGiven=true → 不自动请求
                PathfindingRequester requester = new PathfindingRequester()
                {
                    Destination = new float3(authoring.InitialDestination.x, authoring.InitialDestination.y, authoring.InitialDestination.z),
                    RequestEntity = Entity.Null,
                    RequestGiven = !authoring.HasIntitalDestination,
                };

                // RVO 避障参数（TimeHorizon=2 秒为推荐默认值）
                AvoidanceAgent avoidance = new AvoidanceAgent()
                {
                    TimeHorizon = 2
                };

                // 添加必要的核心组件
                AddComponent(entity, agent);
                AddComponent(entity, follower);
                AddComponent(entity, requester);
                AddBuffer<PathBuffer>(entity); // 一个 PathBuffer 代表一个路径点，一个数组则表示一条路径

                // 如果启用避障，额外添加避障相关组件
                if (authoring.ActivateAvoidance)
                {
                    AddComponent(entity, avoidance);
                    AddBuffer<NeighbourBuffer>(entity); // 邻居缓冲区（由系统自动填充）
                }
            }
        }
    }
}
