using Unity.Entities;
using Unity.Mathematics;

namespace Navigation
{
    /// <summary>
    /// 代理（Agent）核心组件，存储寻路代理的基本属性
    /// 挂载此组件的 Entity 会被寻路系统识别为可移动的角色
    /// </summary>
    public struct Agent : IComponentData
    {
        /// <summary>移动速度（单位/秒）</summary>
        public float MaxSpeed;

        /// <summary>
        /// 代理的物理半径，用于：
        /// 1. 判断是否到达路径点
        /// 2. RVO 避障碰撞半径计算
        /// </summary>
        public float Radius;

        /// <summary>代理的高度，用于确定 Y 轴悬浮高度（Position.y = Height / 2）</summary>
        public float Height;

        /// <summary>转向速度（度/秒），控制旋转朝向目标的平滑程度</summary>
        public float TurnSpeed;

        /// <summary>
        /// 路径点切换距离阈值：代理距离当前路径点小于此值时，切换到下一个点。
        /// 推荐设置为 Grid.NodeSize，设为负值则自动使用 Grid.NodeSize
        /// </summary>
        public float NextTargetTouchDistance;

        /// <summary>
        /// 是否在到达目的地后忽略避障。
        /// 设为 true 时，其他代理不会因本代理停止而绕开它（减少运算开销）
        /// </summary>
        public bool IgnoreAvoidanceWhenDestReached;

        /// <summary>停止距离：代理距最终目的地小于此值时，视为到达目标</summary>
        public float StoppingDistance;

        /// <summary>当前速度向量（由系统每帧更新，记录实际移动方向和大小）</summary>
        public float3 CurrentVelocity;
    }

    /// <summary>
    /// 路径跟随组件，记录代理沿路径移动的状态。
    /// 即在代理沿着已获取路径移动时用来记录当时状态的组件
    /// 与 PathBuffer 配合使用，追踪当前目标路径点索引
    /// </summary>
    public struct PathFollower : IComponentData
    {
        /// <summary>是否有可用路径（路径规划成功后由系统置为 true）</summary>
        public bool PathAvailable;

        /// <summary>
        /// 当前追踪的路径点索引（在 PathBuffer 中的位置）
        /// TargetIndex 映射一个 PathBuffer ，即当前路径点(每个 PathBuffer 存储一个路径点)
        /// </summary>
        public int TargetIndex;

        /// <summary>是否已到达最终目的地（移动状态机终态；由 AgentMovementSystem 读写）</summary>
        public bool DestinationReached;
    }

    /// <summary>
    /// 持续跟随目标组件。挂载此组件后，代理会持续追踪目标 Entity 的位置并重新寻路。
    /// 目标 Entity 必须拥有 LocalTransform 组件
    /// </summary>
    public struct ConstantTargetFollower : IComponentData
    {
        /// <summary>追踪的目标 Entity（必须有 LocalTransform 组件）</summary>
        public Entity Target;

        /// <summary>
        /// 是否在目标静止时也重新寻路。
        /// 不推荐设为 true，除非场景地形频繁变化（性能开销大）
        /// </summary>
        public bool RecalculateOnStationary;

        /// <summary>目标上一帧的位置，用于判断目标是否移动（避免不必要的重新寻路）</summary>
        public float3 TargetLastPosition;
    }
}
