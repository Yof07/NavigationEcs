using Unity.Entities;

namespace Navigation
{
    /// <summary>
    /// RVO（互惠速度障碍）避障组件。
    /// 挂载此组件后，代理会参与多代理碰撞避免计算（ORCA 算法）。
    /// 不挂载则代理不参与避障，也不被其他代理避让
    /// </summary>
    public struct AvoidanceAgent : IComponentData
    {
        /// <summary>
        /// 时间窗口（秒）：预测未来多长时间内的碰撞。
        /// 值越大，代理越早开始规避；推荐值为 2
        /// </summary>
        public float TimeHorizon;
    }

    /// <summary>
    /// 邻居代理缓冲组件，存储 RVO 避障计算时需要考虑的邻居代理列表。
    /// 由 AgentMovementSystem 填充（KNN 或均匀网格），无需手动设置。
    /// InternalBufferCapacity(0) 表示数据存储在堆内存中（不限制大小）
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct NeighbourBuffer : IBufferElementData
    {
        /// <summary>邻居代理的 Entity 引用</summary>
        public Entity Agent;
    }
}
