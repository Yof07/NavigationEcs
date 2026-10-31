using Unity.Entities;

namespace Navigation
{
    /// <summary>
    /// 地形惩罚组件，需挂载在已设置正确LayerMask的碰撞体Entity上
    /// 与Obstacle的区别：Obstacle使节点完全不可通行，
    /// 而Penalty仅增加节点代价，寻路算法会尽量绕开高惩罚区域但仍可通行
    /// 举例：道路（低惩罚）vs草地（高惩罚），引导代理优先走道路
    /// </summary>
    public struct Penalty : IComponentData
    {
        /// <summary>
        /// 惩罚值，叠加到被此碰撞体覆盖的网格节点上
        /// 经过CheckObstaclesSystem的高斯模糊后，惩罚会平滑扩散到周边节点
        /// 值越大，A* 算法越倾向于选择其他路径
        /// </summary>
        public int PenaltyValue;
    }
}
