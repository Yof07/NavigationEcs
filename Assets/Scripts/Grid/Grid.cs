using Unity.Entities;

namespace Navigation
{
    /// <summary>
    /// 不同岛屿（Island）间的寻路策略枚举。
    /// 岛屿是指由不可通行节点隔开的相互独立的可通行区域
    /// </summary>
    public enum DifferentIslandsPolicy
    {
        /// <summary>起点与终点不在同一岛屿时，不进行寻路计算（直接放弃）</summary>
        DontPath,

        /// <summary>
        /// 起点与终点不在同一岛屿时，寻路到起点所在岛屿上距目标最近的节点。
        /// 警告：在代理数量多、网格密集时性能消耗较大，不推荐使用
        /// </summary>
        GoToClosest
    }

    /// <summary>
    /// 寻路网格单例组件，描述整个导航网格的全局参数。
    /// 整个场景只有一个 Grid Entity 拥有此组件
    /// </summary>
    public struct Grid : IComponentData
    {
        /// <summary>网格总宽度（世界单位）</summary>
        public float Width;

        /// <summary>网格总高度（世界单位）</summary>
        public float Height;

        /// <summary>X 方向节点数量 = Width / NodeSize（四舍五入）</summary>
        public int NodeXCount;

        /// <summary>Y 方向节点数量 = Height / NodeSize（四舍五入）</summary>
        public int NodeYCount;

        /// <summary>
        /// 不可通行层的 LayerMask 位掩码（int 类型）。
        /// 此层上的碰撞体会将对应节点标记为 IsAccessible = false
        /// </summary>
        public int InaccessibleLMIndex;

        /// <summary>
        /// 惩罚层的 LayerMask 位掩码（int 类型）。
        /// 此层上的碰撞体会给节点增加 Penalty，使寻路算法尽量绕开该区域
        /// </summary>
        public int PenaltyLMIndex;

        /// <summary>每个节点的边长（世界单位），决定网格精度</summary>
        public float NodeSize;

        /// <summary>
        /// 当起点和终点分属不同岛屿时的处理策略。
        /// 详见 DifferentIslandsPolicy 枚举
        /// </summary>
        public DifferentIslandsPolicy DifferentIslandsPolicy;

        /// <summary>
        /// 目标节点不可通行时，BFS 搜索最近可通行节点的范围上限（节点数）。
        /// 推荐值：10-20，随网格密度增大而适当增大
        /// </summary>
        public int TargetInaccessibleSearchTolerance;

        /// <summary>网格刷新后是否让所有代理重新规划路径</summary>
        public bool RecalcPathsAfterGridRefresh;

        /// <summary>
        /// 网格刷新触发标志。添加或移动障碍物/惩罚区域后，
        /// 将此值设为 true 以重新扫描网格节点可通行性
        /// </summary>
        public bool RefreshGrid;

        /// <summary>
        /// 岛屿刷新触发标志。网格刷新后由系统自动置为 true，
        /// 触发 CheckNodeIslandSystem 重新计算连通区域
        /// </summary>
        public bool RefreshIslands;

        /// <summary>
        /// 网格数据版本号：每次障碍/惩罚扫描完成（含首帧那次）由 CheckObstaclesSystem 自增。
        /// 订阅方各自记住"我处理到哪一版"，从而幂等地感知"网格变了"，不需要清位。
        /// 运行期字段，不烘焙、不序列化。
        /// </summary>
        public int Version;
    }

    /// <summary>
    /// 寻路性能参数组件，与 Grid 同挂载在 Grid Entity 上
    /// </summary>
    public struct PathfindingParameters : IComponentData
    {
        /// <summary>
        /// 每帧最多执行的寻路操作数量。
        /// 限制此值可以防止单帧内大量寻路请求导致卡顿（时间分片）
        /// </summary>
        public int MaxPathfindingOpsPerFrame;
    }
}
