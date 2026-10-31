using Unity.Entities;

namespace Navigation
{
    /// <summary>
    /// 邻居查询方式，knn或 均匀网格
    /// </summary>
    public enum NeighbourQueryMethod
    {
        KdTree = 0,
        UniformGrid = 1
    }
    
    /// <summary>
    /// 邻居查询的共享配置（KNN与均匀网格都会读）
    /// 由GridAuthoring烘焙到Grid Entity上，运行期只读；
    /// 它描述的是“怎么找邻居”，与导航图（Grid）的寻路数据无关
    /// </summary>
    public struct NeighbourQuerySettings : IComponentData
    {
        /// <summary>
        /// 邻居查询的实现方式
        /// 枚举0必须是KdTree：老场景没有这个组件/字段时，
        /// 默认行为与改造前保持一致
        /// </summary>
        public NeighbourQueryMethod Method;

        /// <summary>
        /// 每个代理最多保留的邻居数量，**不含自身**
        /// - KdTree分支：查询长度 = MaxNeighbours + 1（KNN结果含自身，写回时排除）；
        /// - UniformGrid分支：上限 = MaxNeighbours（桶查询显式跳过自身，不占名额）
        /// 推荐值4-10，值越大避障越精确但开销越高
        /// </summary>
        public int MaxNeighbours;
    }

    /// <summary>
    /// 均匀网格查询配置，只在NeighbourQuerySettings.Method == UniformGrid时读取
    /// 只管“桶怎么铺、找多远”，与导航网格的NodeSize解耦：
    /// 路径精度变化不应该顺带改变邻居查询的密度
    /// </summary>
    public struct UniformGridQuerySettings : IComponentData
    {
        /// <summary>
        /// cell的世界边长（米），必须大于0
        /// 它只影响候选数量和查询性能，不影响结果正确性；
        /// “找多远”由QueryRadius单独决定
        /// </summary>
        public float CellSize;

        /// <summary>
        /// 查询半径（米），必须大于0：只把距离 ≤ 该值的代理算作邻居
        /// 需覆盖「自身半径 + 场景最大代理半径」，而不是「自己半径的倍数」
        ///
        /// 本Demo所有代理半径一致，此时等价于「2 × 代理半径 + 余量」，默认值即够用；
        /// 因此这里刻意不做max_r统计、也不做分层网格
        /// 若将来引入半径差异明显的单位，本参数会退化成全局最坏值：调大则所有小代理一起
        /// 白扫（扫描格数按R² 增长），调小则大单位的邻居被漏掉（表现为周期性穿模、互弹）
        /// 届时二选一：逐代理rings（保留点登记）或按尺寸分层网格
        ///
        /// 与CellSize独立：半径决定筛多远，CellSize决定扫几格
        /// </summary>
        public float QueryRadius;
    }
}
