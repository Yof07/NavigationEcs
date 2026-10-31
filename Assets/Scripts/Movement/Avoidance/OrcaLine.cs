using Unity.Mathematics;

namespace Navigation
{
    /// <summary>
    /// ORCA（最优互惠碰撞避免）算法中的速度约束半平面
    /// 每个邻居代理会产生一条ORCA线，代表当前代理的速度必须位于该线的某一侧（半平面内）
    /// 用 点+方向 的参数化直线形式表示，避障算法通过线性规划在所有约束下找最优速度
    /// </summary>
    public struct OrcaLine
    {
        /// <summary>
        /// 直线的方向向量（2D，XZ平面）
        /// 代理的最终速度必须位于该方向向量左侧（逆时针半平面），
        /// 即满足：cross(Direction, velocity - Point) >= 0
        /// </summary>
        public float2 Direction;

        /// <summary>
        /// 直线上的一个参考点（2D，XZ平面）
        /// 与Direction共同定义ORCA约束半平面
        /// </summary>
        public float2 Point;
    }
}
