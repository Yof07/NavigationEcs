using System;

namespace Navigation
{
    /// <summary>
    /// 单个网格节点的数据结构，实现了IHeapItem接口以支持最小堆排序（A* open set）
    /// </summary>
    public struct GridNode : IEquatable<GridNode>, IHeapItem<GridNode>
    {
        /// <summary>节点是否可通行（false表示被障碍物占据）</summary>
        public bool IsAccessible;

        /// <summary>节点在网格中的X坐标（列）</summary>
        public int X;

        /// <summary>节点在网格中的Y坐标（行）</summary>
        public int Y;

        /// <summary>
        /// G代价：从起点到此节点的实际移动代价（含地形惩罚）
        /// 直线移动代价=10，斜线移动代价=14
        /// </summary>
        public int GCost;

        /// <summary>
        /// H代价：此节点到目标节点的启发式估计代价（曼哈顿/切比雪夫混合启发）
        /// H = 14 * min(dx,dy) + 10 * abs(dx-dy)
        /// </summary>
        public int HCost;

        /// <summary>
        /// 地形惩罚值，由CheckObstaclesSystem高斯模糊后写入
        /// 值越大，A* 算法越倾向于绕开该节点（但不是不可通行）
        /// </summary>
        public int Penalty;

        /// <summary>此节点在最小堆数组中的索引（由堆自动维护）</summary>
        public int HeapIndex;

        /// <summary>
        /// A* 回溯路径时使用：记录父节点在Nodes数组中的一维索引，
        /// 寻路完成后通过此链依次回溯直到起点
        /// </summary>
        public int PreviousNodeIndex;

        // 实现IHeapItem接口，使NativeMinHeap可以读写此节点的堆索引
        int IHeapItem<GridNode>.HeapIndex { get => HeapIndex; set => HeapIndex = value; }

        /// <summary>
        /// 堆排序比较函数：按F代价（G+H）升序排列，F相同时按H升序
        /// 返回负值表示this应排在_other前面（最小堆语义下优先级更高）
        /// </summary>
        public int CompareTo(GridNode _other)
        {
            int compare = GridUtility.NodeFCost(this).CompareTo(GridUtility.NodeFCost(_other));
            if (compare == 0)
            {
                compare = HCost.CompareTo(_other.HCost);
            }

            return -compare; // 取反使F最小的节点具有最高优先级
        }

        /// <summary>节点相等判断：坐标相同即视为同一节点</summary>
        public bool Equals(GridNode other)
        {
            return X == other.X &&
                   Y == other.Y;
        }

        /// <summary>调试输出：显示该节点的F代价（G+H）</summary>
        public override string ToString()
        {
            return (GCost + HCost).ToString();
        }

        /// <summary>
        /// 哈希值：将X、Y坐标编码为单个整数，用于NativeHashSet的快速查找
        /// 通过位运算将Y的高低16位分别异或到不同位置，降低碰撞概率
        /// </summary>
        public override int GetHashCode()
        {
            return X ^ (Y << 16) ^ (Y >> 16);
        }
    }
}
