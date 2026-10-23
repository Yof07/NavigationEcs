using Unity.Entities;

namespace Navigation
{
    /// <summary>
    /// 网格节点缓冲组件，以一维数组存储二维网格的所有节点。
    /// 索引计算：若 NodeXCount > NodeYCount，则 index = x + y * d；否则 index = y + x * d（d = max(NodeXCount, NodeYCount)）
    /// InternalBufferCapacity(0) 表示数据存储在堆内存中，避免影响 Chunk 布局
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct GridNodesBuffer : IBufferElementData
    {
        /// <summary>该位置的网格节点数据</summary>
        public GridNode Node;
    }

    /// <summary>
    /// 岛屿 ID 缓冲组件，与 GridNodesBuffer 索引对应。
    /// 相同 IslandID 的节点属于同一连通区域（岛屿），
    /// 由 CheckNodeIslandSystem 通过 BFS/DFS 填充
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct NodeIslandBuffer : IBufferElementData
    {
        /// <summary>该节点所属岛屿的编号（从 0 开始递增）</summary>
        public int IslandID;
    }
}
