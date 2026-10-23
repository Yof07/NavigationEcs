using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace Navigation
{
    /// <summary>
    /// 导航网格的 Authoring 组件（挂载在 Unity GameObject 上）。
    /// 在 SubScene 烘焙阶段，GridBaker 将 Inspector 参数转换为 ECS 组件，
    /// 并初始化所有网格节点（默认全部可通行）
    /// </summary>
    public class GridAuthoring : MonoBehaviour
    {
        /// <summary>网格总尺寸（X=宽度, Y=深度，单位：世界空间）</summary>
        [Header("Grid")]
        public Vector2 mGridSize;

        /// <summary>每个节点的边长（越小精度越高，但节点数量呈平方增长）</summary>
        public float mNodeSize;

        /// <summary>RVO 避障时每个代理考虑的最大邻居数量（推荐 4-10）</summary>
        [Tooltip("Number of max neighbors to consider when calculating for avoidance")]
        public int AvoidanceNeighbours;

        /// <summary>网格刷新（障碍物变化）后是否强制所有代理重新寻路</summary>
        [Tooltip("Force all agents to recalculate their paths upon grid refresh")]
        public bool RecalcPathsAfterGridRefresh;

        /// <summary>起点与终点位于不同岛屿时的处理策略</summary>
        [Tooltip("What to do when agent and target point are inaccessible to each other")]
        public DifferentIslandsPolicy DifferentIslandsPolicy;

        /// <summary>目标节点不可通行时，BFS 搜索可通行节点的最大范围（推荐 10-20）</summary>
        [Tooltip("if target is an inaccessible node, how far to to look for an accessible node in a radial direction (best between 10-20 increasing with grid density)")]
        public int TargetInaccessibleSearchTolerance;

        /// <summary>每帧最多执行的寻路操作数（防止大量请求同帧导致卡顿）</summary>
        [Tooltip("Max number of pathfinding operations to calculate per frame")]
        public int MaxPathfindingOpsPerFrame;
 
        /// <summary>不可通行节点的物理层（障碍物层），此层碰撞体覆盖的节点会被标记为不可通行</summary>
        public LayerMask InaccessibleNodes;

        /// <summary>惩罚节点的物理层（惩罚区域层），此层碰撞体覆盖的节点会增加寻路代价</summary>
        public LayerMask PenaltyNodes;
        
        // 以下字段名已进 Inspector，Unity 按名字存值，勿改名。

        /// <summary>邻居查询方式；默认 KdTree（0）。</summary>
        [Tooltip("Neighbour query backend. KdTree = exact k-NN; UniformGrid = uniform grid buckets")]
        public NeighbourQueryMethod NeighbourQueryMethod;

        /// <summary>均匀网格的 cell 边长（米）。</summary>
        [Tooltip("Uniform grid cell size in world units (meters), independent of mNodeSize")]
        public float UniformCellSize = 2f;

        /// <summary>均匀网格的查询半径（米）。</summary>
        [Tooltip("Neighbour query radius in world units (meters); should cover 2 x max agent radius + margin")]
        public float UniformQueryRadius = 2f;


        /// <summary>
        /// Baker 内部类：将 GridAuthoring 参数烘焙为 ECS 单例组件，并初始化网格节点数组
        /// </summary>
        public class GridBaker : Baker<GridAuthoring>
        {
            public override void Bake(GridAuthoring authoring)
            {
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                // 保存 Grid Entity 引用供 GridUtility 外部访问
                GridUtility.SetGridEntity(entity);

                // 根据网格尺寸和节点大小计算节点数量
                Grid grid = new Grid
                {
                    NodeXCount = (int)math.round(authoring.mGridSize.x / authoring.mNodeSize),
                    NodeYCount = (int)math.round(authoring.mGridSize.y / authoring.mNodeSize),
                    NodeSize = authoring.mNodeSize,
                    RecalcPathsAfterGridRefresh = authoring.RecalcPathsAfterGridRefresh,
                    InaccessibleLMIndex = authoring.InaccessibleNodes.value,    // LayerMask 转 int
                    PenaltyLMIndex = authoring.PenaltyNodes.value,
                    DifferentIslandsPolicy = authoring.DifferentIslandsPolicy,
                    TargetInaccessibleSearchTolerance = authoring.TargetInaccessibleSearchTolerance,
                };

                PathfindingParameters pathfindingParameters = new PathfindingParameters()
                {
                    MaxPathfindingOpsPerFrame = authoring.MaxPathfindingOpsPerFrame
                };

                NeighbourQuerySettings neighbourQuerySettings = new NeighbourQuerySettings()
                {
                    Method = authoring.NeighbourQueryMethod,
                    MaxNeighbours = authoring.AvoidanceNeighbours,
                };
                
                UniformGridQuerySettings uniformGridQuerySettings = new UniformGridQuerySettings()
                {
                    CellSize = authoring.UniformCellSize,
                    QueryRadius = authoring.UniformQueryRadius,
                };

                // 添加 Grid 核心组件
                AddComponent(entity, grid);
                AddComponent(entity, pathfindingParameters);
                AddComponent(entity, neighbourQuerySettings);
                AddComponent(entity, uniformGridQuerySettings);

                // 添加各种缓冲区（初始为空）
                AddBuffer<PathfindingRequestQueue>(entity);  // 寻路请求队列
                AddBuffer<NodeIslandBuffer>(entity);  // 岛屿 ID 缓冲区

                // 初始化节点数组（NodeXCount × NodeYCount 个节点，全部默认可通行）
                DynamicBuffer<GridNodesBuffer> nodes = AddBuffer<GridNodesBuffer>(entity);
                nodes.Length = grid.NodeXCount * grid.NodeYCount;

                // d = max(NodeXCount, NodeYCount)，用于一维索引计算
                int d = grid.NodeXCount > grid.NodeYCount ? grid.NodeXCount : grid.NodeYCount;

                for (int x = 0; x < grid.NodeXCount; x++)
                {
                    for (int y = 0; y < grid.NodeYCount; y++)
                    {
                        // 创建默认节点（可通行，无惩罚，G/H 代价为0）
                        GridNode newNode = new GridNode()
                        {
                            X = x,
                            Y = y,
                            IsAccessible = true,
                        };

                        // 根据主轴选择索引方式（保证数组紧凑）
                        int index;
                        if (grid.NodeXCount > grid.NodeYCount)
                            index = x + y * d;
                        else index = y + x * d;

                        nodes[index] = new GridNodesBuffer() { Node = newNode };
                    }
                }
                // 注：CheckObstaclesSystem 在运行时会更新节点的 IsAccessible 和 Penalty
            }
        }
    }
}
