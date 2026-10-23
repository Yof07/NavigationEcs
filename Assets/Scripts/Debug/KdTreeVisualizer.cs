using System.Collections.Generic;
using KNN;
using KNN.Internal;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Navigation
{
    /// <summary>
    /// KD 树实时可视化组件（Scene 视图 Gizmos）。
    ///
    /// 使用方式：
    /// 1. 挂载此脚本到场景中任意 GameObject
    /// 2. 运行游戏，在 Scene 视图中即可看到 KD 树结构
    /// 3. 通过 Inspector 开关各层可视化选项
    ///
    /// 显示内容：
    /// - KD 树各层包围盒（颜色渐变：根=红，叶=蓝）
    /// - 分割平面线框（X 轴=橙，Y 轴=绿，Z 轴=紫）
    /// - 代理间的 K 近邻连线（青色）
    /// - 代理位置点（黄色球）
    ///
    /// 注意：每帧会重新构建一个临时 KD 树用于可视化，仅供 Scene 调试使用，
    /// 不影响 AgentMovementSystem 中的正式 KD 树计算。
    /// </summary>
    [AddComponentMenu("Navigation/KD Tree Visualizer")]
    public class KdTreeVisualizer : MonoBehaviour
    {
        [Header("KD 树包围盒")]
        [Tooltip("是否显示 KD 树各层节点的包围盒")]
        [SerializeField]
        bool showTree = true;

        [Tooltip("最大显示深度（建议 4~6，过大会有大量小包围盒）")]
        [SerializeField, Range(1, 12)]
        int maxDepth = 5;

        [Tooltip("各深度的颜色渐变（左=根节点，右=叶节点）")]
        [SerializeField]
        Gradient depthGradient;

        [Header("分割平面")]
        [Tooltip("是否显示每个内部节点的分割平面线框")]
        [SerializeField]
        bool showSplitPlanes = true;

        [Header("邻居连线")]
        [Tooltip("是否显示代理间的 KNN 邻居连线（复用 NeighbourBuffer 的已计算结果）")]
        [SerializeField]
        bool showNeighbours = true;

        [Tooltip("邻居连线颜色")]
        [SerializeField]
        Color neighbourColor = new Color(0f, 1f, 1f, 0.7f);

        [Header("代理位置")]
        [Tooltip("是否显示每个避障代理的位置标记球")]
        [SerializeField]
        bool showAgentDots = true;

        [Tooltip("标记球半径")]
        [SerializeField, Range(0.02f, 1f)]
        float dotRadius = 0.15f;

        // ── 缓存数据（Update 写入，OnDrawGizmos 读取）────────────────────────────

        /// <summary>
        /// 从 KdNode 拷贝出来的、可直接用于 Gizmos 绘制的一层缓存。
        /// Gizmos 不能在 NativeContainer 上直接绘制，且临时 KD 树在 Update 末尾就释放了，
        /// 所以必须先把需要的信息搬到托管结构里。
        /// </summary>
        struct CachedNode
        {
            public Vector3 center;
            public Vector3 size;
            public int depth;
            public int partitionAxis; // -1=叶节点 0=X 1=Y 2=Z
            public float partitionCoord;
            public Vector3 boundsMin;
            public Vector3 boundsMax;
        }

        readonly List<CachedNode> m_cachedNodes = new List<CachedNode>();
        readonly List<(Vector3 from, Vector3 to)> m_neighbourLines = new List<(Vector3 from, Vector3 to)>();
        readonly List<Vector3> m_agentPositions = new List<Vector3>();

        /// <summary>本次采样中出现的最大节点深度，用于把 depth 归一化到渐变 [0,1]</summary>
        int m_maxDepthFound = 1;

        EntityManager m_em;

        /// <summary>分割平面颜色：X=橙 Y=绿 Z=紫（alpha 仅对 Gizmos 填充生效）</summary>
        static readonly Color[] k_splitColors =
        {
            new Color(1.0f, 0.55f, 0.0f, 0.5f),
            new Color(0.2f, 1.0f, 0.3f, 0.5f),
            new Color(0.7f, 0.2f, 1.0f, 0.5f),
        };

        // ── Unity 回调 ──────────────────────────────────────────────────────────

        /// <summary>
        /// 组件首次添加时设置默认渐变（红→黄→绿→蓝），省得每次手动配色。
        /// </summary>
        void Reset()
        {
            depthGradient = new Gradient();
            depthGradient.SetKeys(
                new[]
                {
                    new GradientColorKey(new Color(1.0f, 0.2f, 0.2f), 0.00f),
                    new GradientColorKey(new Color(1.0f, 0.85f, 0.1f), 0.33f),
                    new GradientColorKey(new Color(0.1f, 0.9f, 0.4f), 0.66f),
                    new GradientColorKey(new Color(0.2f, 0.4f, 1.0f), 1.00f),
                },
                new[]
                {
                    new GradientAlphaKey(0.85f, 0.0f),
                    new GradientAlphaKey(0.25f, 1.0f),
                });
        }

        /// <summary>
        /// 每帧重新采集一次数据：代理位置 → 临时 KD 树 → 拷出节点包围盒 / 分割信息 / 邻居连线。
        /// 之所以放在 Update 而不是 OnDrawGizmos：EntityQuery、Native 容器分配都不该在绘制回调里做。
        /// </summary>
        void Update()
        {
            if (!Application.isPlaying)
                return;

            m_cachedNodes.Clear();
            m_neighbourLines.Clear();
            m_agentPositions.Clear();
            m_maxDepthFound = 1;

            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null)
                return;

            m_em = world.EntityManager;

            // 查询所有避障代理（与 AgentMovementSystem 阶段 3 建树用的是同一批点）
            var query = m_em.CreateEntityQuery(
                ComponentType.ReadOnly<LocalTransform>(),
                ComponentType.ReadOnly<AvoidanceAgent>());     // 这里应该是 AvoidanceAgent，但是我要在没有避障的时候也显示，所以暂时写成ASAgent

            if (query.IsEmpty)
            {
                query.Dispose();
                return;
            }

            var entities = query.ToEntityArray(Allocator.Temp);
            var positions = new NativeArray<float3>(entities.Length, Allocator.Temp);

            for (int i = 0; i < entities.Length; i++)
            {
                positions[i] = m_em.GetComponentData<LocalTransform>(entities[i]).Position;
                m_agentPositions.Add((Vector3)positions[i]);
            }

            // 构建临时 KD 树：Allocator.Temp 不可传给 Job，直接调用 Rebuild() 在主线程同步完成
            var container = new KnnContainer(positions, buildNow: false, Allocator.Temp);
            container.Rebuild();

            if (showTree || showSplitPlanes)
                ExtractTreeNodes(container);

            if (showNeighbours)
                ExtractNeighbourLines(entities, positions);

            container.Dispose();
            positions.Dispose();
            entities.Dispose();
            query.Dispose();
        }

        /// <summary>
        /// 只读缓存，绘制全部来自 m_cachedNodes / m_neighbourLines / m_agentPositions。
        /// </summary>
        void OnDrawGizmos()
        {
            if (!Application.isPlaying)
                return;

            // 1. KD 包围盒（按深度着色）
            if (showTree)
            {
                foreach (var node in m_cachedNodes)
                {
                    float t = m_maxDepthFound > 0 ? (float)node.depth / m_maxDepthFound : 0f;
                    Gizmos.color = depthGradient != null
                        ? depthGradient.Evaluate(t)
                        : Color.white;
                    Gizmos.DrawWireCube(node.center, node.size);
                }
            }

            // 2. 分割平面线框（X=橙 Y=绿 Z=紫）
            if (showSplitPlanes)
            {
                foreach (var node in m_cachedNodes)
                {
                    int axis = node.partitionAxis;
                    if (axis < 0 || axis > 2)
                        continue;

                    Gizmos.color = k_splitColors[axis];
                    DrawSplitPlane(node.boundsMin, node.boundsMax, axis, node.partitionCoord);
                }
            }

            // 3. 邻居连线
            if (showNeighbours)
            {
                Gizmos.color = neighbourColor;
                foreach (var (from, to) in m_neighbourLines)
                {
                    Gizmos.DrawLine(from, to);
                }
            }

            // 4. 代理位置球
            if (showAgentDots)
            {
                Gizmos.color = new Color(1f, 0.9f, 0.05f, 0.9f);
                foreach (var pos in m_agentPositions)
                {
                    Gizmos.DrawSphere(pos, dotRadius);
                }
            }
        }

        // ── 私有辅助 ────────────────────────────────────────────────────────────

        /// <summary>
        /// 广度优先遍历临时 KD 树，把每个节点的包围盒与分割信息拷到 m_cachedNodes。
        /// 只走到 maxDepth 层；节点本身仍会记录，只是不再继续向下展开。
        /// </summary>
        void ExtractTreeNodes(KnnContainer container)
        {
            int root = container.RootIndex;
            if (root < 0)
                return;

            var queue = new Queue<(int idx, int depth)>();
            queue.Enqueue((idx: root, depth: 0));

            while (queue.Count > 0)
            {
                var (idx, depth) = queue.Dequeue();
                if (depth > maxDepth)
                    continue;

                KdNode node = container.InternalNodes[idx];
                float3 bMin = node.Bounds.Min;
                float3 bMax = node.Bounds.Max;

                m_cachedNodes.Add(new CachedNode
                {
                    center = (Vector3)((bMin + bMax) * 0.5f),
                    size = (Vector3)(bMax - bMin),
                    depth = depth,
                    partitionAxis = node.PartitionAxis,
                    partitionCoord = node.PartitionCoordinate,
                    boundsMin = bMin,
                    boundsMax = bMax,
                });

                if (depth > m_maxDepthFound)
                    m_maxDepthFound = depth;

                if (!node.Leaf)
                {
                    if (node.NegativeChildIndex >= 0)
                        queue.Enqueue((idx: node.NegativeChildIndex, depth: depth + 1));
                    if (node.PositiveChildIndex >= 0)
                        queue.Enqueue((idx: node.PositiveChildIndex, depth: depth + 1));
                }
            }
        }

        /// <summary>
        /// 读取 NeighbourBuffer 里已经算好的邻居列表，转成"代理 → 邻居"的线段。
        /// 邻居缓冲区由 AgentMovementSystem 填充，本组件只负责画，不重新查询。
        /// </summary>
        void ExtractNeighbourLines(NativeArray<Entity> entities, NativeArray<float3> positions)
        {
            for (int i = 0; i < entities.Length; i++)
            {
                Entity e = entities[i];
                if (!m_em.HasBuffer<NeighbourBuffer>(e))
                    continue;

                var buf = m_em.GetBuffer<NeighbourBuffer>(e);
                var from = (Vector3)positions[i];

                for (int j = 0; j < buf.Length; j++)
                {
                    Entity nb = buf[j].Agent;
                    if (!m_em.Exists(nb) || !m_em.HasComponent<LocalTransform>(nb))
                        continue;

                    m_neighbourLines.Add((from, (Vector3)m_em.GetComponentData<LocalTransform>(nb).Position));
                }
            }
        }

        /// <summary>
        /// 在节点包围盒内绘制分割平面（轴对齐矩形，用 4 条边画）。
        ///
        /// 平面的法线是分割轴，因此另外两个轴取包围盒的 Min/Max 即可得到矩形四角；
        /// 分割轴上的坐标用 PartitionCoordinate。滑动中点算法在点集偏斜时会把分割坐标
        /// 贴到包围盒边界上，这里 clamp 一下，避免画出退化成一条线的矩形。
        /// </summary>
        /// <param name="boundsMin">节点包围盒最小角点</param>
        /// <param name="boundsMax">节点包围盒最大角点</param>
        /// <param name="axis">分割轴（0=X, 1=Y, 2=Z）</param>
        /// <param name="coord">分割平面在分割轴上的坐标</param>
        void DrawSplitPlane(Vector3 boundsMin, Vector3 boundsMax, int axis, float coord)
        {
            float c = Mathf.Clamp(coord, boundsMin[axis], boundsMax[axis]);

            Vector3 c0, c1, c2, c3;
            if (axis == 0)
            {
                // 法线为 X：矩形落在 YZ 平面上
                c0 = new Vector3(c, boundsMin.y, boundsMin.z);
                c1 = new Vector3(c, boundsMin.y, boundsMax.z);
                c2 = new Vector3(c, boundsMax.y, boundsMax.z);
                c3 = new Vector3(c, boundsMax.y, boundsMin.z);
            }
            else if (axis == 1)
            {
                // 法线为 Y：矩形落在 XZ 平面上
                c0 = new Vector3(boundsMin.x, c, boundsMin.z);
                c1 = new Vector3(boundsMax.x, c, boundsMin.z);
                c2 = new Vector3(boundsMax.x, c, boundsMax.z);
                c3 = new Vector3(boundsMin.x, c, boundsMax.z);
            }
            else
            {
                // 法线为 Z：矩形落在 XY 平面上
                c0 = new Vector3(boundsMin.x, boundsMin.y, c);
                c1 = new Vector3(boundsMax.x, boundsMin.y, c);
                c2 = new Vector3(boundsMax.x, boundsMax.y, c);
                c3 = new Vector3(boundsMin.x, boundsMax.y, c);
            }

            Gizmos.DrawLine(c0, c1);
            Gizmos.DrawLine(c1, c2);
            Gizmos.DrawLine(c2, c3);
            Gizmos.DrawLine(c3, c0);
        }
    }
}
