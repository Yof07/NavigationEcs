using Unity.Collections;
using Unity.Mathematics;

namespace Navigation
{
    /// <summary>
    /// 均匀网格的空间布局：世界 XZ 左下角原点、cell 边长、两个方向上的 cell 数量。
    /// 由调用方根据导航网格的世界包围盒 + UniformGridQuerySettings.CellSize 构造一次，
    /// Build / Query 共用，避免重复计算。
    /// 与导航网格的 NodeSize 无关：cell 尺寸是邻居查询自己的参数。
    /// </summary>
    public struct NeighbourGridLayout
    {
        /// <summary>世界 XZ 平面上的左下角（最小 x / 最小 z）。</summary>
        public float3 Origin;

        /// <summary>cell 的世界边长（米），必须大于 0。</summary>
        public float CellSize;

        /// <summary>X 方向（世界 X）的 cell 数量，与导航网格的 NodeXCount 对应。</summary>
        public int CellCountX;

        /// <summary>Y 方向（对应世界 Z）的 cell 数量，与导航网格的 NodeYCount 对应。</summary>
        public int CellCountY;

        /// <summary>cell 总数。</summary>
        public int TotalCellCount => CellCountX * CellCountY;
    }

    /// <summary>
    /// 均匀网格邻居查询：每帧用两遍计数排序把代理索引按 cell 铺平，
    /// 查询时只扫目标代理周围的若干格，而不是和全场代理比距离。
    /// 纯数据操作，不依赖 SystemState / lookup，可在主线程或 Job 里调用。
    /// </summary>
    public static class AgentNeighbourBuckets
    {
        /// <summary>
        /// 构造布局。worldWidth / worldHeight 是导航网格覆盖的世界范围（X / Z 方向），
        /// cellSize 来自 UniformGridQuerySettings；两者都与 NodeSize 无关。
        /// </summary>
        public static NeighbourGridLayout CreateLayout(float3 _origin, float _worldWidth, float _worldHeight, float _cellSize)
        {
            float cellSize = math.max(_cellSize, 1e-4f);
            int cellCountX = math.max(1, (int)math.ceil(_worldWidth / cellSize));
            int cellCountY = math.max(1, (int)math.ceil(_worldHeight / cellSize));

            return new NeighbourGridLayout
            {
                Origin = _origin,
                CellSize = cellSize,
                CellCountX = cellCountX,
                CellCountY = cellCountY
            };
        }

        /// <summary>
        /// 世界坐标 → 一维 cell 下标；越界坐标 clamp 到边缘 cell。
        /// 只使用 x/z，忽略 y；与导航网格的节点索引无关。
        /// </summary>
        public static int CellIndexOf(float3 _position, in NeighbourGridLayout _layout)
        {
            // 布局非法时直接返回 0，避免下面 clamp 的 min > max
            if (_layout.CellCountX <= 0 || _layout.CellCountY <= 0)
            {
                return 0;
            }

            float invCellSize = 1f / math.max(_layout.CellSize, 1e-4f);

            // 连续世界坐标 → 连续 cell 坐标（区间归属，所以用 floor）。
            // 网格 Y 对应世界 Z，与导航网格的 WorldPosToNode 保持一致。
            float cellX = math.floor((_position.x - _layout.Origin.x) * invCellSize);
            float cellY = math.floor((_position.z - _layout.Origin.z) * invCellSize);

            // 先 clamp 再转 int：避免超大/超小浮点直接转 int 的未定义行为
            int x = (int)math.clamp(cellX, 0f, (float)(_layout.CellCountX - 1));
            int y = (int)math.clamp(cellY, 0f, (float)(_layout.CellCountY - 1));

            return x + y * _layout.CellCountX;
        }
        
        /// <summary>
        /// 建桶：两遍计数排序 + 前缀和。
        /// 前置条件：
        ///   _cellStart.Length == layout.TotalCellCount + 1；
        ///   _cellItems.Length == _positions.Length，_cellItems 存储的是代理的索引。
        /// 构建过程中 _cellStart 的含义会变化：
        ///   计数后 = 每格数量；前缀和后 = 每格结束偏移；散射后 = 每格起始偏移。
        /// 结束后第 b 格的成员是 _cellItems[_cellStart[b] .. _cellStart[b + 1])，
        /// 且格内按代理索引升序；_cellStart[TotalCellCount] 是最后一格的结束哨兵。
        /// </summary>
        public static void Build(NativeArray<float3> _positions, in NeighbourGridLayout _layout, NativeArray<int> _cellStart, NativeArray<int> _cellItems)
        {
            // 1) 清空：cellStart[b] 先暂存第 b 格的数量（索引 TotalCellCount 最后当哨兵）
            for (int i = 0; i <= _layout.TotalCellCount; i++)
                _cellStart[i] = 0;

            // 计数：把每个代理的数量记到它所属格子的槽位
            foreach (var pos in _positions)
            {
                int cell = CellIndexOf(pos, _layout);
                _cellStart[cell]++;
            }
            
            // 计算前缀和：cellStart[b] 变成第 b 格的结束偏移
            for (int i = 1; i < _layout.TotalCellCount; i++)  
                _cellStart[i] += _cellStart[i - 1];
            
            // 4) 倒序散射：每写一个就把结束偏移往前挪一格。
            //    结束后 cellStart[b] 自动变回起始偏移，且格内索引升序。
            for (int i = _positions.Length - 1; i >= 0; i--)
            {
                int cell = CellIndexOf(_positions[i], _layout);
                _cellItems[--_cellStart[cell]] = i;
            }
            
            // 5) 最后一格的结束偏移 = 代理总数（哨兵），即最后一格的终点下标
            // 如 _cellStart[1] 代表第 2 格的起始下标、第 1 格的终点下标，_cellStart[2] 代表第 3 格的起始下标...
            // 那么如果 _cellStart 的长度不 +1 的话，就会导致存储不了最后一个格的终点下标
            _cellStart[_layout.TotalCellCount] = _positions.Length;
        }

        /// <summary>
        /// 查询某个代理半径内的最近邻居，最多写入 _bestIndices.Length 个。
        /// 结果按距离升序；距离相同按代理索引升序。返回实际写入数量。
        ///
        /// 扫描方式：rings = ceil(radius / cellSize)，以自己所在 cell 为中心扫一个
        /// (2 * rings + 1)² 的方阵；每个格子通过 [cellStart[cell], cellStart[cell + 1])
        /// 取候选，再按距离和半径过滤。
        ///
        /// 注意：本函数只排除自身；是否排除「已到达目的地且 IgnoreAvoidanceWhenDestReached」
        /// 的代理由调用方的 IsValidNeighbour 负责，与 KNN 分支保持同一个过滤器。
        /// </summary>
        public static int Query(
            NativeArray<float3> _positions,
            in NeighbourGridLayout _layout,
            NativeArray<int> _cellStart,
            NativeArray<int> _cellItems,
            int _selfIndex,
            float _radius,
            NativeArray<int> _bestIndices,
            NativeArray<float> _bestDistSq)
        {
            int cap = _bestIndices.Length;  // cap 即 maxK，最多保留 cap 个邻居

            if (cap <= 0 ||
                _bestDistSq.Length < cap ||
                _radius <= 0f ||
                _layout.CellCountX <= 0 ||
                _layout.CellCountY <= 0 ||
                _selfIndex < 0 ||
                _selfIndex >= _positions.Length)
            {
                return 0;
            }

            float radiusSq = _radius * _radius;
            float cellSize = math.max(_layout.CellSize, 1e-4f);

            // 最大扫描圈数；先 clamp 浮点再转 int，避免超大半径直接转 int 的未定义行为
            float maxRings = math.max(_layout.CellCountX, _layout.CellCountY);
            int rings = (int)math.clamp(math.ceil(_radius / cellSize), 0f, maxRings);

            float3 selfPos = _positions[_selfIndex];

            int selfCell = CellIndexOf(selfPos, _layout);
            int selfCellX = selfCell % _layout.CellCountX;
            int selfCellY = selfCell / _layout.CellCountX;

            int count = 0;

            for (int dy = -rings; dy <= rings; dy++)
            {
                int cellY = selfCellY + dy;
                if (cellY < 0 || cellY >= _layout.CellCountY)
                {
                    continue;
                }

                for (int dx = -rings; dx <= rings; dx++)
                {
                    int cellX = selfCellX + dx;
                    if (cellX < 0 || cellX >= _layout.CellCountX)
                    {
                        continue;
                    }

                    // 获取要遍历的格子在 _cellItems 中的起始和结束下标，以供后续在能遍历
                    int cell = cellX + cellY * _layout.CellCountX;
                    int start = _cellStart[cell];
                    int end = _cellStart[cell + 1];

                    for (int k = start; k < end; k++)
                    {
                        int other = _cellItems[k];
                        if (other == _selfIndex)
                        {
                            continue;
                        }

                        // 距离过滤
                        float distSq = math.distancesq(selfPos, _positions[other]);
                        if (distSq > radiusSq)
                        {
                            continue;
                        }

                        // 已满且不优于最差者，直接跳过
                        if (count == cap && !IsBetter(distSq, other, _bestDistSq[cap - 1], _bestIndices[cap - 1]))
                        {
                            continue;
                        }

                        // 未满或已满但优于最差者
                        int insert = count < cap ? count : cap - 1;   

                        // 插入排序：保持距离升序，平手按索引升序
                        while (insert > 0 && IsBetter(distSq, other, _bestDistSq[insert - 1], _bestIndices[insert - 1]))
                        {
                            _bestDistSq[insert] = _bestDistSq[insert - 1];
                            _bestIndices[insert] = _bestIndices[insert - 1];
                            insert--;
                        }
                        _bestDistSq[insert] = distSq;
                        _bestIndices[insert] = other;

                        if (count < cap)
                        {
                            count++;
                        }
                    }
                }
            }

            return count;
        }

        /// <summary>候选比较：距离更近者优先；距离相同则索引更小者优先。</summary>
        static bool IsBetter(float _distSq, int _index, float _otherDistSq, int _otherIndex)
        {
            return _distSq < _otherDistSq || (_distSq == _otherDistSq && _index < _otherIndex);
        }

    }
}
