using Unity.Burst;
using Unity.Entities;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Physics;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;

namespace Navigation
{
    /// <summary>
    /// 障碍物和地形惩罚检测系统。
    ///
    /// 职责：
    /// 1. 遍历所有网格节点，通过物理 OverlapBox 检测该节点区域内的碰撞体
    /// 2. 根据碰撞体上的 Obstacle 组件标记节点为不可通行（IsAccessible = false）
    /// 3. 根据 Obstacle/Penalty 组件叠加节点惩罚值
    /// 4. 对惩罚值进行高斯模糊（箱式滤波近似），使惩罚平滑扩散到周边节点
    /// 5. 触发岛屿重新计算（设置 RefreshIslands = true）
    ///
    /// 触发条件：网格 RefreshGrid 标志为 true 时（初始化或障碍物变化后）
    ///
    /// —— 关键设计说明 ——
    /// · 此系统不用 Job，直接在主线程运行。原因：节点数量可能非常大，
    ///   但此系统只在网格刷新时执行一次，性能可接受；且 OverlapBox 需要访问
    ///   PhysicsWorld，在 Job 内需要额外的安全标记。
    /// · ComponentLookup 在 OnCreate 时创建，每帧 Update 刷新引用，
    ///   避免重复创建带来的堆分配开销。
    /// </summary>
    partial struct CheckObstaclesSystem : ISystem
    {
        // 是否已完成障碍物扫描（避免每帧重复扫描）
        private bool mHasCheckedForObstacles;
        // 是否已完成惩罚扫描（当前版本 CheckForPenalties 已合并入 CheckForObstacles）
        private bool mHasCheckedForPenalties;

        // 组件查找缓存（减少每帧重复创建的开销）
        // 按 Entity 和快速组件类型在 chunk 中的基址来计算偏移量查找具体的 Obstacle / Penalty 组件
        // 比 EntityManager.GetComponentData 更高效，内部使用 TypeIndex 直接寻址
        ComponentLookup<Obstacle> mAllObstacles;
        ComponentLookup<Penalty> mAllPenalties;

        /// <summary>
        /// 系统创建时初始化：建立组件查找缓存（Obstacle / Penalty），
        /// 并把两个扫描标志的初值设为 false。
        ///
        /// 初值 false 是刻意的跨系统握手约定：本系统第一帧会无条件执行一次扫描
        /// （不依赖 RefreshGrid），保证场景加载后障碍物数据立即可用；
        /// 而 CheckNodeIslandSystem 的岛屿标志初值故意相反（true，第一帧先不跑），
        /// 必须等本系统扫描完成、置起 RefreshIslands 后才开始工作。
        /// </summary>
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            mHasCheckedForObstacles = false;
            mHasCheckedForPenalties = false;

            mAllObstacles = state.GetComponentLookup<Obstacle>();
            mAllPenalties = state.GetComponentLookup<Penalty>();
        }

        /// <summary>
        /// 每帧执行：扫描网格上的障碍物与惩罚区域，并在网格刷新后按需通知代理重新寻路。
        ///
        /// 扫描时机：
        /// 1. 首次运行：无条件扫描一次（OnCreate 中标志初值为 false）；
        /// 2. 之后仅当外部把 Grid.RefreshGrid 置为 true 时，重置标志并重新扫描。
        /// 扫描完成后置起 RefreshIslands，触发 CheckNodeIslandSystem 重算岛屿。
        ///
        /// 若 Grid.RecalcPathsAfterGridRefresh 为 true 且本帧刚完成刷新，
        /// 则把所有尚未到达目标的代理的 RequestGiven 置为 false，让它们重新发起寻路请求。
        /// </summary>
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // 每帧刷新 Lookup 缓存，使其指向当前帧的 Chunk 数据
            // （Structural Change 改变实体，比如增加删除之类的会导致 Chunk 重组，不刷新会读到过期指针）
            mAllObstacles.Update(ref state);
            mAllPenalties.Update(ref state);

            // 遍历身上同时挂着 Grid 和 LocalTransform 的实体，也就是那个大网格 Entity，通常只有一个，但支持多个共存
            foreach (var (grid, transform, entity) in SystemAPI.Query<RefRW<Grid>, LocalTransform>().WithEntityAccess())
            {
                bool gridRefreshed = false;

                // 检测到 RefreshGrid 请求时，重置扫描状态
                if (grid.ValueRO.RefreshGrid)
                {
                    grid.ValueRW.RefreshGrid = false;
                    mHasCheckedForObstacles = false;
                    mHasCheckedForPenalties = false;
                    gridRefreshed = true;
                }

                // 执行障碍物和惩罚扫描（仅在需要时执行）
                if (!mHasCheckedForObstacles)
                {
                    DynamicBuffer<GridNodesBuffer> nodes = SystemAPI.GetBuffer<GridNodesBuffer>(entity);
                    CheckForObstacles(grid.ValueRO, nodes, transform, ref state);

                    mHasCheckedForObstacles = true;     
                    grid.ValueRW.RefreshIslands = true; // 扫描完成后通知岛屿系统重新计算
                }

                // 网格被外部要求重扫、且配置了「刷新后重新寻路」时，只置一次通知；
                // 真正改 RequestGiven 的是 AgentMovementSystem（它才是 PathFollower 的拥有者）。
                if (gridRefreshed)
                {
                    grid.ValueRW.Version++; // 发布「网格数据变了」，谁来订阅由订阅方自己决定
                }
            }
        }

        /// <summary>
        /// 遍历每个网格节点，通过 OverlapBox 物理查询检测障碍物和惩罚区域。
        ///
        /// —— OverlapBox 工作原理 ——
        /// Unity.Physics 的 OverlapBox 以节点中心为原点，以 NodeSize/2 为半径，
        /// 投出一个轴对齐盒子，返回所有与该盒子相交的碰撞体列表（DistanceHit）。
        /// CollisionFilter 通过位掩码（LayerMask）过滤只关心的层（障碍物层 | 惩罚层），
        /// 避免匹配到不相关的物体（如地面、代理自身）。
        ///
        /// —— 结果处理逻辑 ——
        /// 对每个命中的碰撞体：
        /// 1. 如果其 Entity 有 Obstacle 组件 → 节点标记为不可通行
        ///    （OverlapBox 的 bool 返回值本身代表"是否有碰撞" = IsAccessible 的反值）
        /// 2. 如果其 Entity 有 Penalty 组件 → 节点惩罚值 += PenaltyValue
        ///    （可以和障碍物惩罚叠加，允许一个区域同时是难走地形 + 障碍物边缘）
        ///
        /// 最后调用 BlurPenalties 对所有惩罚值做平滑扩散
        /// </summary>
        private void CheckForObstacles(Grid _grid, DynamicBuffer<GridNodesBuffer> _nodes, LocalTransform _transform, ref SystemState _state)
        {
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;

            // 处理每个节点
            for (int x = 0; x < _grid.NodeXCount; x++)
            {
                for (int y = 0; y < _grid.NodeYCount; y++)
                {
                    // 计算节点的一维索引
                    int iindex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        iindex = x + y * d;
                    else iindex = y + x * d;

                    // 重置节点状态（每次刷新时清除旧数据）
                    var nn = _nodes[iindex];
                    nn.Node.IsAccessible = true;
                    nn.Node.Penalty = 0;
                    _nodes[iindex] = nn;

                    // 获取当前帧的 Unity Physics World
                    /*
                    Unity Physics World 就是把当前 ECS 世界里所有实体的物理状态（碰撞体、质量、速度等）汇总成的一份数据快照，
                    由 BuildPhysicsWorld 每帧重建、以 PhysicsWorldSingleton 单例暴露，供 OverlapBox/CastRay 等查询和物理步进使用
                    */
                    EntityQueryBuilder builder = new EntityQueryBuilder(Allocator.Temp).WithAll<PhysicsWorldSingleton>();
                    EntityQuery singletonQuery = _state.EntityManager.CreateEntityQuery(builder);
                    var physicsWorld = singletonQuery.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
                    singletonQuery.Dispose();

                    // 计算节点中心的世界坐标和检测盒拓展长度，定义与节点同等大小的检测盒（从节点中心向四周扩展 NodeSize/2）
                    float3 extents = new float3(1, 1, 1) * (_grid.NodeSize / 2);
                    float3 worldCoords = GridUtility.GridCoordsToWorld(x, y, _grid, _transform);

                    NativeList<DistanceHit> results = new NativeList<DistanceHit>(Allocator.Temp);

                    // —— OverlapBox 物理查询 ——
                    // 返回值：true = 有碰撞体与盒子相交（可能是障碍物或惩罚区），false = 节点区域内无相关碰撞体
                    // CollisionFilter 说明：
                    //   BelongsTo = ~0u          → 本次查询不属于任何特定层（全 1 = 与任意层都能交互）
                    //   CollidesWith = 障碍物层 | 惩罚层  → 只检测这两个层上的碰撞体，把符合这两层的碰撞体筛出来
                    //   GroupIndex = 0           → 不使用分组过滤
                    // 注意：OverlapBox 的 bool 返回值只要 CollidesWith 内任意层有碰撞就返回 true，
                    //   即使命中的碰撞体只是惩罚层而非障碍物层，也会返回 true。
                    // 所以不能直接用返回值决定 IsAccessible，需要再遍历 results 区分出障碍物后再写入
                    bool isAccessible = !physicsWorld.OverlapBox(worldCoords, quaternion.identity, extents, ref results, new CollisionFilter()
                    {
                        BelongsTo = ~0u,         // 属于任意层
                        // 位或，即物体(检测盒)压着惩罚区或障碍物都检查
                        CollidesWith = (uint)_grid.InaccessibleLMIndex | (uint)_grid.PenaltyLMIndex,
                        GroupIndex = 0
                    });

                    // 处理查询结果中的每个碰撞体
                    for (int i = 0; i < results.Length; i++)
                    {
                        // 处理障碍物组件：标记节点不可通行并设置边缘惩罚
                        if (mAllObstacles.HasComponent(results[i].Entity))
                        {
                            int index;
                            if (_grid.NodeXCount > _grid.NodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            Obstacle obstacle = mAllObstacles[results[i].Entity];
                            var newNode = _nodes[index];
                            newNode.Node.IsAccessible = isAccessible;           // 由 OverlapBox 的 bool 返回值决定
                            newNode.Node.Penalty = obstacle.PenaltyOnObstacleValue; // 障碍物边缘惩罚
                            _nodes[index] = newNode;
                        }

                        // 处理惩罚区域组件：叠加地形惩罚值（可与障碍物惩罚叠加）
                        if (mAllPenalties.HasComponent(results[i].Entity))
                        {
                            int index;
                            if (_grid.NodeXCount > _grid.NodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            Penalty penalty = mAllPenalties[results[i].Entity];
                            var newNode = _nodes[index];
                            newNode.Node.Penalty = penalty.PenaltyValue + newNode.Node.Penalty; // 叠加惩罚
                            _nodes[index] = newNode;
                        }
                    }

                    results.Dispose();
                }
            }

            // 对所有节点的惩罚值进行高斯模糊（半径 3 = 7x7 核），使边缘惩罚平滑扩散
            BlurPenalties(3, _grid, _nodes);
        }

        /// <summary>
        /// 仅扫描惩罚区域（不检测障碍物），已合并入 CheckForObstacles，保留备用
        /// </summary>
        private void CheckForPenalties(Grid _grid, DynamicBuffer<GridNodesBuffer> _nodes, LocalTransform _transform, ref SystemState _state)
        {
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;

            for (int x = 0; x < _grid.NodeXCount; x++)
            {
                for (int y = 0; y < _grid.NodeYCount; y++)
                {
                    int iindex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        iindex = x + y * d;
                    else iindex = y + x * d;

                    var nn = _nodes[iindex];
                    nn.Node.Penalty = 0;
                    _nodes[iindex] = nn;

                    EntityQueryBuilder builder = new EntityQueryBuilder(Allocator.Temp).WithAll<PhysicsWorldSingleton>();
                    EntityQuery singletonQuery = _state.EntityManager.CreateEntityQuery(builder);
                    var physicsWorld = singletonQuery.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
                    singletonQuery.Dispose();

                    float3 extents = new float3(1, 1, 1) * (_grid.NodeSize / 2);
                    float3 worldCoords = GridUtility.GridCoordsToWorld(x, y, _grid, _transform);

                    NativeList<DistanceHit> results = new NativeList<DistanceHit>(Allocator.Temp);

                    physicsWorld.OverlapBox(worldCoords, quaternion.identity, extents, ref results, new CollisionFilter()
                    {
                        BelongsTo = ~0u,
                        CollidesWith = (uint)_grid.PenaltyLMIndex,
                        GroupIndex = 0
                    });

                    for (int i = 0; i < results.Length; i++)
                    {
                        if (mAllPenalties.HasComponent(results[i].Entity))
                        {
                            int index;
                            if (_grid.NodeXCount > _grid.NodeYCount)
                                index = x + y * d;
                            else index = y + x * d;

                            Penalty penalty = mAllPenalties[results[i].Entity];
                            var newNode = _nodes[index];
                            newNode.Node.Penalty = penalty.PenaltyValue;
                            _nodes[index] = newNode;
                        }
                    }

                    results.Dispose();
                }
            }

            BlurPenalties(3, _grid, _nodes);
        }

        /// <summary>
        /// 对所有节点的惩罚值进行高斯模糊（使用可分离的箱式滤波近似高斯模糊）。
        ///
        /// 原理：二维高斯模糊 = 先水平方向 1D 箱式滤波 + 再垂直方向 1D 箱式滤波
        /// 滑动窗口算法保证每次加减各一个元素，时间复杂度 O(N)（而非朴素的 O(N * kernel²)）
        ///
        /// 效果：原本集中在障碍物节点上的高惩罚值会平滑扩散到周边 blurSize 个节点，
        /// 让 A* 自然规划出远离障碍物边缘的路径
        /// </summary>
        /// <param name="_blurSize">模糊半径（推荐值 3，kernel 大小 = 2*3+1 = 7）</param>
        void BlurPenalties(int _blurSize, Grid _grid, DynamicBuffer<GridNodesBuffer> _nodes)
        {
            int kernelSize = _blurSize * 2 + 1; // 核大小（7x7）
            int d = _grid.NodeXCount > _grid.NodeYCount ? _grid.NodeXCount : _grid.NodeYCount;

            // 水平方向和垂直方向的中间结果数组
            NativeArray<int> horizontalPass = new NativeArray<int>(_grid.NodeXCount * _grid.NodeYCount, Allocator.TempJob);
            NativeArray<int> verticalPass = new NativeArray<int>(_grid.NodeXCount * _grid.NodeYCount, Allocator.TempJob);

            // ======== 第一遍：水平方向箱式滤波 ========
            // 对每行，使用滑动窗口累加 [x-blurSize, x+blurSize] 范围内的惩罚值
            for (int y = 0; y < _grid.NodeYCount; y++)
            {
                // 初始化第一列（x=0）的滑动窗口：累加 [-blurSize, blurSize] 范围（左侧 clamp 到 0）
                for (int x = -_blurSize; x <= _blurSize; x++)
                {
                    int clampX = math.clamp(x, 0, _blurSize);
                    int index;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        index = clampX + y * d;
                    else index = y + clampX * d;

                    int hpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        hpIndex = 0 + y * d;
                    else hpIndex = y + 0 * d;

                    horizontalPass[hpIndex] += _nodes[index].Node.Penalty;
                }

                // 滑动窗口：每次向右移动一格，减去最左边的值，加上最右边的新值
                for (int x = 1; x < _grid.NodeXCount; x++)
                {
                    int removeIndex = math.clamp(x - _blurSize - 1, 0, _grid.NodeXCount);
                    int addIndex = math.clamp(x + _blurSize, 0, _grid.NodeXCount - 1);

                    // 计算相关一维索引（省略重复代码说明）
                    int removehpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        removehpIndex = removeIndex + y * d;
                    else removehpIndex = y + removeIndex * d;

                    int addhpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        addhpIndex = addIndex + y * d;
                    else addhpIndex = y + addIndex * d;

                    int hpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        hpIndex = x + y * d;
                    else hpIndex = y + x * d;

                    int prevhpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        prevhpIndex = (x - 1) + y * d;
                    else prevhpIndex = y + (x - 1) * d;

                    // 滑动：窗口和 = 前一个窗口和 - 离开窗口的值 + 进入窗口的值
                    horizontalPass[hpIndex] = horizontalPass[prevhpIndex] - _nodes[removehpIndex].Node.Penalty + _nodes[addhpIndex].Node.Penalty;
                }
            }

            // ======== 第二遍：垂直方向箱式滤波 ========
            // 对每列，使用滑动窗口累加 [y-blurSize, y+blurSize] 范围内的水平方向结果
            for (int x = 0; x < _grid.NodeXCount; x++)
            {
                // 初始化第一行（y=0）的滑动窗口
                for (int y = -_blurSize; y <= _blurSize; y++)
                {
                    int clampY = math.clamp(y, 0, _blurSize);
                    int index;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        index = x + clampY * d;
                    else index = clampY + x * d;

                    int vpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        vpIndex = x + 0 * d;
                    else vpIndex = 0 + x * d;

                    verticalPass[vpIndex] += horizontalPass[index];
                }

                // 滑动窗口
                for (int y = 1; y < _grid.NodeYCount; y++)
                {
                    int vpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        vpIndex = x + y * d;
                    else vpIndex = y + x * d;

                    int prevvpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        prevvpIndex = x + (y - 1) * d;
                    else prevvpIndex = (y - 1) + x * d;

                    int removeIndex = math.clamp(y - _blurSize - 1, 0, _grid.NodeYCount);
                    int addIndex = math.clamp(y + _blurSize, 0, _grid.NodeYCount - 1);

                    int removevpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        removevpIndex = x + removeIndex * d;
                    else removevpIndex = removeIndex + x * d;

                    int addvpIndex;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        addvpIndex = x + addIndex * d;
                    else addvpIndex = addIndex + x * d;

                    // 垂直方向滑动
                    verticalPass[vpIndex] = verticalPass[prevvpIndex] - horizontalPass[removevpIndex] + horizontalPass[addvpIndex];

                    // 最终模糊值 = 两次滑动窗口之和 / 核面积（kernelSize²）
                    int blurredWeight = (int)math.round((float)verticalPass[vpIndex] / (kernelSize * kernelSize));

                    int index;
                    if (_grid.NodeXCount > _grid.NodeYCount)
                        index = x + y * d;
                    else index = y + x * d;

                    // 将模糊后的惩罚值写回节点
                    GridNode newNode = _nodes[index].Node;
                    newNode.Penalty = blurredWeight;
                    _nodes[index] = new GridNodesBuffer() { Node = newNode };
                }
            }

            horizontalPass.Dispose();
            verticalPass.Dispose();
        }
    }
}
