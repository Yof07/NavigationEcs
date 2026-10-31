using Unity.Entities;
using Unity.Collections;
using Unity.Mathematics;

namespace Navigation
{
    /// <summary>
    /// 一次寻路操作的输入数据组件，由PathfindingRequestSystem创建并挂载到临时Entity上，
    /// 寻路完成后该Entity由AgentMovementSystem销毁
    /// </summary>
    public struct PathfindingOperation : IComponentData
    {
        /// <summary>起始世界坐标（代理当前位置）</summary>
        public float3 StartPoint;

        /// <summary>目标世界坐标（代理期望到达的位置）</summary>
        public float3 TargetPoint;
    }

    /// <summary>
    /// 路径点缓冲组件，存储A* 算法规划出的完整路径（有序路径点列表）
    /// 同时挂载在：
    /// 1. 寻路操作Entity（临时）：PathfindingSystem写入结果
    /// 2. 代理Entity（持久）：AgentMovementSystem从操作Entity复制到此处后跟随
    /// InternalBufferCapacity(0) 表示所有数据存储在堆内存中，路径长度不受限制
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct PathBuffer : IBufferElementData
    {
        /// <summary>
        /// 路径点的世界坐标（按 起点→终点 顺序排列）
        /// 即一条路径是由多个点组成的，DynamicBuffer<PathBuffer> 就存储了一个代理的一条路径
        /// </summary>
        public float3 Point;
    }

    /// <summary>
    /// 寻路操作结果组件，与PathfindingOperation挂载在同一临时Entity上
    /// PathfindingSystem写入结果，AgentMovementSystem读取后处理路径并销毁该Entity
    /// </summary>
    public struct PathfindingResult : IComponentData
    {
        /// <summary>是否找到路径（即使目标不可达也可能为true，此时返回尽量接近目标的路径）</summary>
        public bool PathFound;

        /// <summary>寻路是否已完成（完成后AgentMovementSystem才会读取结果）</summary>
        public bool FinishedSearch;
    }

    /// <summary>
    /// 寻路请求队列缓冲组件，挂载在Grid Entity上作为全局请求队列
    /// PathfindingRequestSystem将代理的寻路请求追加到此队列，
    /// 然后按MaxPathfindingOpsPerFrame的限制逐帧取出并创建操作Entity
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct PathfindingRequestQueue : IBufferElementData
    {
        /// <summary>发起寻路请求的代理Entity（用于寻路完成后将路径写回该代理）</summary>
        public Entity RequestingAgentEntity;

        /// <summary>该代理的寻路操作数据（起点和终点）</summary>
        public PathfindingOperation Operation;
    }

    /// <summary>
    /// 寻路请求组件，代理通过此组件向寻路系统提交请求
    /// 将RequestGiven置为false即可触发新的寻路请求
    ///
    /// 归属Pathfinding：它是Movement ↔ Pathfinding的契约，
    /// 与PathfindingOperation / PathfindingResult / PathBuffer同一层，
    /// 这样PathfindingRequestSystem不需要引用Movement的类型
    /// </summary>
    public struct PathfindingRequester : IComponentData
    {
        /// <summary>目标世界坐标</summary>
        public float3 Destination;

        /// <summary>
        /// 请求是否已提交
        /// false = 待处理的新请求，系统处理后置为true
        /// </summary>
        public bool RequestGiven;

        /// <summary>
        /// 对应的寻路操作Entity（由PathfindingRequestSystem自动创建和管理）
        /// 不要手动设置此值
        /// </summary>
        public Entity RequestEntity;
    }
}
