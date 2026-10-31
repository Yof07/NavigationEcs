using System;
using Unity.Entities;

namespace Navigation.Demos
{
    /// <summary>
    /// 一圈同心圆的生成参数：半径（世界单位）与该圈的代理数量
    /// 由COASpawner / GTSpawner在Inspector里配置，
    /// Baker写成DynamicBuffer，SpawnerSystem运行时读取
    /// </summary>
    [Serializable]
    [InternalBufferCapacity(8)]
    public struct SpawnRing : IBufferElementData
    {
        public int Radius;
        public int Count;
    }
}
