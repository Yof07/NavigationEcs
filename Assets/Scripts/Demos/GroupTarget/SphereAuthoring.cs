using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Physics;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;

namespace Navigation.Demos
{
    /// <summary>
    /// 测试目标球的Authoring组件，挂载在GroupTarget Demo场景中的目标球GameObject上
    ///
    /// 烘焙后，对应Entity会被添加TargetSphereMarker标记组件，
    /// 用途：
    ///   1. GTSpawnerSystem通过SystemAPI.GetSingletonEntity&lt;TargetSphereMarker&gt;() 找到目标球Entity，
    ///      并将其赋值给代理的ConstantTargetFollower.Target，使代理持续追踪它
    ///   2. MoveSphere MonoBehaviour通过EntityManager.CreateEntityQuery找到此Entity，
    ///      在每帧根据方向键输入更新其LocalTransform.Position，实现玩家控制目标球移动
    ///
    /// 注意：场景中只应有一个挂载SphereAuthoring的GameObject，
    /// 否则GetSingletonEntity会抛出异常（要求有且仅有一个匹配Entity）
    /// </summary>
    public class SphereAuthoring : MonoBehaviour
    {
        /// <summary>
        /// Baker内部类：将SphereAuthoring烘焙为带TargetSphereMarker标记组件的ECS Entity
        /// 在SubScene烘焙阶段自动执行
        /// </summary>
        public class SphereBaker : Baker<SphereAuthoring>
        {
            public override void Bake(SphereAuthoring authoring)
            {
                // Dynamic：目标球在运行时由MoveSphere脚本驱动移动，需要动态Transform
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                // TargetSphereMarker是纯标记组件（空struct），无需设置任何字段
                AddComponent(entity, new TargetSphereMarker() { });
            }
        }
    }
}