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
    /// 存储代理 Prefab 对应的 ECS Entity 引用，供 GTSpawnerSystem 读取并批量实例化代理。
    /// </summary>
    public struct GTAgentPrefabComponent : IComponentData
    {
        public Entity Value;
    }

    /// <summary>
    /// GroupTarget 场景的代理生成器 Authoring 组件。
    /// 在 SubScene 烘焙阶段将 mAgentPrefab 转换为 ECS Entity 引用，
    /// 由 GTSpawnerSystem 在运行时读取并批量生成持续追踪目标球的代理。
    /// </summary>
    public class GTSpawner : MonoBehaviour
    {
        /// <summary> 
        /// 代理 Prefab（需包含 AgentAuthoring 和 ConstantTargetFollower 所需组件），
        /// 即 GroupTarget 场景中的 Agent 预制体，代理的"皮套"
        /// </summary>
        public GameObject mAgentPrefab;

        /// <summary>
        /// 每圈同心圆的半径与代理数量，可在 Inspector 里增删。
        /// 默认 2 圈：20 + 20 = 40 个代理。
        /// 改完需要重新烘焙（保存 SubScene 或重进 Play）。
        /// </summary>
        public SpawnRing[] Rings =
        {
            new SpawnRing { Radius = 10, Count = 20 },
            new SpawnRing { Radius = 11, Count = 20 },
        };

        public class GTSpawnerBaker : Baker<GTSpawner>
        {
            public override void Bake(GTSpawner authoring)
            {
                // WorldSpace：Spawner 自身不需要跟随父节点变换
                var entity = GetEntity(TransformUsageFlags.WorldSpace);
                // Dynamic：代理在运行时会移动，需要动态 Transform
                // 烘焙的是一个 prefab 类型的资产，authoring.mAgentPrefab 指向一个 prefab 类型的资产
                // 烘焙它的 baker 是 AgentAuthoring 里的 AgentBaker
                // 这句代码执行完后，对于 authoring.mAgentPrefab 这个 prefab 类型的资产，它已经有了实体，但还没被烘焙
                var agentEntity = GetEntity(authoring.mAgentPrefab, TransformUsageFlags.Dynamic);

                var prefCom = new GTAgentPrefabComponent()
                {
                    Value = agentEntity,
                };

                AddComponent(entity, prefCom);

                // 把 Inspector 的环配置写进 buffer，供 GTSpawnerSystem 读取
                DynamicBuffer<SpawnRing> rings = AddBuffer<SpawnRing>(entity);
                SpawnRing[] ringConfig = authoring.Rings;

                if (ringConfig == null || ringConfig.Length == 0)
                {
                    ringConfig = new[]
                    {
                        new SpawnRing { Radius = 10, Count = 20 },
                        new SpawnRing { Radius = 11, Count = 20 },
                    };
                }

                for (int i = 0; i < ringConfig.Length; i++)
                {
                    rings.Add(ringConfig[i]);
                }
            }
        }
    }
}
