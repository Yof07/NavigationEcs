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
    /// 存储代理Prefab对应的ECS Entity引用，供GTSpawnerSystem读取并批量实例化代理
    /// </summary>
    public struct GTAgentPrefabComponent : IComponentData
    {
        public Entity Value;
    }

    /// <summary>
    /// GroupTarget场景的代理生成器Authoring组件
    /// 在SubScene烘焙阶段将mAgentPrefab转换为ECS Entity引用，
    /// 由GTSpawnerSystem在运行时读取并批量生成持续追踪目标球的代理
    /// </summary>
    public class GTSpawner : MonoBehaviour
    {
        /// <summary> 
        /// 代理Prefab（需包含AgentAuthoring和ConstantTargetFollower所需组件），
        /// 即GroupTarget场景中的Agent预制体，代理的"皮套"
        /// </summary>
        public GameObject mAgentPrefab;

        /// <summary>
        /// 每圈同心圆的半径与代理数量，可在Inspector里增删
        /// 默认2圈：20 + 20 = 40个代理
        /// 改完需要重新烘焙（保存SubScene或重进Play）
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
                // WorldSpace：Spawner自身不需要跟随父节点变换
                var entity = GetEntity(TransformUsageFlags.WorldSpace);
                // Dynamic：代理在运行时会移动，需要动态Transform
                // 烘焙的是一个prefab类型的资产，authoring.mAgentPrefab指向一个prefab类型的资产
                // 烘焙它的baker是AgentAuthoring里的AgentBaker
                // 这句代码执行完后，对于authoring.mAgentPrefab这个prefab类型的资产，它已经有了实体，但还没被烘焙
                var agentEntity = GetEntity(authoring.mAgentPrefab, TransformUsageFlags.Dynamic);

                var prefCom = new GTAgentPrefabComponent()
                {
                    Value = agentEntity,
                };

                AddComponent(entity, prefCom);

                // 把Inspector的环配置写进buffer，供GTSpawnerSystem读取
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
