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
    /// 存储代理Prefab对应的ECS Entity引用，供COASpawnerSystem读取并批量实例化代理
    /// </summary>
    public struct AgentPrefabComponent : IComponentData
    {
        public Entity Value;
    }

    /// <summary>
    /// COA（Circle Of Agents）场景的代理生成器Authoring组件
    /// 在SubScene烘焙阶段将mAgentPrefab转换为ECS Entity引用，
    /// 由COASpawnerSystem在运行时读取并批量生成代理
    /// </summary>
    public class COASpawner : MonoBehaviour
    {
        /// <summary>代理Prefab（需包含AgentAuthoring组件）</summary>
        public GameObject mAgentPrefab;

        /// <summary>
        /// 每圈同心圆的半径与代理数量，可在Inspector里增删
        /// 默认4圈：75 + 80 + 85 + 90 = 330个代理
        /// 改完需要重新烘焙（保存SubScene或重进Play）
        /// </summary>
        public SpawnRing[] Rings =
        {
            new SpawnRing { Radius = 25, Count = 74 },
            new SpawnRing { Radius = 27, Count = 75 },
            new SpawnRing { Radius = 29, Count = 76 },
            new SpawnRing { Radius = 31, Count = 77 },
        };

        public class COASpawnerBaker : Baker<COASpawner>
        {
            public override void Bake(COASpawner authoring)
            {
                // Dynamic：代理在运行时会移动，需要动态Transform
                var agentEntity = GetEntity(authoring.mAgentPrefab, TransformUsageFlags.Dynamic);

                var prefCom = new AgentPrefabComponent()
                {
                    Value = agentEntity,
                };
                // WorldSpace：Spawner自身不需要跟随父节点变换
                var entity = GetEntity(TransformUsageFlags.WorldSpace);
                AddComponent(entity, prefCom);

                // 把Inspector的环配置写进buffer，供COASpawnerSystem读取
                DynamicBuffer<SpawnRing> rings = AddBuffer<SpawnRing>(entity);
                SpawnRing[] ringConfig = authoring.Rings;

                if (ringConfig == null || ringConfig.Length == 0)
                {
                    ringConfig = new[]
                    {
                        new SpawnRing { Radius = 25, Count = 75 },
                        new SpawnRing { Radius = 26, Count = 80 },
                        new SpawnRing { Radius = 27, Count = 85 },
                        new SpawnRing { Radius = 28, Count = 90 },
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
