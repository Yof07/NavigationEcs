using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Physics;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Transforms;


namespace Navigation
{
    /// <summary>
    /// 障碍物的Authoring组件，挂载在需要标记为"不可通行"的GameObject上
    ///
    /// 使用前提：
    /// 1. 该GameObject必须同时拥有碰撞体（Collider / PhysicsShape）
    /// 2. 碰撞体所在的Layer必须与GridAuthoring.InaccessibleNodes的LayerMask一致，
    ///    否则CheckObstaclesSystem的OverlapBox检测将无法命中此障碍物
    ///
    /// 烘焙后，CheckObstaclesSystem会在运行时通过物理重叠检测找到此Entity，
    /// 并将被碰撞体覆盖的网格节点标记为IsAccessible = false（不可通行）
    ///
    /// PenaltyOnObstacle可选设置边缘惩罚值：
    ///   - 值 = 0：节点仅被标记为不可通行，无额外惩罚扩散
    ///   - 值 > 0：在障碍物节点上叠加此惩罚值，经高斯模糊扩散后，
    ///             周边节点也会有一定惩罚值，引导路径自然远离障碍物边缘
    /// </summary>
    public class ObstacleAuthoring : MonoBehaviour
    {
        /// <summary>
        /// 障碍物边缘惩罚值（可选，0表示不启用边缘惩罚扩散）
        /// 值越大，代理路径越倾向于远离此障碍物边缘（但不影响节点的可通行性判断）
        /// </summary>
        public int PenaltyOnObstacle;

        /// <summary>
        /// Baker内部类：将ObstacleAuthoring参数烘焙为ECS组件Obstacle
        /// 在SubScene烘焙阶段自动执行，生成静态ECS数据
        /// </summary>
        public class ObstacleBaker : Baker<ObstacleAuthoring>
        {
            public override void Bake(ObstacleAuthoring authoring)
            {
                // Dynamic：障碍物可能在运行时移动（如移动平台），需要动态Transform
                var entity = GetEntity(TransformUsageFlags.Dynamic);

                AddComponent(entity, new Obstacle()
                {
                    PenaltyOnObstacleValue = authoring.PenaltyOnObstacle
                });
            }
        }
    }
}