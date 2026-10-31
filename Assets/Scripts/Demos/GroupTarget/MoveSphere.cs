using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;

namespace Navigation.Demos
{
    /// <summary>
    /// GroupTarget场景中目标球的控制器（普通MonoBehaviour，挂载在场景外的GameObject上）
    /// 通过方向键移动ECS中的TargetSphereMarker Entity，所有挂载ConstantTargetFollower的代理
    /// 会自动检测到目标位置变化并重新寻路
    /// </summary>
    public class MoveSphere : MonoBehaviour
    {
        private EntityManager EM;
        private Entity SphereEntity;

        private float Speed = 10;

        void Start()
        {
            // 延迟1秒初始化，等待ECS World和SubScene完成烘焙
            StartCoroutine(Init());
        }

        IEnumerator Init()
        {
            yield return new WaitForSeconds(1);
            EM = World.DefaultGameObjectInjectionWorld.EntityManager;
            // 查询场景中唯一的TargetSphereMarker单例Entity，只要挂了TargetSphereMarker就能筛选出来
            var q = EM.CreateEntityQuery(ComponentType.ReadOnly<TargetSphereMarker>());
            SphereEntity = q.GetSingletonEntity();
            q.Dispose();
        }

        void Update()
        {
            EM = World.DefaultGameObjectInjectionWorld.EntityManager;

            if (EM.Exists(SphereEntity))
            {
                LocalTransform lt = EM.GetComponentData<LocalTransform>(SphereEntity);

                // 方向键控制目标球在XZ平面移动
                if (Input.GetKey(KeyCode.RightArrow))
                {
                    lt.Position.x += Time.deltaTime * Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }

                if (Input.GetKey(KeyCode.LeftArrow))
                {
                    lt.Position.x += Time.deltaTime * -Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }

                if (Input.GetKey(KeyCode.UpArrow))
                {
                    lt.Position.z += Time.deltaTime * Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }

                if (Input.GetKey(KeyCode.DownArrow))
                {
                    lt.Position.z += Time.deltaTime * -Speed;
                    EM.SetComponentData(SphereEntity, lt);
                }
            }
        }
    }
}
