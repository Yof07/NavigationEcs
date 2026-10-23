using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Navigation
{
    /// <summary>
    /// 路径跟随的共享逻辑：Y 轴固定、路径点推进、终点判定、平滑转向。
    /// 阶段 2（无避障）与阶段 4（避障）用的是同一套判定，差异只有
    /// 「最后一个路径点的触发距离」——由调用方传入。
    /// </summary>
    public static class PathFollowing
    {
        /// <summary>
        /// 障碍射线的起点：脚底上方 0.1 米（避免与地面碰撞体抢先命中）。
        /// 代理中心固定在 Height/2，所以底部是 Position - Up * (Height/2)，不能减整个 Height。
        /// </summary>
        public static float3 ObstacleRayOrigin(float3 _center, in Agent _agent)
        {
            return new float3(_center.x, _center.y - _agent.Height * 0.5f + 0.1f, _center.z);
        }

        /// <summary>强制 Y 轴固定在代理中心高度（防止 Y 轴漂移）</summary>
        public static void ClampToAgentHeight(ref LocalTransform _transform, in Agent _agent)
        {
            if (math.abs(_transform.Position.y - _agent.Height / 2) > 0.001f)
            {
                _transform.Position.y = _agent.Height / 2;
            }
        }

        /// <summary>取当前路径点（Y 对齐代理高度）</summary>
        public static float3 CurrentPoint(DynamicBuffer<PathBuffer> _path, in Agent _agent, int _index)
        {
            float3 point = _path[_index].Point;
            point.y = _agent.Height / 2;
            return point;
        }

        /// <summary>
        /// 按当前路径点推进索引：
        /// - 非最后一个点：阈值 = Radius + NextTargetTouchDistance（负值取 Grid.NodeSize）；
        /// - 最后一个点：阈值 = _lastPointTouchDistance（阶段 2 用 Radius + StoppingDistance，阶段 4 只用 Radius）。
        /// </summary>
        public static void AdvanceIndex(
            ref PathFollower _follower,
            in Agent _agent,
            DynamicBuffer<PathBuffer> _path,
            float3 _position,
            float _nodeSize,
            float _lastPointTouchDistance)
        {
            float3 point = CurrentPoint(_path, _agent, _follower.TargetIndex);

            if (_follower.TargetIndex != _path.Length - 1)
            {
                float nextTouchDistance = _agent.NextTargetTouchDistance < 0
                    ? _nodeSize
                    : _agent.NextTargetTouchDistance;

                if (math.distance(point, _position) < _agent.Radius + nextTouchDistance)
                {
                    _follower.TargetIndex++;
                }
            }
            else if (math.distance(point, _position) < _lastPointTouchDistance)
            {
                _follower.TargetIndex++;
            }
        }

        /// <summary>是否足够接近最终目的地（精确停止判定）</summary>
        public static bool IsFinalDestinationReached(in PathfindingRequester _requester, in Agent _agent, float3 _position)
        {
            float3 finalDestinationPoint = _requester.Destination;
            finalDestinationPoint.y = _agent.Height / 2;

            return math.distance(finalDestinationPoint, _position) < _agent.Radius + _agent.StoppingDistance;
        }

        /// <summary>朝移动方向平滑旋转</summary>
        public static void ApplyRotation(ref LocalTransform _transform, float3 _direction, float _turnSpeed, float _deltaTime)
        {
            quaternion destRot = quaternion.LookRotationSafe(_direction, _transform.Up());
            float ang = math.angle(destRot, _transform.Rotation);

            if (math.abs(ang) > 0.01f)
            {
                float angStep = math.min(ang, math.radians(_deltaTime * _turnSpeed));
                _transform.Rotation = math.slerp(_transform.Rotation, destRot, angStep / ang);
            }
        }
    }
}
