using UnityEngine;

namespace Navigation
{
    /// <summary>
    /// RTS 相机：焦点 + 球坐标模型。
    ///
    /// 相机的位置不用直接拖 Transform 来维护，而是由三个量算出来：
    /// - Pivot    焦点，地面上的关注点（平移改它）
    /// - Distance 相机到焦点的距离（缩放改它）
    /// - Yaw      水平朝向角（Q/E 改它），Pitch 固定不变
    ///
    /// 每帧：position = pivot - Euler(pitch, yaw, 0) * forward * distance
    /// 三者互不干扰，所以平移/缩放/旋转可以同时进行而不会互相带偏。
    ///
    /// 输入只读 InputActionsManager 的属性，不直接依赖 Input System 的类型；
    /// 以后换输入方案（比如退回旧 Input API）不需要动这个脚本。
    ///
    /// 挂在 Main Camera 上即可。
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public sealed class CameraController : MonoBehaviour
    {
        [Header("焦点与视角")]
        [Tooltip("地面上的关注点。地图中心通常是原点；拖动平移时改的就是它")]
        [SerializeField] private Vector3 m_Pivot = Vector3.zero;

        [Tooltip("俯角（度）。固定值，不参与输入；RTS 常用 45~60")]
        [SerializeField, Range(20f, 89f)] private float m_Pitch = 55f;

        [Tooltip("水平朝向角（度）。Q/E 旋转改的是它的期望值")]
        [SerializeField] private float m_Yaw = 0f;

        [Header("距离")]
        [Tooltip("相机到焦点的初始距离（米）")]
        [SerializeField] private float m_Distance = 40f;

        [Tooltip("最近能拉到多近；太小会钻进地面")]
        [SerializeField] private float m_MinDistance = 8f;

        [Tooltip("最远能拉到多远；太大代理会缩成一个点")]
        [SerializeField] private float m_MaxDistance = 90f;

        [Tooltip("滚轮灵敏度：每 1 单位滚轮增量改变距离的比例（滚一格约 120 单位）")]
        [SerializeField] private float m_ZoomStep = 0.002f;

        [Header("平移")]
        [Tooltip("中键拖拽的灵敏度倍数；1 = 内容跟手移动")]
        [SerializeField] private float m_PanSpeed = 1f;

        [Header("旋转")]
        [Tooltip("按住 Q/E 时每秒转多少度")]
        [SerializeField] private float m_RotateSpeed = 90f;

        [Header("平滑")]
        [Tooltip("期望值追上来的时间（秒）。0 = 不做平滑，输入立刻生效")]
        [SerializeField] private float m_SmoothTime = 0.06f;

        [Header("焦点范围（XZ 平面）")]
        [Tooltip("焦点允许的最小 X / Z；用来防止把相机拖出地图")]
        [SerializeField] private Vector2 m_PivotMin = new Vector2(-40f, -40f);

        [Tooltip("焦点允许的最大 X / Z")]
        [SerializeField] private Vector2 m_PivotMax = new Vector2(40f, 40f);

        /// <summary>相机组件，Awake 里缓存，避免每帧 GetComponent</summary>
        private Camera m_Camera;

        /// <summary>上一帧的指针位置；和本帧相减得到像素位移（中键拖拽平移用）</summary>
        private Vector2 m_LastPointer;

        // ----- 「期望值」：输入直接修改这三个，代表玩家想去的位置/距离/角度 -----

        /// <summary>期望的焦点（世界坐标，Y 恒为 0 的地面点）</summary>
        private Vector3 m_TargetPivot;

        /// <summary>期望的相机距离（米）</summary>
        private float m_TargetDistance;

        /// <summary>期望的水平角（度）</summary>
        private float m_TargetYaw;

        // ----- 「当前值」：每帧朝期望值平滑逼近，实际用来算相机位姿 -----
        // 分开存是为了缓动：滚轮一格会跳几十毫秒的变化，直接套到相机上会显得顿。

        /// <summary>当前实际使用的焦点</summary>
        private Vector3 m_CurrentPivot;

        /// <summary>当前实际使用的距离</summary>
        private float m_CurrentDistance;

        /// <summary>当前实际使用的水平角</summary>
        private float m_CurrentYaw;

        // ----- SmoothDamp 需要的速度缓存（不要手动改，由函数自己维护）-----

        /// <summary>焦点平滑的速度缓存</summary>
        private Vector3 m_PivotVelocity;

        /// <summary>距离平滑的速度缓存</summary>
        private float m_DistanceVelocity;

        /// <summary>角度平滑的速度缓存</summary>
        private float m_YawVelocity;

        private void Awake()
        {
            m_Camera = GetComponent<Camera>();

            // 首帧让当前值 = 期望值，避免从 (0,0,0) 平滑过来时相机飞一段
            m_TargetPivot = m_Pivot;
            m_TargetDistance = m_Distance;
            m_TargetYaw = m_Yaw;

            m_CurrentPivot = m_TargetPivot;
            m_CurrentDistance = m_TargetDistance;
            m_CurrentYaw = m_TargetYaw;

            ApplyPose();
        }

        private void Update()
        {
            InputActionsManager input = InputActionsManager.Instance;

            if (input == null)
            {
                return;
            }

            Vector2 pointer = input.PointerPosition;

            HandlePan(input, pointer);
            HandleZoom(input);
            HandleRotate(input);

            // 放在最后：本帧的指针位置成为下一帧位移计算的起点
            m_LastPointer = pointer;

            ClampPivot();
            SmoothTowardsTarget();
            ApplyPose();
        }

        /// <summary>
        /// 中键拖拽平移：把像素位移换算成世界位移。
        ///
        /// 换算用的是「焦点平面处，屏幕高度方向覆盖多少世界单位」：
        ///   2 * distance * tan(fov/2) / Screen.height
        /// 透视相机下这个比值随距离变化（越远，一像素对应越长的距离），
        /// 所以必须乘当前距离，否则拉远之后拖拽会变得跟不上手。
        ///
        /// 方向用「水平面上的前向」而不是相机自身前向：相机是斜的，
        /// 直接用相机前向会让拖拽带一个向下的分量，表现是越拖越往地里钻。
        /// </summary>
        private void HandlePan(InputActionsManager _input, Vector2 _pointer)
        {
            if (!_input.PanHeld)
            {
                return;
            }

            Vector2 delta = _pointer - m_LastPointer;

            if (delta.sqrMagnitude <= 0f)
            {
                return;
            }

            float worldPerPixel = 2f * m_TargetDistance * Mathf.Tan(m_Camera.fieldOfView * 0.5f * Mathf.Deg2Rad) / Screen.height;

            Quaternion yawRotation = Quaternion.Euler(0f, m_TargetYaw, 0f);
            Vector3 right = yawRotation * Vector3.right;
            Vector3 forward = yawRotation * Vector3.forward;

            // 减号：往右拖，焦点向左走 —— 视觉上内容跟着鼠标走（抓住地图拖）
            m_TargetPivot -= (right * delta.x + forward * delta.y) * (worldPerPixel * m_PanSpeed);
        }

        /// <summary>
        /// 滚轮缩放。步长按距离成比例（乘 m_TargetDistance），
        /// 所以近处每次只挪一点、远处一次跨度大，手感比固定步长自然。
        /// </summary>
        private void HandleZoom(InputActionsManager _input)
        {
            float scroll = _input.ScrollDelta;

            if (Mathf.Abs(scroll) < 0.01f)
            {
                return;
            }

            m_TargetDistance = Mathf.Clamp(
                m_TargetDistance - scroll * m_ZoomStep * m_TargetDistance,
                m_MinDistance,
                m_MaxDistance);
        }

        /// <summary>
        /// Q/E 旋转。只改水平角，俯角固定，
        /// 所以转一圈下来焦点仍在原地、焦距不变。
        /// </summary>
        private void HandleRotate(InputActionsManager _input)
        {
            float rotate = _input.RotateAxis;

            if (Mathf.Abs(rotate) < 0.01f)
            {
                return;
            }

            m_TargetYaw += rotate * m_RotateSpeed * Time.deltaTime;
        }

        /// <summary>把期望焦点夹在地图范围内，防止相机被拖到地图外面看空气</summary>
        private void ClampPivot()
        {
            m_TargetPivot.x = Mathf.Clamp(m_TargetPivot.x, m_PivotMin.x, m_PivotMax.x);
            m_TargetPivot.z = Mathf.Clamp(m_TargetPivot.z, m_PivotMin.y, m_PivotMax.y);
        }

        /// <summary>
        /// 让当前值逼近期望值。三个量各自独立平滑：
        /// 焦点用 Vector3.SmoothDamp，距离用 Mathf.SmoothDamp，
        /// 角度用 SmoothDampAngle（走最短弧，跨越 ±180° 时不会绕一大圈）。
        /// </summary>
        private void SmoothTowardsTarget()
        {
            if (m_SmoothTime <= 0f)
            {
                m_CurrentPivot = m_TargetPivot;
                m_CurrentDistance = m_TargetDistance;
                m_CurrentYaw = m_TargetYaw;
                return;
            }

            m_CurrentPivot = Vector3.SmoothDamp(m_CurrentPivot, m_TargetPivot, ref m_PivotVelocity, m_SmoothTime);
            m_CurrentDistance = Mathf.SmoothDamp(m_CurrentDistance, m_TargetDistance, ref m_DistanceVelocity, m_SmoothTime);
            m_CurrentYaw = Mathf.SmoothDampAngle(m_CurrentYaw, m_TargetYaw, ref m_YawVelocity, m_SmoothTime);
        }

        /// <summary>
        /// 由焦点 / 距离 / 角度算出最终位姿：
        /// 从焦点沿「相机朝向的反方向」退 distance 米，朝向始终看焦点。
        /// </summary>
        private void ApplyPose()
        {
            Quaternion rotation = Quaternion.Euler(m_Pitch, m_CurrentYaw, 0f);
            Vector3 position = m_CurrentPivot - rotation * Vector3.forward * m_CurrentDistance;

            transform.SetPositionAndRotation(position, rotation);
        }
    }
}
