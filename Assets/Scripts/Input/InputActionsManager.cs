using UnityEngine;
using UnityEngine.InputSystem;

// 注意：命名空间不能叫 Navigation.Input —— 会遮住 UnityEngine.Input，
// 让工程里其它文件的 Input.GetKeyDown 解析失败。统一放 Navigation。
namespace Navigation
{
    /// <summary>
    /// 输入基底：把新 Input System 的动作收成一个只读门面，
    /// 其它脚本（框选、相机、命令）只读属性，不直接碰 InputAction。
    ///
    /// 动作定义在 <c>Assets/Input/GlobalInput.inputactions</c>（map = "Gameplay"），
    /// 并勾了 Generate C# Class，所以这里用的是生成的强类型包装 <see cref="GlobalInput"/>：
    /// 名字写错会在编译期报错，不用字符串查找，也不需要在 Inspector 上拖资源引用。
    ///
    /// 用法：场景里放一个空 GameObject，挂上本组件（建议做成 Prefab，两个场景各放一份）。
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class InputActionsManager : MonoBehaviour
    {
        /// <summary>全局访问点。场景里只允许一个，重复挂载会被 DisallowMultipleComponent 拦住。</summary>
        public static InputActionsManager Instance { get; private set; }

        /// <summary>生成的包装类实例；它内部从 .inputactions 的 JSON 构建出 InputActionAsset</summary>
        private GlobalInput m_Input;

        /// <summary>Gameplay map；Enable/Disable 整张 map 即可一次开关所有动作</summary>
        private InputActionMap m_Map;

        /// <summary>指针屏幕坐标（像素）</summary>
        private InputAction m_Pointer;

        /// <summary>左键：框选</summary>
        private InputAction m_Select;

        /// <summary>右键：下令</summary>
        private InputAction m_Command;

        /// <summary>滚轮：缩放（每格约 120）</summary>
        private InputAction m_Scroll;

        /// <summary>中键按住：拖拽平移</summary>
        private InputAction m_PanDrag;

        /// <summary>Q / E：旋转，-1 ~ +1</summary>
        private InputAction m_Rotate;

        // ----- 对外只读接口 -----

        /// <summary>指针位置（屏幕像素）</summary>
        public Vector2 PointerPosition => m_Pointer.ReadValue<Vector2>();

        /// <summary>本帧按下左键</summary>
        public bool SelectPressedThisFrame => m_Select.WasPressedThisFrame();

        /// <summary>本帧松开左键</summary>
        public bool SelectReleasedThisFrame => m_Select.WasReleasedThisFrame();

        /// <summary>左键是否按住</summary>
        public bool SelectHeld => m_Select.IsPressed();

        /// <summary>本帧按下右键</summary>
        public bool CommandPressedThisFrame => m_Command.WasPressedThisFrame();

        /// <summary>滚轮增量（向上为正）</summary>
        public float ScrollDelta => m_Scroll.ReadValue<float>();

        /// <summary>是否按住中键（拖拽平移）</summary>
        public bool PanHeld => m_PanDrag.IsPressed();

        /// <summary>旋转输入：Q = -1，E = +1，都没按 = 0</summary>
        public float RotateAxis => m_Rotate.ReadValue<float>();

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Debug.LogWarning($"[Input] 场景里存在多个 {nameof(InputActionsManager)}，已销毁多余的那个。");
                Destroy(this);
                return;
            }

            Instance = this;

            // 包装类自己持有 asset（从 JSON 构建），所以不需要 Resources.Load
            m_Input = new GlobalInput();
            m_Map = m_Input.Gameplay.Get();

            m_Pointer = m_Input.Gameplay.Pointer;
            m_Select = m_Input.Gameplay.Select;
            m_Command = m_Input.Gameplay.Command;
            m_Scroll = m_Input.Gameplay.Scroll;
            m_PanDrag = m_Input.Gameplay.PanDrag;
            m_Rotate = m_Input.Gameplay.Rotate;
        }

        private void OnEnable()
        {
            m_Map?.Enable();
        }

        private void OnDisable()
        {
            m_Map?.Disable();
        }

        private void OnDestroy()
        {
            // 包装类的 asset 是 new 出来的，必须 Dispose，否则每次进 Play 都泄漏一份
            m_Input?.Dispose();

            if (Instance == this)
            {
                Instance = null;
            }
        }
    }
}
