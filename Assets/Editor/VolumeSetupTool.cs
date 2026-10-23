using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace Navigation.EditorTools
{
    /// <summary>
    /// 一键配置后处理：创建/更新 DefaultVolumeProfile，把它挂到场景里的 Global Volume，
    /// 并打开主相机的 Post Processing。
    ///
    /// 用法：菜单 Tools → Setup Default Volume（每个要用的场景跑一次）。
    /// 跑完记得 Ctrl+S 保存场景，否则 Volume GameObject 只在内存里。
    /// </summary>
    public static class VolumeSetupTool
    {
        /// <summary>Volume Profile 资源路径</summary>
        private const string ProfilePath = "Assets/Settings/DefaultVolumeProfile.asset";

        [MenuItem("Tools/Setup Default Volume", false, 100)]
        public static void Setup()
        {
            VolumeProfile profile = LoadOrCreateProfile();
            ConfigureProfile(profile);

            Volume volume = EnsureGlobalVolume();
            volume.profile = profile;

            EnableCameraPostProcessing();

            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());

            Debug.Log("[VolumeSetup] DefaultVolumeProfile 已就绪并挂到 Global Volume；" +
                      "记得 Ctrl+S 保存场景。", volume);
        }

        /// <summary>加载已有 profile；没有就新建一个资源文件</summary>
        private static VolumeProfile LoadOrCreateProfile()
        {
            VolumeProfile profile = AssetDatabase.LoadAssetAtPath<VolumeProfile>(ProfilePath);

            if (profile != null)
            {
                return profile;
            }

            profile = ScriptableObject.CreateInstance<VolumeProfile>();
            AssetDatabase.CreateAsset(profile, ProfilePath);
            AssetDatabase.SaveAssets();

            return profile;
        }

        /// <summary>写入 Tonemapping(ACES) / ColorAdjustments(饱和度+10、对比+5) / Bloom(0.2, 1.1)</summary>
        private static void ConfigureProfile(VolumeProfile _profile)
        {
            // 清掉历史遗留的空引用（Add<T> 只在内存里建组件，没 AddObjectToAsset 就会留下 fileID: 0）
            for (int i = _profile.components.Count - 1; i >= 0; i--)
            {
                if (_profile.components[i] == null)
                {
                    _profile.components.RemoveAt(i);
                }
            }

            Tonemapping tonemapping = GetOrAdd<Tonemapping>(_profile);
            tonemapping.active = true;
            tonemapping.mode.overrideState = true;
            tonemapping.mode.value = TonemappingMode.ACES;

            ColorAdjustments colorAdjustments = GetOrAdd<ColorAdjustments>(_profile);
            colorAdjustments.active = true;
            colorAdjustments.saturation.overrideState = true;
            colorAdjustments.saturation.value = 10f;
            colorAdjustments.contrast.overrideState = true;
            colorAdjustments.contrast.value = 5f;

            Bloom bloom = GetOrAdd<Bloom>(_profile);
            bloom.active = true;
            bloom.intensity.overrideState = true;
            bloom.intensity.value = 0.2f;
            bloom.threshold.overrideState = true;
            bloom.threshold.value = 1.1f;

            EditorUtility.SetDirty(_profile);
            AssetDatabase.SaveAssets();
        }

        /// <summary>场景里没有 Global Volume 就建一个</summary>
        private static Volume EnsureGlobalVolume()
        {
            Volume[] volumes = Object.FindObjectsByType<Volume>(FindObjectsSortMode.None);

            foreach (Volume existing in volumes)
            {
                if (existing.isGlobal)
                {
                    return existing;
                }
            }

            GameObject go = new GameObject("Global Volume");
            Volume volume = go.AddComponent<Volume>();
            volume.isGlobal = true;
            volume.priority = 1f;

            return volume;
        }

        /// <summary>主相机打开 Post Processing（URP 里这个开关在 UniversalAdditionalCameraData 上）</summary>
        private static void EnableCameraPostProcessing()
        {
            Camera camera = Camera.main;

            if (camera == null)
            {
                Debug.LogWarning("[VolumeSetup] 场景里没有 MainCamera，跳过相机后处理开关。");
                return;
            }

            camera.GetUniversalAdditionalCameraData().renderPostProcessing = true;
            EditorUtility.SetDirty(camera);
        }

        /// <summary>profile 里已有该组件就复用；新建时必须 AddObjectToAsset，否则组件只存在于内存、保存后变 fileID: 0</summary>
        private static T GetOrAdd<T>(VolumeProfile _profile) where T : VolumeComponent
        {
            if (_profile.TryGet(out T existing))
            {
                return existing;
            }

            T component = _profile.Add<T>(true);
            AssetDatabase.AddObjectToAsset(component, _profile);

            return component;
        }
    }
}
