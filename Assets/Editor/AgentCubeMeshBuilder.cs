using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace Editor
{
    /// <summary>
    /// 生成「代理魔方」网格：一个 mesh 内装着 N×N×N 壳层小方块（内含一个自动贴合的核心块）。
    /// 每个小方块是独立的 6 面 24 顶点，硬边法线；
    /// 方块中心写入 TEXCOORD1（float3），供顶点着色器计算球面波相位与径向外推。
    /// </summary>
    public class AgentCubeMeshBuilder : EditorWindow
    {
        private int m_N = 5;
        private float m_EdgeLength = 1f;
        private float m_Gap = 0.08f;
        private string m_OutputDir = "Assets/Art/Models/Agent";

        [MenuItem("Tools/Navigation/Agent Cube Mesh Builder")]
        public static void Open()
        {
            AgentCubeMeshBuilder window = GetWindow<AgentCubeMeshBuilder>("Agent Cube");
            window.minSize = new Vector2(380f, 300f);
            window.Show();
        }

        private void OnGUI()
        {
            EditorGUILayout.LabelField("壳层魔方", EditorStyles.boldLabel);

            m_N = EditorGUILayout.IntSlider(new GUIContent("阶数 N", "每个方向的方块数，N=1 表示普通方块"), m_N, 2, 12);
            m_EdgeLength = EditorGUILayout.FloatField(new GUIContent("整体边长", "生成后的总尺寸，缩放在 Transform 上另行控制"), m_EdgeLength);
            m_Gap = EditorGUILayout.Slider(new GUIContent("方块间隙", "0.08 = 每块留 8% 缝"), m_Gap, 0f, 0.4f);

            EditorGUILayout.Space();
            m_OutputDir = EditorGUILayout.TextField(new GUIContent("输出目录"), m_OutputDir);

            EditorGUILayout.Space();
            int shellCount = ShellCount(m_N);
            EditorGUILayout.HelpBox(
                $"小方块数：{shellCount}（N³−(N−2)³）\n" +
                $"顶点：{shellCount * 24:N0}　三角形：{shellCount * 12:N0}",
                MessageType.Info);

            EditorGUILayout.Space();
            if (GUILayout.Button("生成 Mesh 资产", GUILayout.Height(32f)))
            {
                Build();
            }
        }

        /// <summary>
        /// 计算魔方最外层表面的小方块的数量
        /// </summary>
        /// <param name="n"></param> 阶数
        /// <returns></returns> 最外层表面的小方块的数量
        private static int ShellCount(int n)
        {
            int inner = Mathf.Max(0, n - 2);
            return n * n * n - inner * inner * inner;
        }

        private void Build()
        {
            int n = Mathf.Max(2, m_N);
            float edge = Mathf.Max(0.01f, m_EdgeLength);
            float half = edge * 0.5f;
            float cell = edge / n;
            // 小方块边长的一半
            float blockHalf = cell * (1f - Mathf.Clamp(m_Gap, 0f, 0.9f)) * 0.5f;

            // 存储所有最外层小方块 和 内核心(除了最外层小方块的体积)
            // 顶点缓冲，顶点法线，小方块中心坐标，三角形索引
            List<Vector3> vertices = new List<Vector3>();
            List<Vector3> normals = new List<Vector3>();
            List<Vector3> centers = new List<Vector3>();
            List<int> triangles = new List<int>();

            for (int x = 0; x < n; x++)
            {
                for (int y = 0; y < n; y++)
                {
                    for (int z = 0; z < n; z++)
                    {
                        bool onSurface = x == 0 || x == n - 1
                                      || y == 0 || y == n - 1
                                      || z == 0 || z == n - 1;
                        if (!onSurface)
                        {
                            continue;
                        }

                        Vector3 center = new Vector3(
                            (x + 0.5f) * cell - half,
                            (y + 0.5f) * cell - half,
                            (z + 0.5f) * cell - half);

                        AddBlock(vertices, normals, centers, triangles, center, blockHalf);
                    }
                }
            }

            // 内核心：刚好贴住最外层小方块的内表面，填住缩缝后透出的背景，再乘 0.98 避开与外壳内表面的共面（否则缝隙里会 z-fighting）
            //      就是除了最外层小方块以外的剩下的部分
            float coreHalf = (half - cell * 0.5f - blockHalf) * 0.98f;
            if (coreHalf > 0.001f)
            {
                // 魔方从外侧看会漏光，把内核心方块也存储进去
                AddBlock(vertices, normals, centers, triangles, Vector3.zero, coreHalf);
            }

            // 创建 Mesh，数据存储到 Mesh 中
            Mesh mesh = new Mesh
            {
                name = $"AgentCube_N{n}"
            };
            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetUVs(1, centers);    // 存储到 TEXCOORD1 中
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();

            // 在指定目录生成对应 Mesh 资产
            Directory.CreateDirectory(m_OutputDir);
            string path = $"{m_OutputDir}/AgentCube_N{n}.asset";
            AssetDatabase.CreateAsset(mesh, path);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Selection.activeObject = mesh;
            EditorGUIUtility.PingObject(mesh);

            Debug.Log($"[AgentCubeMeshBuilder] 已生成 {path}，小方块 {ShellCount(n)} 个，" +
                      $"顶点 {vertices.Count:N0}，三角形 {triangles.Count / 3:N0}");
        }

        /// <summary>
        /// 追加一个以 center 为中心、半边长 h 的硬边立方体。
        /// 每个面 4 个顶点、6 个索引；
        /// 方块中心坐标写入 TEXCOORD1，后续 shader 直接从里面取出数据就可以作为中心点
        /// </summary>
        private static void AddBlock(
            List<Vector3> vertices,
            List<Vector3> normals,
            List<Vector3> centers,
            List<int> triangles,
            Vector3 center,
            float h)
        {
            // 六个面：法线 + 四个角（相对中心，顺时针，从外侧看）
            AddFace(vertices, normals, centers, triangles, center, h, Vector3.forward,
                new Vector3(-1f, -1f, 1f), new Vector3(1f, -1f, 1f), new Vector3(1f, 1f, 1f), new Vector3(-1f, 1f, 1f));
            AddFace(vertices, normals, centers, triangles, center, h, Vector3.back,
                new Vector3(1f, -1f, -1f), new Vector3(-1f, -1f, -1f), new Vector3(-1f, 1f, -1f), new Vector3(1f, 1f, -1f));
            AddFace(vertices, normals, centers, triangles, center, h, Vector3.right,
                new Vector3(1f, -1f, 1f), new Vector3(1f, -1f, -1f), new Vector3(1f, 1f, -1f), new Vector3(1f, 1f, 1f));
            AddFace(vertices, normals, centers, triangles, center, h, Vector3.left,
                new Vector3(-1f, -1f, -1f), new Vector3(-1f, -1f, 1f), new Vector3(-1f, 1f, 1f), new Vector3(-1f, 1f, -1f));
            AddFace(vertices, normals, centers, triangles, center, h, Vector3.up,
                new Vector3(-1f, 1f, 1f), new Vector3(1f, 1f, 1f), new Vector3(1f, 1f, -1f), new Vector3(-1f, 1f, -1f));
            AddFace(vertices, normals, centers, triangles, center, h, Vector3.down,
                new Vector3(-1f, -1f, -1f), new Vector3(1f, -1f, -1f), new Vector3(1f, -1f, 1f), new Vector3(-1f, -1f, 1f));
        }

        /// <summary>
        /// 存储方块一个面的数据
        /// </summary>
        private static void AddFace(
            List<Vector3> vertices,
            List<Vector3> normals,
            List<Vector3> centers,
            List<int> triangles,
            Vector3 center,
            float h,
            Vector3 normal,
            Vector3 a,
            Vector3 b,
            Vector3 c,
            Vector3 d)
        {
            int start = vertices.Count;

            vertices.Add(center + a * h);
            vertices.Add(center + b * h);
            vertices.Add(center + c * h);
            vertices.Add(center + d * h);

            normals.Add(normal);
            normals.Add(normal);
            normals.Add(normal);
            normals.Add(normal);

            // 小方块中心：TEXCOORD1，米制绝对坐标
            centers.Add(center);
            centers.Add(center);
            centers.Add(center);
            centers.Add(center);

            triangles.Add(start + 0);
            triangles.Add(start + 2);
            triangles.Add(start + 1);

            triangles.Add(start + 0);
            triangles.Add(start + 3);
            triangles.Add(start + 2);
        }
    }
}
