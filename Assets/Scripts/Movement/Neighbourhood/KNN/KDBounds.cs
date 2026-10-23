//MIT License
//
//Copyright(c) 2018 Vili Volčini / viliwonka
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.
//
// Modifed 2019 Arthur Brussee

using Unity.Mathematics;

namespace KNN.Internal
{
	/// <summary>
	/// KD树节点的轴对齐包围盒（AABB）。
	///
	/// 每个 KD 树节点都关联一个包围盒，用来描述该节点所覆盖的空间区域。
	/// 包围盒由最小点（Min）和最大点（Max）定义，形成一个轴对齐的长方体。
	///
	/// 在 KNN 查询中，通过计算查询点到包围盒的最近距离，
	/// 可以快速剪枝那些不可能包含最近邻的节点，从而大幅提升查询效率。
	/// </summary>
	public struct KdNodeBounds
	{
		/// <summary>
		/// 包围盒的最小角点（各轴的最小坐标值）。
		/// 例如：Min = (xMin, yMin, zMin)
		/// </summary>
		public float3 Min;

		/// <summary>
		/// 包围盒的最大角点（各轴的最大坐标值）。
		/// 例如：Max = (xMax, yMax, zMax)
		/// </summary>
		public float3 Max;

		/// <summary>
		/// 包围盒在各轴上的尺寸（长宽高）。
		/// Size = Max - Min，每个分量表示对应轴的跨度。
		/// 该属性用于确定 KD 树分割时应选择哪个轴（选最长轴分割可使树更平衡）。
		/// </summary>
		public float3 Size => Max - Min;

		/// <summary>
		/// 计算给定点到包围盒的最近点。
		///
		/// 原理：对每个坐标轴，将点的坐标 clamp 到 [Min, Max] 区间内。
		/// - 若点在包围盒内部，则返回点本身（clamp不改变值）。
		/// - 若点在包围盒外部，则返回包围盒表面上距离该点最近的点。
		///		- 即若包围盒内某个点和给定点的 x、y、z 轴的差值平方和最小，则返回那个点
		///
		/// 该方法在 KNN 查询中用于估算查询点到某个节点包围盒的最小可能距离，
		/// 以决定是否需要进入该节点搜索。
		/// </summary>
		/// <param name="point">要测试的查询点</param>
		/// <returns>包围盒上距 point 最近的点（若 point 在盒内则返回 point 本身）</returns>
		public float3 ClosestPoint(float3 point)
		{
			return math.clamp(point, Min, Max);
		}
	}
}
