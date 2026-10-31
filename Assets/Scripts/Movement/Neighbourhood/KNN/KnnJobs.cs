using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnsafeUtilityEx = KNN.Internal.UnsafeUtilityEx;

/// <summary>
/// KNN查询的Unity Job System集成
///
/// 本文件提供多种Job类型，将KnnContainer的查询操作封装为Unity的IJob / IJobParallelForBatch，
/// 使其可以在Job线程中并行执行，并通过 [BurstCompile] 标签编译为高性能的原生代码（LLVM IR）
///
/// 可用Job类型：
/// - QueryKNearestJob：单点K近邻查询（单线程Job）
/// - QueryKNearestBatchJob：批量K近邻查询（并行Job，每线程处理一批点）
/// - QueryRangeJob：单点范围查询（单线程Job）
/// - QueryRangeBatchJob：批量范围查询（并行Job，配合RangeQueryResult）
/// - KnnRebuildJob：KD树重建（单线程Job，用于在Job线程中异步构建树）
/// </summary>
namespace KNN.Jobs
{

	// ─────────────────────────────────────────────────────────────────────────────
	// K近邻查询（K-Nearest Neighbors）
	// ─────────────────────────────────────────────────────────────────────────────
	/// <summary>
	/// 单点K近邻查询Job
	///
	/// 适用场景：需要对单个查询点执行KNN查询时
	/// 使用方式：
	/// <code>
	/// var result = new NativeArray&lt;int&gt;(k, Allocator.TempJob);
	/// var job = new QueryKNearestJob(container, queryPos, result);
	/// job.Schedule().Complete();
	/// // result中存储了最近的k个点的原始索引
	/// result.Dispose();
	/// </code>
	/// </summary>
	[BurstCompile(CompileSynchronously = true)]
	public struct QueryKNearestJob : IJob
	{
		/// <summary>
		/// KD树容器（只读访问，不修改树结构）
		/// </summary>
		[ReadOnly] KnnContainer m_container;

		/// <summary>
		/// 写入最近K个点原始索引的输出缓冲区
		/// 缓冲区长度即为K值
		/// </summary>
		[WriteOnly] NativeSlice<int> m_result;

		/// <summary>
		/// 查询点的3D位置
		/// </summary>
		float3 m_queryPosition;

		/// <summary>
		/// 构造函数，初始化Job参数
		/// </summary>
		/// <param name="container">已构建好的KD树容器</param>
		/// <param name="queryPosition">查询点位置</param>
		/// <param name="result">输出缓冲区，长度 = K（要查找的近邻数量）</param>
		public QueryKNearestJob(KnnContainer container, float3 queryPosition, NativeSlice<int> result)
		{
			m_result = result;
			m_queryPosition = queryPosition;
			m_container = container;
		}

		/// <summary>
		/// Job执行入口，调用KnnContainer.QueryKNearest完成查询
		/// </summary>
		void IJob.Execute()
		{
			m_container.QueryKNearest(m_queryPosition, m_result);
		}
	}

	/// <summary>
	/// 批量K近邻查询Job（并行版本）
	///
	/// 对多个查询点并行执行KNN查询，适合每帧对大量Agent进行近邻查询的场景
	///
	/// 结果布局：results是一个扁平数组，
	/// 第i个查询点的K个近邻索引存储在results[i*k .. i*k+k]
	///
	/// 使用方式：
	/// <code>
	/// int k = 5;
	/// var queryPositions = new NativeArray&lt;float3&gt;(queryCount, Allocator.TempJob);
	/// var results = new NativeArray&lt;int&gt;(queryCount * k, Allocator.TempJob);
	/// var job = new QueryKNearestBatchJob(container, queryPositions, results);
	/// job.ScheduleBatch(queryCount, batchSize).Complete();
	/// // results[i*k + j] = 第i个查询点的第j个近邻的原始索引
	/// </code>
	/// </summary>
	[BurstCompile(CompileSynchronously = true)]
	public struct QueryKNearestBatchJob : IJobParallelForBatch
	{
		/// <summary>
		/// KD树容器（只读，可被多线程安全并发访问）
		/// </summary>
		[ReadOnly] KnnContainer m_container;

		/// <summary>
		/// 所有查询点的位置数组（只读）
		/// </summary>
		[ReadOnly] NativeSlice<float3> m_queryPositions;

		// Unity really doesn't like it when we write to the same underlying array
		// Even if slices don't overlap... So we're just being dangerous here
		/// <summary>
		/// 所有查询结果的输出数组（扁平化存储）
		///
		/// 禁用了并行写入限制（NativeDisableParallelForRestriction）和安全限制：
		/// 虽然不同线程写入的是不同的切片范围（不重叠），但Unity的安全系统无法
		/// 静态验证这一点，因此需要手动禁用检查
		/// 调用方需确保各批次的索引范围不重叠
		/// </summary>
		[NativeDisableParallelForRestriction, NativeDisableContainerSafetyRestriction]
		NativeSlice<int> m_results;

		/// <summary>
		/// K值（每个查询点要找的近邻数量），由results.Length / queryPositions.Length计算得出
		/// </summary>
		int m_k;

		/// <summary>
		/// 构造函数，初始化批量查询Job
		/// </summary>
		/// <param name="container">已构建好的KD树容器</param>
		/// <param name="queryPositions">查询点位置数组</param>
		/// <param name="results">
		/// 输出数组，长度必须是queryPositions.Length的整数倍，
		/// 倍数即为K值（每个查询点的近邻数）
		/// </param>
		public QueryKNearestBatchJob(KnnContainer container, NativeArray<float3> queryPositions, NativeSlice<int> results)
		{
			m_container = container;
			m_queryPositions = queryPositions;
			m_results = results;

		#if ENABLE_UNITY_COLLECTIONS_CHECKS
			// 在Editor模式下验证results数组长度是否为queryPositions的整数倍
			if (queryPositions.Length == 0 || results.Length % queryPositions.Length != 0)
			{
				Debug.LogError("Make sure your results array is a multiple in length of your querypositions array!");
			}
		#endif

			m_k = results.Length / queryPositions.Length;
		}

		/// <summary>
		/// 并行批次执行入口每个线程处理 [startIndex, startIndex + count) 范围内的查询点
		/// </summary>
		/// <param name="startIndex">本批次的起始查询点索引</param>
		/// <param name="count">本批次要处理的查询点数量</param>
		public void Execute(int startIndex, int count)
		{
			// Write results to proper slice!
			// 为每个查询点取出对应的结果切片，并执行KNN查询
			for (int index = startIndex; index < startIndex + count; ++index)
			{
				// 第index个查询点的结果存储在results[index*k .. index*k+k)
				NativeSlice<int> resultsSlice = m_results.Slice(index * m_k, m_k);
				m_container.QueryKNearest(m_queryPositions[index], resultsSlice);
			}
		}
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// 范围查询（Range Query）
	// ─────────────────────────────────────────────────────────────────────────────
	/// <summary>
	/// 单点范围查询Job
	///
	/// 查找所有与查询点距离不超过range的点，将其原始索引写入result列表
	/// 结果数量不固定（取决于范围内有多少点）
	/// </summary>
	[BurstCompile(CompileSynchronously = true)]
	public struct QueryRangeJob : IJob
	{
		/// <summary>
		/// KD树容器（只读）
		/// </summary>
		[ReadOnly] KnnContainer m_container;

		/// <summary>
		/// 输出列表，存储范围内所有点的原始索引
		/// 使用NativeList因为结果数量未知（会动态增长）
		/// </summary>
		[WriteOnly] NativeList<int> m_result;

		/// <summary>
		/// 查询半径
		/// </summary>
		float m_range;

		/// <summary>
		/// 查询点的3D位置
		/// </summary>
		float3 m_queryPosition;

		/// <summary>
		/// 构造函数，初始化Job参数
		/// </summary>
		/// <param name="container">已构建好的KD树容器</param>
		/// <param name="queryPosition">查询点位置</param>
		/// <param name="range">查询半径</param>
		/// <param name="result">输出列表</param>
		public QueryRangeJob(KnnContainer container, float3 queryPosition, float range, NativeList<int> result)
		{
			m_result = result;
			m_range = range;
			m_queryPosition = queryPosition;
			m_container = container;
		}

		/// <summary>
		/// Job执行入口，调用KnnContainer.QueryRange完成查询
		/// </summary>
		void IJob.Execute()
		{
			m_container.QueryRange(m_queryPosition, m_range, m_result);
		}
	}

	/// <summary>
	/// 批量范围查询的单个结果容器（unsafe实现）
	///
	/// 用于在并行Job中存储每个查询点的范围查询结果（索引列表）
	/// 内部使用unsafe指针存储索引，以支持Burst编译和零GC
	///
	/// 使用流程：
	/// 1. 创建：new RangeQueryResult(maxCount, allocator)
	/// 2. 在Job中通过SetResults写入数据
	/// 3. 通过this[i] 读取结果
	/// 4. 使用完毕后调用Dispose() 释放内存
	/// </summary>
	public unsafe struct RangeQueryResult
	{
		/// <summary>
		/// 实际查询到的结果数量（<= m_capacity）
		/// </summary>
		public int Length;

		/// <summary>
		/// 存储点原始索引的非托管数组指针
		/// </summary>
		int* m_indices;

		/// <summary>
		/// 分配的最大容量（若实际结果数量超过此值，超出部分会被截断）
		/// </summary>
		int m_capacity;

		/// <summary>
		/// 内存分配器类型
		/// </summary>
		Allocator m_allocator;

		/// <summary>
		/// 索引访问器，返回第index个结果的点原始索引
		/// 越界时抛出IndexOutOfRangeException
		/// </summary>
		/// <param name="index">结果的位置索引（0 <= index < Length）</param>
		/// <returns>点在KnnContainer.Points中的原始索引</returns>
		public int this[int index]
		{
			get
			{
				if (index >= m_capacity)
				{
					throw new IndexOutOfRangeException();
				}

				return UnsafeUtility.ReadArrayElement<int>(m_indices, index);
			}
		}

		/// <summary>
		/// 构造函数，分配结果存储内存
		/// </summary>
		/// <param name="maxCount">最大可存储的结果数量（超出部分会被截断）</param>
		/// <param name="allocator">内存分配器</param>
		public RangeQueryResult(int maxCount, Allocator allocator)
		{
			m_capacity = maxCount;
			m_indices = UnsafeUtilityEx.AllocArray<int>(m_capacity, allocator);
			Length = 0;
			m_allocator = allocator;
		}

		/// <summary>
		/// 将NativeList中的查询结果复制到此结构中
		///
		/// 若result.Length > m_capacity，只复制前m_capacity个结果（截断）
		/// </summary>
		/// <param name="result">由KnnContainer.QueryRange填充的结果列表</param>
		public void SetResults(NativeList<int> result)
		{
			UnsafeUtility.MemCpy(m_indices, result.GetUnsafePtr(), Mathf.Min(m_capacity, result.Length) * sizeof(int));
			Length = Mathf.Min(m_capacity, result.Length);
		}

		/// <summary>
		/// 释放非托管内存
		/// </summary>
		public void Dispose()
		{
			UnsafeUtility.Free(m_indices, m_allocator);
		}
	}

	/// <summary>
	/// 批量范围查询Job（并行版本）
	///
	/// 对多个查询点并行执行范围查询，每个查询点的结果独立存储在Results数组中
	///
	/// 使用方式：
	/// <code>
	/// var queryPositions = new NativeArray&lt;float3&gt;(n, Allocator.TempJob);
	/// var results = new NativeArray&lt;RangeQueryResult&gt;(n, Allocator.TempJob);
	/// // 为每个结果预分配内存
	/// for (int i = 0; i &lt; n; i++)
	///     results[i] = new RangeQueryResult(maxResultPerQuery, Allocator.TempJob);
	///
	/// var job = new QueryRangeBatchJob(container, queryPositions, range, results);
	/// job.ScheduleBatch(n, batchSize).Complete();
	///
	/// // 读取结果
	/// for (int i = 0; i &lt; n; i++) {
	///     var r = results[i];
	///     for (int j = 0; j &lt; r.Length; j++) Debug.Log(r[j]);
	///     r.Dispose();
	/// }
	/// results.Dispose();
	/// </code>
	/// </summary>
	[BurstCompile(CompileSynchronously = true)]
	public struct QueryRangeBatchJob : IJobParallelForBatch
	{
		/// <summary>
		/// KD树容器（只读）
		/// </summary>
		[ReadOnly] KnnContainer m_container;

		/// <summary>
		/// 所有查询点的位置数组（只读）
		/// </summary>
		[ReadOnly] NativeSlice<float3> m_queryPositions;

		/// <summary>
		/// 查询半径（所有查询点使用相同的半径）
		/// </summary>
		float m_range;

		/// <summary>
		/// 每个查询点的范围查询结果（公开，Job执行后可读取）
		/// </summary>
		public NativeArray<RangeQueryResult> Results;

		/// <summary>
		/// 构造函数，初始化批量范围查询Job
		/// </summary>
		/// <param name="container">已构建好的KD树容器</param>
		/// <param name="queryPositions">查询点位置数组</param>
		/// <param name="range">查询半径</param>
		/// <param name="results">
		/// 预分配好的RangeQueryResult数组（每个元素需提前new并指定最大容量）
		/// </param>
		public QueryRangeBatchJob(KnnContainer container, NativeArray<float3> queryPositions, float range, NativeArray<RangeQueryResult> results)
		{
			m_container = container;
			m_queryPositions = queryPositions;
			m_range = range;
			Results = results;
		}

		/// <summary>
		/// 并行批次执行入口处理 [startIndex, startIndex + count) 范围内的查询点
		/// 每个查询点使用Allocator.Temp的临时列表收集结果，再复制到对应RangeQueryResult
		/// </summary>
		/// <param name="startIndex">本批次起始查询点索引</param>
		/// <param name="count">本批次查询点数量</param>
		public void Execute(int startIndex, int count)
		{
			// Write results to proper slice!
			for (int index = startIndex; index < startIndex + count; ++index)
			{
				// 使用临时列表收集范围查询结果（Allocator.Temp在帧内有效，Job线程中可用）
				var tempList = new NativeList<int>(Allocator.Temp);
				m_container.QueryRange(m_queryPositions[index], m_range, tempList);

				// 将结果复制到预分配的RangeQueryResult
				var result = Results[index];
				result.SetResults(tempList);

				// 注意：RangeQueryResult是struct，修改后需写回数组
				Results[index] = result;
			}
		}
	}

	// ─────────────────────────────────────────────────────────────────────────────
	// KD树重建（Rebuild）
	// ─────────────────────────────────────────────────────────────────────────────
	/// <summary>
	/// KD树重建Job
	///
	/// 将KnnContainer.Rebuild() 封装为IJob，使KD树的构建可以在Job线程中异步执行，
	/// 避免在主线程阻塞
	///
	/// 通常在以下场景使用：
	/// - 点集发生变化（如障碍物移动、动态点云更新）后，需要重建KD树索引
	/// - 初始化时若不想立即同步构建（KnnContainer构造函数buildNow=false），
	///   可通过此Job异步构建
	///
	/// 使用方式：
	/// <code>
	/// var container = new KnnContainer(points, false, Allocator.Persistent);
	/// var rebuildJob = new KnnRebuildJob(container);
	/// JobHandle handle = rebuildJob.Schedule();
	/// // ... 可在此期间做其他事 
	/// handle.Complete(); // 等待重建完成
	/// </code>
	/// </summary>
	[BurstCompile(CompileSynchronously = true)]
	public struct KnnRebuildJob : IJob
	{
		/// <summary>
		/// 待重建的KD树容器（需要写入访问，因为Rebuild会修改内部节点数据）
		/// </summary>
		KnnContainer m_container;

		/// <summary>
		/// 构造函数
		/// </summary>
		/// <param name="container">需要重建的KNN容器</param>
		public KnnRebuildJob(KnnContainer container)
		{
			m_container = container;
		}

		/// <summary>
		/// Job执行入口，调用KnnContainer.Rebuild() 重建KD树
		/// </summary>
		void IJob.Execute()
		{
			m_container.Rebuild();
		}
	}
}
