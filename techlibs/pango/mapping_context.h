/*
 * mapping_context.h
 *
 * MappingContext - 映射上下文/状态管理器
 * 管理映射过程中的全局状态（引用计数、使用标记）
 * ⭐ 性能关键：包含memoization优化的精确面积计算
 *
 * 功能：
 * - 管理引用计数和使用标记
 * - 精确面积计算（带memoization缓存，命中率>95%）
 * - 支持迭代间状态恢复
 *
 * 作者：根据8号文档Section 3.3实现
 * 日期：2025-10-04
 */

#ifndef MAPPING_CONTEXT_H
#define MAPPING_CONTEXT_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "graph_utils.h"

YOSYS_NAMESPACE_BEGIN

// 前向声明
class GraphUtils;
struct SingleCut;  // 前向声明，完整定义在heuristic_evaluator.h

class MappingContext {
public:
	// ===== 构造和初始化 =====

	MappingContext(Module *module, SigMap &sigmap, GraphUtils *graph);

	// ===== 迭代管理 =====

	/**
	 * 开始新的迭代
	 * @note 使精确面积缓存失效，准备新一轮计算
	 */
	void startNewIteration() {
		current_iteration++;
		// 缓存自动失效（通过 iteration 编号检查）
	}

	// ===== 状态查询 =====

	/**
	 * 获取信号的扇出引用计数
	 * @param signal 信号
	 * @return 引用计数（被多少个映射的LUT使用）
	 */
	int getFanoutRefs(SigBit signal) const;

	/**
	 * 检查信号是否被使用
	 * @param signal 信号
	 * @return true 如果信号在当前映射中被使用
	 */
	bool isUsed(SigBit signal) const;

	/**
	 * 获取信号的当前映射
	 * @param signal 信号
	 * @return 映射的割，如果未映射则返回nullptr
	 */
	const SingleCut* getCurrentMapping(SigBit signal) const;

	// ===== 精确面积计算（带 memoization，文档4核心优化）⭐⭐⭐ =====

	/**
	 * 计算信号的精确面积
	 * @param signal 信号
	 * @return 实现此信号所需的LUT数量（考虑共享）
	 *
	 * @note 关键优化：使用 memoization 缓存
	 * @note 缓存在单次迭代内有效
	 * @note 预期缓存命中率 > 95%（文档4要求）
	 */
	int getExactArea(SigBit signal);

	// ===== 状态更新 =====

	/**
	 * 恢复引用计数（每次迭代开始时调用）
	 * @param mapping 当前的映射结果
	 *
	 * @note 从输出开始BFS遍历，重新计算所有引用计数
	 * @note O(N) 复杂度，可接受（文档4确认）
	 */
	void recoverReferences(const dict<SigBit, SingleCut> &mapping);

	/**
	 * 取消引用（将信号从映射中移除时调用）
	 * @param signal 信号
	 * @param area_delta 输出参数：面积变化量
	 */
	void dereference(SigBit signal, int &area_delta);

	/**
	 * 添加引用（将信号加入映射时调用）
	 * @param signal 信号
	 */
	void reference(SigBit signal);

	// ===== 性能监控（文档4要求）=====

	struct PerfStats {
		int recover_calls = 0;
		int exact_area_calls = 0;
		int exact_area_cache_hits = 0;

		double cache_hit_rate() const {
			return exact_area_calls > 0 ?
			       (double)exact_area_cache_hits / exact_area_calls : 0.0;
		}
	};

	PerfStats getPerformanceStats() const {
		return perf_stats;
	}

	void resetPerformanceStats() {
		perf_stats = PerfStats();
	}

private:
	Module *module;
	SigMap &sigmap;
	GraphUtils *graph;

	// 状态数据
	dict<SigBit, int> fanout_refs;           // 扇出引用计数
	dict<SigBit, bool> used;                 // 使用标记
	dict<SigBit, SingleCut> current_mapping; // 当前映射

	// Memoization 缓存系统（文档4核心）
	dict<SigBit, int> exact_area_cache;      // 精确面积缓存
	dict<SigBit, int> cache_iteration;       // 缓存对应的迭代编号
	int current_iteration;

	// 性能统计
	mutable PerfStats perf_stats;

	// 辅助方法
	int getExactAreaRecursive(SigBit signal, pool<SigBit> &visited);
};

YOSYS_NAMESPACE_END

#endif // MAPPING_CONTEXT_H
