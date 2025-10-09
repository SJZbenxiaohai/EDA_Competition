/*
 * cut_manager.h
 *
 * CutManager - 割管理器
 * 负责割的枚举、合并和筛选
 *
 * 功能：
 * - 割枚举（K=6，支持双输出合并）
 * - 优先割选择（保留P=20个最优割）
 * - Cut Dominance 剪枝（v1.4优化）
 *
 * 作者：根据8号文档Section 3.6实现
 * 日期：2025-10-04
 */

#ifndef CUT_MANAGER_H
#define CUT_MANAGER_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "graph_utils.h"
#include "heuristic_evaluator.h"
#include "timing_analyzer.h"

YOSYS_NAMESPACE_BEGIN

// 前向声明
class GraphUtils;
class HeuristicEvaluator;
class TimingAnalyzer;

// 割的基础定义
using Cut = pool<SigBit>;

class CutManager {
public:
	// ===== 构造和初始化 =====

	CutManager(
		Module *module,
		HeuristicEvaluator *evaluator,
		TimingAnalyzer *timing,
		GraphUtils *graph
	);

	// ===== 割枚举 =====

	/**
	 * 计算所有信号的优先割
	 * @param max_cut_size 最大割大小（K=6）
	 * @param max_cuts 每个信号保留的最大割数量（P=20）
	 */
	void computePriorityCuts(int max_cut_size = 6, int max_cuts = 20);

	/**
	 * 获取信号的最优割
	 * @param signal 信号
	 * @return 最优的单输出割
	 */
	SingleCut getBestCut(SigBit signal) const;

	/**
	 * 获取信号的所有优先割
	 * @param signal 信号
	 * @return 优先割向量（按优先级排序）
	 */
	const vector<SingleCut>& getPriorityCuts(SigBit signal) const;

	/**
	 * 获取信号的指定大小的所有割
	 * @param signal 信号
	 * @param size 割的大小
	 * @return 该大小的所有割
	 */
	const vector<Cut>& getCutsBySize(SigBit signal, int size) const;

	// ===== 统计信息 =====

	struct Statistics {
		int total_cuts = 0;
		int total_signals = 0;
		double avg_cuts_per_signal = 0.0;
	};

	Statistics getStatistics() const;

private:
	Module *module;
	HeuristicEvaluator *evaluator;
	TimingAnalyzer *timing;
	GraphUtils *graph;

	// 割存储
	dict<SigBit, vector<SingleCut>> priority_cuts;  // 每个信号的优先割（最多P个）
	dict<SigBit, dict<int, vector<Cut>>> cuts_by_size;  // 按大小分类的割

	// 参数
	int max_cut_size;
	int max_cuts;

	// 内部方法
	void enumerateCutsForGate(Cell *gate);
	void mergeCuts(const vector<Cut> &cuts_a, const vector<Cut> &cuts_b,
	               vector<Cut> &result);
	void selectPriorityCuts(SigBit signal);

	// v1.4 新增（暂不实现）
	// bool isDominated(const Cut &a, const Cut &b);
};

YOSYS_NAMESPACE_END

#endif // CUT_MANAGER_H
