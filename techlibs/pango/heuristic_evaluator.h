/*
 * heuristic_evaluator.h
 *
 * HeuristicEvaluator - 启发式评估器
 * 动态计算割的评估指标（深度、面积流、精确面积）
 *
 * 功能：
 * - 不存储派生数据（避免数据不一致）
 * - 通过TimingAnalyzer和MappingContext查询状态
 * - 支持多种评估模式切换
 *
 * 作者：根据8号文档Section 3.5实现
 * 日期：2025-10-04
 */

#ifndef HEURISTIC_EVALUATOR_H
#define HEURISTIC_EVALUATOR_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "timing_analyzer.h"
#include "mapping_context.h"

YOSYS_NAMESPACE_BEGIN

// 前向声明
class TimingAnalyzer;
class MappingContext;

// 评估模式枚举
enum class EvaluationMode {
	DEPTH,          // 深度优先（第一趟映射）
	AREA_FLOW,      // 面积流优先（中间迭代）
	EXACT_AREA      // 精确面积优先（最后一趟）
};

// 单输出割的定义（完整版，包含所有信息）
struct SingleCut {
	pool<SigBit> inputs;          // 输入信号集合
	SigBit output;                // 输出信号

	// ⚠️ 重要：不存储派生数据（深度、面积流等）
	// 所有派生数据由 HeuristicEvaluator 动态计算

	SingleCut() = default;

	SingleCut(const pool<SigBit> &inputs_, SigBit output_)
		: inputs(inputs_), output(output_) {}

	// ⚠️⚠️⚠️ 关键说明：operator< 的用途（修正1相关）
	//
	// 此 operator< 用于容器（如 set）的**元素唯一性**判断和排序，
	// 不用于优先级队列的排序！
	//
	// 在 GlobalMerger::runGlobalMapping() 中使用 multiset<SingleCut> 时，
	// 必须提供一个**动态比较器**（基于 HeuristicEvaluator），
	// 而不是依赖此静态的 operator<。
	//
	// ⚠️ BUG修复：必须同时比较 output 和 inputs 才能唯一确定一个割
	// 只比较output会导致同一输出的不同割被误认为相同，从而丢失候选割
	//
	// 详见：8号文档Section 3.7.2 的 CutComparator 定义。
	bool operator<(const SingleCut &other) const {
		// 必须同时比较 output 和 inputs 才能唯一确定一个割
		if (output != other.output) {
			return output < other.output;
		}
		// pool<SigBit> 没有 operator<，所以我们先比较大小
		if (inputs.size() != other.inputs.size()) {
			return inputs.size() < other.inputs.size();
		}
		// 如果大小相同，将pool转换为有序vector进行字典序比较
		// 这确保了确定性和唯一性
		vector<SigBit> inputs_vec(inputs.begin(), inputs.end());
		vector<SigBit> other_inputs_vec(other.inputs.begin(), other.inputs.end());
		std::sort(inputs_vec.begin(), inputs_vec.end());
		std::sort(other_inputs_vec.begin(), other_inputs_vec.end());
		return inputs_vec < other_inputs_vec;
	}
};

class HeuristicEvaluator {
public:
	// ===== 构造和初始化 =====

	HeuristicEvaluator(
		Module *module,
		MappingContext *context,
		TimingAnalyzer *timing
	);

	// ===== 模式设置 =====

	void setMode(EvaluationMode mode) {
		current_mode = mode;
	}

	EvaluationMode getMode() const {
		return current_mode;
	}

	// ===== 单输出割评估（动态计算）=====

	/**
	 * 计算割的深度
	 * @param cut 单输出割
	 * @return 深度值（通过 TimingAnalyzer 查询）
	 */
	int computeDepth(const SingleCut &cut);

	/**
	 * 计算面积流
	 * @param cut 单输出割
	 * @return 面积流值
	 *
	 * @formula area_flow = (area + 1) / fanout_refs
	 *          area: 输入的总面积
	 *          +1: 当前LUT
	 *          fanout_refs: 输出的扇出引用数
	 */
	double computeAreaFlow(const SingleCut &cut);

	/**
	 * 计算精确面积
	 * @param cut 单输出割
	 * @return 精确面积（通过 MappingContext 的 memoization 计算）
	 */
	int computeArea(const SingleCut &cut);

	// ===== 割比较（根据当前模式）=====

	/**
	 * 比较两个单输出割
	 * @return true 如果 a 优于 b
	 */
	bool compareSingleCuts(const SingleCut &a, const SingleCut &b);

private:
	Module *module;
	MappingContext *context;
	TimingAnalyzer *timing;

	EvaluationMode current_mode;

	// 辅助比较函数
	bool compareByDepth(const SingleCut &a, const SingleCut &b);
	bool compareByAreaFlow(const SingleCut &a, const SingleCut &b);
	bool compareByExactArea(const SingleCut &a, const SingleCut &b);
};

YOSYS_NAMESPACE_END

#endif // HEURISTIC_EVALUATOR_H
