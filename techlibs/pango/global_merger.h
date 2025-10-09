/*
 * global_merger.h
 *
 * GlobalMerger - 全局映射器
 * MVP版本：仅实现单输出映射
 * v1.1版本：添加两阶段过滤双输出合并
 *
 * 功能：
 * - 单输出映射（MVP）
 * - 两阶段过滤双输出合并（v1.1）
 * - 严谨的真值表约束验证（v1.1）
 * - I5选择循环
 *
 * 作者：根据8号文档Section 3.7实现
 * 日期：2025-10-04
 */

#ifndef GLOBAL_MERGER_H
#define GLOBAL_MERGER_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "cut_manager.h"
#include "heuristic_evaluator.h"
#include "timing_analyzer.h"
#include "truth_table_computer.h"
#include "graph_utils.h"

YOSYS_NAMESPACE_BEGIN

// 前向声明
class CutManager;
class HeuristicEvaluator;
class TimingAnalyzer;
class TruthTableComputer;
class GraphUtils;

// 双输出割定义
struct DoubleCut {
	pool<SigBit> inputs;          // 合并后的输入集合（≤6个）
	SigBit output1;               // Z 输出
	SigBit output2;               // Z5 输出
	SigBit selected_i5;           // 选中的I5输入（文档7要求）⭐

	DoubleCut() = default;

	bool valid() const {
		return output1 != SigBit() && output2 != SigBit();
	}
};

// 映射结果定义
struct MappingResult {
	// 单输出映射
	dict<SigBit, SingleCut> single_mappings;

	// 双输出映射（v1.1新增）
	dict<pair<SigBit, SigBit>, DoubleCut> double_mappings;

	// 统计信息
	struct Statistics {
		int num_single_luts = 0;
		int num_double_luts = 0;
		int total_luts = 0;
		int max_depth = 0;
		double avg_area_flow = 0.0;

		// v1.1 性能统计
		int stage1_candidates = 0;     // 阶段1候选数
		int stage2_candidates = 0;     // 阶段2候选数
		double filter_efficiency = 0.0; // 筛选效率
	};

	Statistics stats;
};

class GlobalMerger {
public:
	// ===== 构造和初始化 =====

	GlobalMerger(
		Module *module,
		CutManager *cut_mgr,
		HeuristicEvaluator *evaluator,
		TimingAnalyzer *timing,
		TruthTableComputer *truth_table,  // v1.1 新增依赖
		GraphUtils *graph  // 添加graph依赖
	);

	// ===== 主映射接口 =====

	/**
	 * 运行全局映射
	 * @note MVP: 仅单输出映射
	 * @note v1.1: 单输出 + 双输出合并
	 */
	void runGlobalMapping();

	// ===== 结果查询 =====

	const dict<SigBit, SingleCut>& getSingleMappings() const {
		return single_mappings;
	}

	const dict<pair<SigBit, SigBit>, DoubleCut>& getDoubleMappings() const {
		return double_mappings;
	}

	MappingResult getResult() const;

	// ===== v1.1 启发式权重配置（文档7 Section 2.3.2）⭐ =====

	struct HeuristicConfig {
		float input_count_weight = 1.0f;      // 输入数量权重
		float depth_penalty_weight = 10.0f;   // 深度增加惩罚
		float area_flow_weight = 5.0f;        // 面积流权重
		float input_sharing_weight = -2.0f;   // 输入共享奖励（负值）
	};

	void setHeuristicConfig(const HeuristicConfig &config) {
		heuristic_config = config;
	}

	// ===== v1.1 双输出控制 =====

	/**
	 * 启用/禁用双输出合并功能
	 * @param enable true启用，false禁用
	 * @note v1.1版本使用，MVP版本默认false
	 */
	void setEnableDoubleOutput(bool enable) {
		enable_double_output = enable;
	}

	bool getEnableDoubleOutput() const {
		return enable_double_output;
	}

private:
	Module *module;
	SigMap sigmap;  // 添加sigmap成员
	CutManager *cut_mgr;
	HeuristicEvaluator *evaluator;
	TimingAnalyzer *timing;
	TruthTableComputer *truth_table;  // v1.1 新增
	GraphUtils *graph;  // 添加graph成员

	// 映射结果
	dict<SigBit, SingleCut> single_mappings;
	dict<pair<SigBit, SigBit>, DoubleCut> double_mappings;

	// v1.1 配置
	HeuristicConfig heuristic_config;
	bool enable_double_output;  // v1.1 启用

	// ===== 修正1：动态比较器定义（强制修正）⭐⭐⭐ =====

	/**
	 * CutComparator：用于 multiset 的动态比较器
	 *
	 * 问题：SingleCut 的 operator< 是静态的字典序比较，
	 *      无法反映当前评估模式下的优先级。
	 *
	 * 解决：定义一个比较器结构体，持有 HeuristicEvaluator 的引用，
	 *      调用其 compareSingleCuts() 进行动态比较。
	 *
	 * 用法：std::multiset<SingleCut, CutComparator> Q(CutComparator(evaluator));
	 */
	struct CutComparator {
		HeuristicEvaluator* evaluator;

		// 构造函数，传入评估器
		CutComparator(HeuristicEvaluator* eval) : evaluator(eval) {}

		// 重载()运算符，定义比较逻辑
		bool operator()(const SingleCut& a, const SingleCut& b) const {
			// 调用评估器进行动态比较
			return evaluator->compareSingleCuts(a, b);
		}
	};

	// ===== v1.1 两阶段过滤双输出合并（文档7 Section 2.2）⭐⭐⭐ =====

	/**
	 * CandidatePair: 双输出候选对（用于阶段1筛选）
	 *
	 * @note 文档8 Section 3.7.3要求的结构
	 * @note 修正2：包含精确的输入映射参数
	 */
	struct CandidatePair {
		SigBit z5_output;                     // Z5的输出信号
		Cut z5_inputs;                        // Z5的输入集合
		SigBit selected_i5;                   // 选中的I5输入
		Cut z_remaining_inputs;               // Z去除I5后的输入
		float structural_score;               // 结构化启发式分数

		// 修正2新增：精确映射参数 ⭐⭐⭐
		dict<int, int> z5_to_z_input_map;     // Z5到Z的输入映射
		vector<int> z_dont_care_indices;      // Z的don't care输入索引
	};

	/**
	 * 检查输入兼容性（阶段1使用，修正2）⭐⭐⭐
	 *
	 * @param z_remaining Z去除I5后的输入
	 * @param z5_inputs Z5的输入
	 * @param z5_to_z_input_map [输出] Z5的第i个输入 -> Z的第j个输入的映射
	 * @param z_dont_care_indices [输出] Z中未被Z5使用的输入索引列表
	 * @return true 如果兼容（Z5是Z的子集）
	 *
	 * @note 修正2：必须返回精确的输入映射关系
	 * @note 用于 verifyTruthTableConstraint 中正确确定 don't care 输入
	 */
	bool checkInputCompatibility(
		const Cut &z_remaining,
		const Cut &z5_inputs,
		dict<int, int> &z5_to_z_input_map,      // 输出参数 ⭐
		vector<int> &z_dont_care_indices        // 输出参数 ⭐
	);

	/**
	 * 计算结构化启发式分数（阶段1使用，文档7 Section 2.3.2）
	 * @return 分数（越小越好）
	 *
	 * @note 考虑因素：
	 *   1. 合并后的输入数量
	 *   2. 深度惩罚
	 *   3. 面积流估算
	 *   4. 输入共享度
	 */
	float computeStructuralScore(
		SigBit z_output,
		SigBit z5_output,
		const Cut &merged_inputs,
		SigBit selected_i5
	);

	/**
	 * 验证真值表约束（阶段2使用，修正2）⭐⭐⭐
	 *
	 * @param z_init Z的真值表
	 * @param z5_init Z5的真值表
	 * @param z_num_inputs Z的输入数量
	 * @param z5_num_inputs Z5的输入数量
	 * @param z5_to_z_input_map Z5到Z的输入映射（来自checkInputCompatibility）
	 * @param z_dont_care_indices Z的don't care输入索引（来自checkInputCompatibility）
	 * @return true 如果可以合并
	 *
	 * @note 严谨版本，使用isIndependentOfInputs和projectTruthTable
	 * @note 绝不使用文档7 Section 2.3.3的简化版本
	 */
	bool verifyTruthTableConstraint(
		const Const &z_init,
		const Const &z5_init,
		int z_num_inputs,
		int z5_num_inputs,
		const dict<int, int> &z5_to_z_input_map,
		const vector<int> &z_dont_care_indices
	);

	/**
	 * 寻找最佳双输出割（两阶段过滤）⭐⭐⭐
	 *
	 * @param now 当前节点（Z输出）
	 * @param Q 优先队列
	 * @return 最佳双输出割（如果找到）
	 *
	 * @note 阶段1：快速结构化筛选（廉价）
	 * @note 阶段2：精确真值表验证（昂贵，只对少数候选）
	 */
	DoubleCut findBestDoubleCut(
		SigBit now,
		const std::multiset<SingleCut, CutComparator> &Q
	);

	// 辅助方法
	int countSuccessors(SigBit signal);

	/**
	 * 寻找可映射的驱动门（穿透IO缓冲器）
	 *
	 * @param signal 起始信号
	 * @return 可映射的组合逻辑门，如果未找到则返回nullptr
	 *
	 * @note 会"穿透"GTP_BUF、GTP_INV、GTP_OUTBUF等单输入缓冲器
	 * @note 遇到$_*组合逻辑门时停止并返回
	 * @note 遇到其他类型（FF、RAM、多输入原语等）时返回nullptr
	 */
	Cell* find_mappable_driver(SigBit signal);
};

YOSYS_NAMESPACE_END

#endif // GLOBAL_MERGER_H
