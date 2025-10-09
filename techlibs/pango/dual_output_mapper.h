/*
 * dual_output_mapper.h
 *
 * DualOutputMapper - 双输出映射器总控模块
 * 协调所有子模块，执行多趟映射流程
 *
 * 功能：
 * - 创建和管理所有子模块
 * - 执行多趟映射流程（深度 -> 面积流 -> 精确面积）
 * - 生成最终网表
 *
 * 作者：根据8号文档Section 3.9实现
 * 日期：2025-10-04
 */

#ifndef DUAL_OUTPUT_MAPPER_H
#define DUAL_OUTPUT_MAPPER_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include <memory>  // 修正4：使用智能指针 ⭐

#include "graph_utils.h"
#include "timing_analyzer.h"
#include "mapping_context.h"
#include "truth_table_computer.h"
#include "heuristic_evaluator.h"
#include "cut_manager.h"
#include "global_merger.h"

YOSYS_NAMESPACE_BEGIN

class DualOutputMapper {
public:
	// ===== 构造和初始化 =====

	DualOutputMapper(Module *module, SigMap &sigmap);

	// ===== 主运行接口 =====

	/**
	 * 运行双输出映射器
	 * @note 完整流程见8号文档Section 4.1（第四部分）
	 */
	void run();

	// ===== 结果查询 =====

	MappingResult getResult() const;

private:
	Module *module;
	SigMap &sigmap;

	// ===== 修正4：使用智能指针管理子模块（建议修正）⭐ =====
	//
	// 问题：原设计使用裸指针（new/delete），容易内存泄漏。
	//
	// 解决：使用 std::unique_ptr 自动管理生命周期。
	//      当 DualOutputMapper 对象析构时，所有子模块自动销毁。
	//
	std::unique_ptr<GraphUtils> graph;
	std::unique_ptr<TimingAnalyzer> timing;
	std::unique_ptr<MappingContext> context;
	std::unique_ptr<TruthTableComputer> truth_table;
	std::unique_ptr<HeuristicEvaluator> evaluator;
	std::unique_ptr<CutManager> cut_mgr;
	std::unique_ptr<GlobalMerger> merger;
	// std::unique_ptr<LocalRefiner> refiner;  // v1.3

	// 最终结果
	MappingResult final_result;

	// 辅助方法
	void createModules();
	void runDepthMapping();
	void runAreaFlowMapping(int max_iterations = 10);
	void runExactAreaMapping();
	// void runLocalRefinement();  // v1.3
	void generateNetlist();
};

YOSYS_NAMESPACE_END

#endif // DUAL_OUTPUT_MAPPER_H
