/*
 * timing_analyzer.h
 *
 * TimingAnalyzer - 时序分析器（SSOT - 单一数据源）
 * 作为所有深度/时序信息的唯一数据源
 *
 * 功能：
 * - 计算到达时间（AT）和需求时间（RT）
 * - 提供整数深度接口（供常规模块使用）
 * - 提供浮点时序接口（供LocalRefiner使用）
 *
 * 作者：根据8号文档Section 3.2实现
 * 日期：2025-10-04
 */

#ifndef TIMING_ANALYZER_H
#define TIMING_ANALYZER_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "graph_utils.h"

YOSYS_NAMESPACE_BEGIN

// 前向声明
class GraphUtils;

class TimingAnalyzer {
public:
	// ===== 构造和初始化 =====

	TimingAnalyzer(Module *module, SigMap &sigmap, GraphUtils *graph);

	/**
	 * 计算所有信号的到达时间（前向分析）
	 * @note 必须在使用查询接口前调用
	 */
	void computeArrivalTimes();

	/**
	 * 计算所有信号的需求时间（反向分析）
	 * @param target_depth 目标深度（通常是关键路径深度）
	 * @note 必须在 computeArrivalTimes() 之后调用
	 */
	void computeRequiredTimes(int target_depth);

	// ===== 深度查询接口（整数，供常规模块使用）=====

	/**
	 * 获取信号的整数深度
	 * @param signal 信号
	 * @return 从主输入到此信号的最长路径（层数）
	 */
	int getDepth(SigBit signal) const {
		if (!arrival_time.count(signal)) {
			return 0;  // 主输入或常量
		}
		return (int)std::ceil(arrival_time.at(signal));
	}

	/**
	 * 获取割的深度
	 * @param inputs 割的输入集合
	 * @return max(depth(input)) + 1
	 */
	int getCutDepth(const pool<SigBit> &inputs) const;

	// ===== 精确时序查询接口（浮点，供LocalRefiner使用）=====

	/**
	 * 获取精确的到达时间（浮点数）
	 * @param signal 信号
	 * @return 到达时间（单位：门延迟）
	 */
	float getArrivalTime(SigBit signal) const;

	/**
	 * 获取需求时间
	 * @param signal 信号
	 * @return 需求时间
	 */
	float getRequiredTime(SigBit signal) const;

	/**
	 * 获取时序裕量
	 * @param signal 信号
	 * @return slack = required_time - arrival_time
	 */
	float getSlack(SigBit signal) const {
		return getRequiredTime(signal) - getArrivalTime(signal);
	}

	// ===== 全局信息 =====

	/**
	 * 获取关键路径深度
	 * @return 电路的最大深度
	 */
	int getCriticalPathDepth() const {
		return critical_depth;
	}

	/**
	 * 批量导出到达时间map（供时序信息传递使用）
	 * @return 所有信号的到达时间
	 * @note 用于工具链间的时序信息传递
	 */
	const dict<SigBit, float>& getArrivalTimeMap() const {
		return arrival_time;
	}

	// ===== 调试接口 =====

	void printTimingReport() const;

private:
	Module *module;
	SigMap &sigmap;
	GraphUtils *graph;

	// 时序数据（浮点数，精确）
	dict<SigBit, float> arrival_time;
	dict<SigBit, float> required_time;

	// 全局信息
	int critical_depth;

	// 辅助方法
	void computeATRecursive(SigBit signal);
	void computeRTRecursive(SigBit signal);

	/**
	 * 获取门延迟
	 * @note 简化模型：所有门延迟为1（单位延迟模型，文档4确认）
	 */
	float getGateDelay(Cell *gate) const {
		return 1.0f;
	}
};

YOSYS_NAMESPACE_END

#endif // TIMING_ANALYZER_H
