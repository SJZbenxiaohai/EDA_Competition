/*
 * timing_analyzer.cc
 *
 * TimingAnalyzer 实现
 *
 * 作者：根据8号文档Section 3.2实现
 * 日期：2025-10-04
 */

#include "timing_analyzer.h"

YOSYS_NAMESPACE_BEGIN

TimingAnalyzer::TimingAnalyzer(Module *module, SigMap &sigmap, GraphUtils *graph)
	: module(module), sigmap(sigmap), graph(graph), critical_depth(0)
{
}

void TimingAnalyzer::computeArrivalTimes()
{
	arrival_time.clear();

	// 主输入 AT = 0
	for (auto wire : module->wires()) {
		if (wire->port_input) {
			for (int i = 0; i < wire->width; i++) {
				SigBit bit(wire, i);
				arrival_time[sigmap(bit)] = 0.0f;
			}
		}
	}

	// 常量 AT = 0
	arrival_time[State::S0] = 0.0f;
	arrival_time[State::S1] = 0.0f;

	// 按拓扑序计算（利用 GraphUtils）
	critical_depth = 0;
	for (SigBit signal : graph->getTopologicalOrder()) {
		Cell *driver = graph->getDriver(signal);
		if (!driver) continue;

		// AT(output) = max(AT(inputs)) + delay(gate)
		float max_input_at = 0.0f;
		for (SigBit input : graph->getCellInputs(driver)) {
			if (arrival_time.count(input)) {
				max_input_at = std::max(max_input_at, arrival_time[input]);
			}
		}

		float at = max_input_at + getGateDelay(driver);
		arrival_time[signal] = at;

		critical_depth = std::max(critical_depth, (int)std::ceil(at));
	}

	log("TimingAnalyzer: Critical path depth = %d\n", critical_depth);
}

void TimingAnalyzer::computeRequiredTimes(int target_depth)
{
	required_time.clear();

	// 主输出 RT = target_depth
	for (auto wire : module->wires()) {
		if (wire->port_output) {
			for (int i = 0; i < wire->width; i++) {
				SigBit bit(wire, i);
				required_time[sigmap(bit)] = (float)target_depth;
			}
		}
	}

	// 按反向拓扑序计算（从输出到输入）
	for (SigBit signal : graph->getReverseTopologicalOrder()) {
		Cell *driver = graph->getDriver(signal);
		if (!driver) continue;

		// 如果信号没有RT（不在主输出的扇入锥中），跳过
		if (!required_time.count(signal)) {
			continue;
		}

		float signal_rt = required_time[signal];

		// RT(input) = min(RT(outputs)) - delay(gate)
		for (SigBit input : graph->getCellInputs(driver)) {
			float input_rt = signal_rt - getGateDelay(driver);

			if (required_time.count(input)) {
				// 已有RT，取最小值
				required_time[input] = std::min(required_time[input], input_rt);
			} else {
				// 第一次设置RT
				required_time[input] = input_rt;
			}
		}
	}
}

int TimingAnalyzer::getCutDepth(const pool<SigBit> &inputs) const
{
	int max_depth = 0;
	for (SigBit input : inputs) {
		max_depth = std::max(max_depth, getDepth(input));
	}
	return max_depth + 1;  // +1 是新LUT的延迟
}

float TimingAnalyzer::getArrivalTime(SigBit signal) const
{
	if (arrival_time.count(signal)) {
		return arrival_time.at(signal);
	}
	return 0.0f;
}

float TimingAnalyzer::getRequiredTime(SigBit signal) const
{
	if (required_time.count(signal)) {
		return required_time.at(signal);
	}
	return (float)critical_depth;
}

void TimingAnalyzer::printTimingReport() const
{
	log("Timing Analysis Report:\n");
	log("  Critical path depth: %d\n", critical_depth);
	log("  Signals analyzed: %zu\n", arrival_time.size());
}

YOSYS_NAMESPACE_END
