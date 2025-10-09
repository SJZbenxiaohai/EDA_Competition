/*
 * mapping_context.cc
 *
 * MappingContext 实现
 *
 * 作者：根据8号文档Section 3.3实现
 * 日期：2025-10-04
 */

#include "mapping_context.h"
#include "heuristic_evaluator.h"  // 包含SingleCut的完整定义

YOSYS_NAMESPACE_BEGIN

MappingContext::MappingContext(Module *module, SigMap &sigmap, GraphUtils *graph)
	: module(module), sigmap(sigmap), graph(graph), current_iteration(0)
{
}

int MappingContext::getFanoutRefs(SigBit signal) const
{
	if (fanout_refs.count(signal)) {
		return fanout_refs.at(signal);
	}
	return 0;
}

bool MappingContext::isUsed(SigBit signal) const
{
	return used.count(signal) && used.at(signal);
}

const SingleCut* MappingContext::getCurrentMapping(SigBit signal) const
{
	if (current_mapping.count(signal)) {
		return &current_mapping.at(signal);
	}
	return nullptr;
}

int MappingContext::getExactArea(SigBit signal)
{
	perf_stats.exact_area_calls++;

	// ===== 检查缓存 =====
	if (exact_area_cache.count(signal) &&
	    cache_iteration[signal] == current_iteration)
	{
		perf_stats.exact_area_cache_hits++;
		return exact_area_cache[signal];
	}

	// ===== 缓存未命中，计算面积 =====
	pool<SigBit> visited;
	int area = getExactAreaRecursive(signal, visited);

	// ===== 缓存结果 =====
	exact_area_cache[signal] = area;
	cache_iteration[signal] = current_iteration;

	return area;
}

int MappingContext::getExactAreaRecursive(
	SigBit signal,
	pool<SigBit> &visited)
{
	// 避免重复访问
	if (visited.count(signal)) {
		return 0;
	}
	visited.insert(signal);

	// 主输入或常量：面积为0
	if (!current_mapping.count(signal)) {
		return 0;
	}

	const SingleCut &cut = current_mapping.at(signal);

	// ===== 修正3：明确终止条件逻辑 =====
	//
	// 一个信号的LUT需要被物理实现（计入面积）当且仅当：
	// 1. 它是一个主输出（Primary Output）
	// 2. 或者它被多个内部LUT引用（fanout > 1）
	//
	// 如果它只被一个内部LUT引用且不是主输出，
	// 则它的面积可以被"吸收"（递归计算输入面积）。

	// 检查是否是主输出
	bool is_primary_output = false;
	for (auto wire : module->wires()) {
		if (wire->port_output) {
			for (int i = 0; i < wire->width; i++) {
				if (sigmap(SigBit(wire, i)) == sigmap(signal)) {
					is_primary_output = true;
					break;
				}
			}
			if (is_primary_output) break;
		}
	}

	// 终止条件：主输出或被多次引用
	if (is_primary_output || getFanoutRefs(signal) > 1) {
		return 1;  // 这个LUT需要物理实现
	} else {
		// 单次引用，可能被内联，递归计算输入的面积
		int area = 0;
		for (SigBit input : cut.inputs) {
			area += getExactAreaRecursive(input, visited);
		}
		return area;
	}
}

void MappingContext::recoverReferences(const dict<SigBit, SingleCut> &mapping)
{
	perf_stats.recover_calls++;

	// 清空引用计数和使用标记
	fanout_refs.clear();
	used.clear();

	// 保存当前映射
	current_mapping = mapping;

	// 收集所有主输出
	pool<SigBit> outputs;
	for (auto wire : module->wires()) {
		if (wire->port_output) {
			for (int i = 0; i < wire->width; i++) {
				SigBit bit = sigmap(SigBit(wire, i));
				outputs.insert(bit);
			}
		}
	}

	// BFS遍历，从输出开始，重建引用计数
	graph->bfsTraverse(outputs, [&](SigBit signal) {
		// 标记此信号被使用
		used[signal] = true;

		// 如果此信号有映射（是一个LUT输出）
		if (mapping.count(signal)) {
			const SingleCut &cut = mapping.at(signal);

			// 对于每个输入，增加引用计数
			for (SigBit input : cut.inputs) {
				fanout_refs[input]++;
			}
		}
	});
}

void MappingContext::dereference(SigBit signal, int &area_delta)
{
	area_delta = 0;

	// 如果信号没有映射，直接返回
	if (!current_mapping.count(signal)) {
		return;
	}

	const SingleCut &cut = current_mapping.at(signal);

	// 减少此LUT的输入的引用计数
	for (SigBit input : cut.inputs) {
		if (fanout_refs.count(input)) {
			fanout_refs[input]--;

			// 如果引用计数降为0，且输入也是一个映射的LUT
			// 递归dereference（面积可能减少）
			if (fanout_refs[input] == 0 && current_mapping.count(input)) {
				int input_area_delta = 0;
				dereference(input, input_area_delta);
				area_delta += input_area_delta;
			}
		}
	}

	// 当前LUT被移除，面积减1
	area_delta--;

	// 标记此信号未使用
	used[signal] = false;
}

void MappingContext::reference(SigBit signal)
{
	// 如果信号没有映射，直接返回
	if (!current_mapping.count(signal)) {
		return;
	}

	const SingleCut &cut = current_mapping.at(signal);

	// 增加此LUT的输入的引用计数
	for (SigBit input : cut.inputs) {
		fanout_refs[input]++;

		// 如果引用计数从0变为1，且输入也是一个映射的LUT
		// 递归reference（这个输入LUT现在需要物理实现）
		if (fanout_refs[input] == 1 && current_mapping.count(input)) {
			reference(input);
		}
	}

	// 标记此信号被使用
	used[signal] = true;
}

YOSYS_NAMESPACE_END
