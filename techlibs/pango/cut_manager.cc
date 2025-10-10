/*
 * cut_manager.cc
 *
 * CutManager 实现
 *
 * 作者：根据8号文档Section 3.6实现
 * 日期：2025-10-04
 */

#include "cut_manager.h"

YOSYS_NAMESPACE_BEGIN

CutManager::CutManager(
	Module *module,
	HeuristicEvaluator *evaluator,
	TimingAnalyzer *timing,
	GraphUtils *graph)
	: module(module), evaluator(evaluator), timing(timing), graph(graph),
	  max_cut_size(6), max_cuts(20)
{
}

void CutManager::computePriorityCuts(int max_cut_size, int max_cuts)
{
	this->max_cut_size = max_cut_size;
	this->max_cuts = max_cuts;

	// 清空之前的数据
	priority_cuts.clear();
	cuts_by_size.clear();

	log("CutManager: Computing priority cuts (K=%d, P=%d)...\n", max_cut_size, max_cuts);

	// ===== 调试：统计cell类型 =====
	dict<IdString, int> cell_type_count;
	for (auto cell : module->cells()) {
		cell_type_count[cell->type]++;
	}
	log("CutManager: Cell type statistics:\n");
	for (auto &pair : cell_type_count) {
		log("  %s: %d\n", log_id(pair.first), pair.second);
	}

	// 1. 获取拓扑序
	const vector<SigBit> &topo_order = graph->getTopologicalOrder();

	log("CutManager: Processing %zu signals in topological order\n", topo_order.size());

	// 2. 按拓扑序遍历每个信号
	int processed_signals = 0;
	for (SigBit signal : topo_order) {
		// 获取驱动这个信号的门
		Cell *driver = graph->getDriver(signal);

		if (!driver) {
			// 主输入或常量：添加trivial cut（平凡割）
			Cut trivial;
			trivial.insert(signal);
			cuts_by_size[signal][1].push_back(trivial);
			processed_signals++;
		} else {
			// 内部门：枚举割
			enumerateCutsForGate(driver);
			processed_signals++;
		}

		// 3. 选择优先割
		selectPriorityCuts(signal);
	}

	log("CutManager: Priority cuts computed for %d signals.\n", processed_signals);
}

SingleCut CutManager::getBestCut(SigBit signal) const
{
	if (priority_cuts.count(signal) && !priority_cuts.at(signal).empty()) {
		return priority_cuts.at(signal)[0];  // 第一个是最优的
	}

	// 退化情况：返回平凡割（信号自身）
	SingleCut trivial;
	trivial.inputs.insert(signal);
	trivial.output = signal;
	return trivial;
}

const vector<SingleCut>& CutManager::getPriorityCuts(SigBit signal) const
{
	static vector<SingleCut> empty_vec;
	if (priority_cuts.count(signal)) {
		return priority_cuts.at(signal);
	}
	return empty_vec;
}

const vector<Cut>& CutManager::getCutsBySize(SigBit signal, int size) const
{
	static vector<Cut> empty_vec;
	if (cuts_by_size.count(signal) && cuts_by_size.at(signal).count(size)) {
		return cuts_by_size.at(signal).at(size);
	}
	return empty_vec;
}

void CutManager::enumerateCutsForGate(Cell *gate)
{
	// 获取门的输出信号
	SigBit output = graph->getCellOutput(gate);
	if (!output.wire) return;  // 如果门没有有效输出，则跳过

	// 获取门的输入信号
	vector<SigBit> inputs_vec = graph->getCellInputs(gate);

	pool<Cut> new_cuts;
	// ⭐⭐⭐ 修复：不应为内部门添加平凡割{output}，因为那会创建自环 ⭐⭐⭐
	// 平凡割只应用于主输入(PI)，在computePriorityCuts中的lines 58-60已正确处理
	// 内部门的割应该从其输入的割合并得到

	if (inputs_vec.empty()) {
		// 没有输入的门（例如常量生成器），使用平凡割
		new_cuts.insert({output});
	}
	else if (inputs_vec.size() == 1) {
		// 单输入门 (NOT, BUF)，直接继承其输入的所有优先割的输入集
		const auto& input_priority_cuts = getPriorityCuts(inputs_vec[0]);
		if (input_priority_cuts.empty()) {
			// ⭐⭐⭐ 修复：如果输入没有优先割，创建使用输入信号的平凡割 ⭐⭐⭐
			// 注意：这里使用输入信号inputs_vec[0]，而不是输出信号output
			new_cuts.insert({inputs_vec[0]});
		} else {
			for (const auto& sc : input_priority_cuts) {
				new_cuts.insert(sc.inputs);
			}
		}
	}
	else {
		// 多输入门，进行割合并
		// 初始集合：第一个输入的割
		pool<Cut> merged_so_far;
		const auto& first_input_cuts = getPriorityCuts(inputs_vec[0]);
		if (first_input_cuts.empty()) { // 如果第一个输入没有割，至少给它一个平凡割
			merged_so_far.insert({inputs_vec[0]});
		} else {
			for(const auto& sc : first_input_cuts) {
				merged_so_far.insert(sc.inputs);
			}
		}

		// 依次与后续输入的割集合并
		for (size_t i = 1; i < inputs_vec.size(); i++) {
			pool<Cut> next_merged;
			const auto& next_input_cuts = getPriorityCuts(inputs_vec[i]);

			pool<Cut> current_input_cuts;
			if (next_input_cuts.empty()) {
				current_input_cuts.insert({inputs_vec[i]});
			} else {
				for(const auto& sc : next_input_cuts) {
					current_input_cuts.insert(sc.inputs);
				}
			}

			// 笛卡尔积合并
			for (const auto& cut1 : merged_so_far) {
				for (const auto& cut2 : current_input_cuts) {
					Cut merged = cut1;
					merged.insert(cut2.begin(), cut2.end());
					if ((int)merged.size() <= max_cut_size) {
						next_merged.insert(merged);
					}
				}
			}
			merged_so_far = next_merged;  // 更新合并结果
		}
		new_cuts.insert(merged_so_far.begin(), merged_so_far.end());
	}

	// 将所有新生成的、唯一的割按大小分类存储
	for (const auto& cut : new_cuts) {
		cuts_by_size[output][cut.size()].push_back(cut);
	}
}

void CutManager::mergeCuts(const vector<Cut> &cuts_a, const vector<Cut> &cuts_b,
                            vector<Cut> &result)
{
	result.clear();

	// 合并策略：计算每对割的并集
	for (const Cut &cut_a : cuts_a) {
		for (const Cut &cut_b : cuts_b) {
			// 计算并集
			Cut merged = cut_a;
			for (SigBit bit : cut_b) {
				merged.insert(bit);
			}

			// 检查大小限制
			if ((int)merged.size() <= max_cut_size) {
				result.push_back(merged);
			}
		}
	}
}

void CutManager::selectPriorityCuts(SigBit signal)
{
	// 收集所有候选割
	vector<SingleCut> all_cuts;

	// 从cuts_by_size中收集所有大小的割
	for (int k = 1; k <= max_cut_size; k++) {
		if (cuts_by_size.count(signal) && cuts_by_size[signal].count(k)) {
			for (const Cut &cut : cuts_by_size[signal][k]) {
				// 构造SingleCut
				SingleCut single_cut(cut, signal);
				all_cuts.push_back(single_cut);
			}
		}
	}

	if (all_cuts.empty()) {
		return;
	}

	// 使用evaluator排序（lambda函数捕获this指针）
	std::sort(all_cuts.begin(), all_cuts.end(),
	          [this](const SingleCut &a, const SingleCut &b) {
		          return evaluator->compareSingleCuts(a, b);
	          });

	// 选择前P个
	int num_to_keep = std::min((int)all_cuts.size(), max_cuts);
	priority_cuts[signal].assign(
		all_cuts.begin(),
		all_cuts.begin() + num_to_keep
	);
}

CutManager::Statistics CutManager::getStatistics() const
{
	Statistics stats;

	stats.total_signals = priority_cuts.size();
	stats.total_cuts = 0;

	for (const auto &pair : priority_cuts) {
		stats.total_cuts += pair.second.size();
	}

	if (stats.total_signals > 0) {
		stats.avg_cuts_per_signal = (double)stats.total_cuts / stats.total_signals;
	}

	return stats;
}

// ===== P2方案：双输出感知的CutManager辅助函数（32号文档 Section 2.1.4）⭐⭐⭐ =====

/**
 * 获取信号的拓扑邻居节点
 *
 * 功能：收集扇入驱动节点和扇出读者节点
 * 用途：为双输出感知的割保留策略识别潜在的合并伙伴
 *
 * @param signal 信号
 * @return 拓扑邻居信号集合
 */
pool<SigBit> CutManager::getTopologicalNeighbors(SigBit signal)
{
	pool<SigBit> neighbors;

	// 1. 扇入邻居：signal 驱动节点的所有输出
	Cell *driver = graph->getDriver(signal);
	if (driver && driver->type.begins_with("$_")) {
		// 获取驱动节点的所有输入
		auto driver_inputs = graph->getCellInputs(driver);
		for (SigBit input : driver_inputs) {
			// 获取每个输入的驱动节点
			Cell *input_driver = graph->getDriver(input);
			if (input_driver && input_driver->type.begins_with("$_")) {
				SigBit input_driver_out = graph->getCellOutput(input_driver);
				if (input_driver_out.wire && input_driver_out != signal) {
					neighbors.insert(input_driver_out);
				}
			}
		}
	}

	// 2. 扇出邻居：signal 的读者节点的所有输出
	auto readers = graph->getReaders(signal);
	for (Cell *reader : readers) {
		if (reader->type.begins_with("$_")) {
			SigBit reader_out = graph->getCellOutput(reader);
			if (reader_out.wire && reader_out != signal) {
				neighbors.insert(reader_out);
			}
		}
	}

	return neighbors;
}

/**
 * 检查割与邻居节点的输入共享潜力
 *
 * 功能：判断一个割是否与其拓扑邻居有较高的输入共享度
 * 标准：如果与任意邻居共享≥3个输入，认为有双输出合并潜力
 *
 * @param cut 候选割
 * @param signal 割的输出信号
 * @return 如果有高共享潜力返回true，否则返回false
 */
bool CutManager::checkNeighborInputSharing(const SingleCut& cut, SigBit signal)
{
	// 获取signal的拓扑邻居
	pool<SigBit> neighbors = getTopologicalNeighbors(signal);

	for (SigBit neighbor : neighbors) {
		if (!priority_cuts.count(neighbor)) continue;

		// 检查这个cut与邻居的最佳割是否有高共享度
		const SingleCut& neighbor_best = getBestCut(neighbor);

		int shared = 0;
		for (SigBit input : cut.inputs) {
			if (neighbor_best.inputs.count(input)) {
				shared++;
			}
		}

		// 如果共享3个以上输入，认为有双输出潜力
		if (shared >= 3) {
			return true;
		}
	}

	return false;
}

YOSYS_NAMESPACE_END
