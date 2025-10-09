/*
 * global_merger.cc
 *
 * GlobalMerger 实现
 * MVP版本：仅实现单输出映射
 *
 * 作者：根据8号文档Section 3.7实现
 * 日期：2025-10-04
 */

#include "global_merger.h"
#include "kernel/celltypes.h"

YOSYS_NAMESPACE_BEGIN

GlobalMerger::GlobalMerger(
	Module *module,
	CutManager *cut_mgr,
	HeuristicEvaluator *evaluator,
	TimingAnalyzer *timing,
	TruthTableComputer *truth_table,
	GraphUtils *graph)
	: module(module), cut_mgr(cut_mgr), evaluator(evaluator),
	  timing(timing), truth_table(truth_table), graph(graph), enable_double_output(false)
{
	// 初始化sigmap
	sigmap.set(module);
}

void GlobalMerger::runGlobalMapping()
{
	single_mappings.clear();
	double_mappings.clear();
	log("GlobalMerger: Starting global mapping...\n");

	CutComparator comparator(evaluator);
	std::multiset<SingleCut, CutComparator> Q(comparator);
	pool<SigBit> visited;

	// ===== 步骤1: 收集所有需要映射的组合逻辑门输出 =====
	pool<SigBit> all_comb_outputs;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			SigBit output = graph->getCellOutput(cell);
			if (output.wire) {
				all_comb_outputs.insert(output);
			}
		}
	}
	log("  Found %zu combinational gate outputs to map.\n", all_comb_outputs.size());

	// ===== 步骤2: 找出主输出及其驱动器 =====
	pool<SigBit> po_signals;
	for (auto wire : module->wires()) {
		if (wire->port_output) {
			for (int i = 0; i < wire->width; i++) {
				po_signals.insert(sigmap(SigBit(wire, i)));
			}
		}
	}
	log("  Found %zu primary output signals.\n", po_signals.size());

	// ===== 步骤3: 初始化队列 - 优先从主输出的驱动器开始 =====
	log("  Step 3: Initializing queue from primary output drivers...\n");
	int initialized_from_po = 0;
	int po_count = 0;
	for (SigBit po_bit : po_signals) {
		po_count++;
		Cell* mappable_driver = find_mappable_driver(po_bit);
		if (mappable_driver) {
			SigBit driver_output = graph->getCellOutput(mappable_driver);
			if (!visited.count(driver_output)) {
				SingleCut best_cut = cut_mgr->getBestCut(driver_output);
				Q.insert(best_cut);
				visited.insert(driver_output);
				initialized_from_po++;

				// Log details for first 5 initialized signals
				if (initialized_from_po <= 5) {
					log("    PO %d: %s -> driver: %s (type: %s) -> output: %s\n",
					    initialized_from_po, log_signal(po_bit),
					    log_id(mappable_driver->name), log_id(mappable_driver->type),
					    log_signal(driver_output));
					log("      Best cut has %zu inputs\n", best_cut.inputs.size());
				}
			}
		} else {
			// Log why this PO didn't yield a mappable driver
			if (po_count <= 5) {
				log("    PO %d: %s -> no mappable driver found\n", po_count, log_signal(po_bit));
			}
		}
	}
	log("  Step 3: Initialized %d signals from %d primary outputs.\n", initialized_from_po, (int)po_signals.size());

	// ===== 步骤4: 始终从所有组合逻辑门开始（确保完整覆盖）⭐⭐⭐ =====
	// 修复前：只有当Q.empty()时才执行
	// 修复后：无论PO初始化结果如何，都添加所有组合逻辑门，确保完整覆盖
	// 原因：大多数设计的PO直接连接FF，backward traversal会在FF边界停止，
	//      导致只有少数节点被初始化，大部分逻辑无法覆盖
	log("  Step 4: Adding all combinational gates to queue for complete coverage...\n");
	int step4_added = 0;
	for (SigBit comb_output : all_comb_outputs) {
		if (!visited.count(comb_output)) {
			Q.insert(cut_mgr->getBestCut(comb_output));
			visited.insert(comb_output);
			step4_added++;
		}
	}
	log("  Step 4: Added %d combinational gates to queue.\n", step4_added);

	log("  Initialized queue with %zu signals.\n", Q.size());

	// ===== 步骤5: 主循环 =====
	int processed_nodes = 0;
	while (!Q.empty()) {
		auto it = Q.begin();
		SingleCut now_cut = *it;
		Q.erase(it);
		SigBit now = now_cut.output;
		processed_nodes++;

		// Detailed logging for first 5 processed nodes
		if (processed_nodes <= 5) {
			log("  Processing node %d: %s (cut size: %zu)\n",
			    processed_nodes, log_signal(now), now_cut.inputs.size());
			log("    Cut inputs: ");
			int input_idx = 0;
			for (SigBit input : now_cut.inputs) {
				Cell* driver = graph->getDriver(input);
				if (driver) {
					log("%s (driven by %s [%s]) ", log_signal(input),
					    log_id(driver->name), log_id(driver->type));
				} else {
					log("%s (no driver - PI or const) ", log_signal(input));
				}
				input_idx++;
				if (input_idx >= 6) {
					log("...");
					break;
				}
			}
			log("\n");
		}

		// 决策：双输出 vs 单输出 (v1.1 逻辑)
		bool use_double = false;
		DoubleCut best_double;
		if (enable_double_output) {
			best_double = findBestDoubleCut(now, Q);
			if (best_double.valid()) {
				use_double = true;
			}
		}

		// 获取本次映射所覆盖的输入信号
		const pool<SigBit>& inputs_to_expand = use_double ? best_double.inputs : now_cut.inputs;

		// 更新映射结果
		if (use_double) {
			double_mappings[{now, best_double.output2}] = best_double;
			visited.insert(best_double.output2); // 标记Z5也被映射了
		} else {
			single_mappings[now] = now_cut;
		}

		// ===== 步骤6: 扩展遍历边界 (已修正 - 根本性修复) ⭐⭐⭐ =====
		// 将新发现的、需要映射的逻辑门加入队列
		// 修复前：无条件加入所有输入，包括主输入和FF输出
		// 修复后：只加入由组合逻辑驱动的输入
		int step6_expanded = 0;
		for (SigBit input_bit : inputs_to_expand) {
			if (visited.count(input_bit)) continue;

			Cell* driver = graph->getDriver(input_bit);

			// ⭐ 关键：只有当输入是由另一个需要映射的组合逻辑驱动时，才将其加入队列
			if (driver && driver->type.begins_with("$_")) {
				SigBit driver_output = graph->getCellOutput(driver);
				if (driver_output == input_bit) { // 确认是这个门的输出
					Q.insert(cut_mgr->getBestCut(driver_output));
					visited.insert(driver_output); // 预先标记，避免重复
					step6_expanded++;
				}
			}
			// 如果输入是主输入或来自FF，则遍历到此为止，不加入队列
		}

		if (processed_nodes <= 5) {  // Only log for first few nodes
			log("  Node %d: Step 6 expanded %d new nodes to queue (cut had %zu inputs)\n",
			    processed_nodes, step6_expanded, inputs_to_expand.size());
		}
	}

	// ===== 步骤7: 确保所有组合逻辑门都被映射 =====
	// 有些组合逻辑门可能不在遍历路径中，仍需要映射
	log("  Step 7: Checking %zu combinational outputs, %zu already visited.\n",
	    all_comb_outputs.size(), visited.size());

	int step7_added = 0;
	for (SigBit comb_output : all_comb_outputs) {
		if (!visited.count(comb_output)) {
			single_mappings[comb_output] = cut_mgr->getBestCut(comb_output);
			visited.insert(comb_output);
			processed_nodes++;
			step7_added++;
		}
	}

	log("  Step 7: Added %d unmapped gates to single_mappings.\n", step7_added);
	log("GlobalMerger: Traversal complete. Processed %d nodes.\n", processed_nodes);
	log("GlobalMerger: Found %zu single LUTs, %zu double LUTs.\n",
	    single_mappings.size(), double_mappings.size());

	// ===== 调试：统计割的大小分布 =====
	dict<int, int> cut_size_distribution;
	for (const auto &pair : single_mappings) {
		int size = pair.second.inputs.size();
		cut_size_distribution[size]++;
	}
	log("GlobalMerger: Cut size distribution:\n");
	for (int size = 1; size <= 6; size++) {
		if (cut_size_distribution.count(size)) {
			log("  Size %d: %d cuts\n", size, cut_size_distribution[size]);
		}
	}
}

MappingResult GlobalMerger::getResult() const
{
	MappingResult result;
	result.single_mappings = single_mappings;
	result.double_mappings = double_mappings;

	// TODO: 计算统计信息
	result.stats.num_single_luts = single_mappings.size();
	result.stats.num_double_luts = double_mappings.size();
	result.stats.total_luts = result.stats.num_single_luts + result.stats.num_double_luts;

	return result;
}

int GlobalMerger::countSuccessors(SigBit signal)
{
	// TODO: 实现后继节点计数
	return 0;
}

Cell* GlobalMerger::find_mappable_driver(SigBit signal)
{
	SigBit current_signal = sigmap(signal);
	static int debug_count = 0;  // Track first few calls for debugging
	bool should_log = (debug_count < 10);  // Log first 10 calls
	debug_count++;

	if (should_log) {
		log("    find_mappable_driver for %s:\n", log_signal(signal));
	}

	for (int i = 0; i < 100; ++i) // 循环保护
	{
		Cell* driver = graph->getDriver(current_signal);

		if (!driver) {
			if (should_log) {
				log("      Step %d: No driver for %s (likely PI)\n", i, log_signal(current_signal));
			}
			return nullptr;
		}

		if (should_log) {
			log("      Step %d: %s driven by %s [%s]\n", i, log_signal(current_signal),
			    log_id(driver->name), log_id(driver->type));
		}

		// 目标：找到可映射的组合逻辑门
		if (driver->type.begins_with("$_")) {
			if (should_log) {
				log("      Found combinational gate!\n");
			}
			return driver;
		}

		// ⭐⭐⭐ 修复：扩展可穿透的单元类型 ⭐⭐⭐
		// 检查是否是可"穿透"的单输入缓冲/反相器类型
		// 必须确保这些单元只有一个输入和一个输出
		bool is_transparent = false;
		SigBit next_signal;

		// 单输入缓冲器/反相器（可以穿透）
		if (driver->type.in(ID(GTP_BUF), ID(GTP_INV),
		                   ID(GTP_OUTBUF), ID(GTP_INBUF))) {

			// 严谨地获取单输入信号
			SigSpec input_spec;
			int input_count = 0;
			for (auto &conn : driver->connections()) {
				if (yosys_celltypes.cell_input(driver->type, conn.first)) {
					input_spec = sigmap(conn.second);
					input_count++;
				}
			}

			// 只有当单元严格为单输入时才继续追溯
			if (input_count == 1 && input_spec.is_bit()) {
				next_signal = input_spec.as_bit();
				is_transparent = true;
			}
		}

		if (is_transparent && next_signal.wire) {
			if (should_log) {
				log("      Penetrating through %s\n", log_id(driver->type));
			}
			current_signal = next_signal;
			continue; // 继续向后追溯
		}

		// 遇到其他任何类型的单元（FF, RAM, 多输入原语等），都视为边界，停止追溯
		if (should_log) {
			log("      Hit boundary at %s [%s], stopping\n",
			    log_id(driver->name), log_id(driver->type));
		}
		return nullptr;
	}
	log_warning("find_mappable_driver exceeded max loop iterations for signal %s\n", log_signal(signal));
	return nullptr;
}

// ===== v1.1 双输出合并功能（阶段9）⭐⭐⭐ =====

/**
 * 任务9.1: 检查输入兼容性（修正2核心）⭐⭐⭐
 *
 * 功能：检查Z5的输入是否是Z剩余输入的子集，并返回精确的映射关系
 *
 * 实现：严格按照文档8 Section 9.4.2的伪代码
 * 日期：2025-10-05
 */
bool GlobalMerger::checkInputCompatibility(
	const Cut &z_remaining,
	const Cut &z5_inputs,
	dict<int, int> &z5_to_z_input_map,
	vector<int> &z_dont_care_indices)
{
	// 清空输出参数
	z5_to_z_input_map.clear();
	z_dont_care_indices.clear();

	// ===== 步骤1：将无序集合转为有序向量（便于索引）=====
	vector<SigBit> z_vec(z_remaining.begin(), z_remaining.end());
	vector<SigBit> z5_vec(z5_inputs.begin(), z5_inputs.end());

	// ⚠️ 关键：排序以确保确定性（pool<SigBit>的迭代顺序不保证）
	// 文档8 Section 9.4.2强调
	std::sort(z_vec.begin(), z_vec.end());
	std::sort(z5_vec.begin(), z5_vec.end());

	// ===== 步骤2：检查Z5的每个输入是否在Z中，并建立映射 =====
	for (size_t i = 0; i < z5_vec.size(); i++) {
		bool found = false;
		for (size_t j = 0; j < z_vec.size(); j++) {
			if (z5_vec[i] == z_vec[j]) {
				// 找到匹配：Z5的第i个输入对应Z的第j个输入
				z5_to_z_input_map[i] = j;
				found = true;
				break;
			}
		}

		if (!found) {
			// Z5 有 Z 中不存在的输入，不兼容
			log_debug("  checkInputCompatibility: Z5 input %s not found in Z\n",
			         log_signal(z5_vec[i]));
			return false;
		}
	}

	// ===== 步骤3：找出 Z 中未被 Z5 使用的输入（don't care）=====
	// 先收集被Z5使用的Z索引
	pool<int> used_z_indices;
	for (const auto &pair : z5_to_z_input_map) {
		used_z_indices.insert(pair.second);
	}

	// 遍历Z的所有索引，未使用的加入 don't care 列表
	for (size_t j = 0; j < z_vec.size(); j++) {
		if (!used_z_indices.count(j)) {
			z_dont_care_indices.push_back(j);
		}
	}

	// ===== 步骤4：调试验证（一致性检查）=====
	#ifdef DEBUG_MODE
	// 映射大小应等于Z5的输入数量
	log_assert(z5_to_z_input_map.size() == z5_vec.size());
	// don't care数量应等于Z减去Z5的输入数量
	log_assert(z_dont_care_indices.size() == z_vec.size() - z5_vec.size());
	#endif

	// 调试日志：输出映射关系（帮助诊断）
	log_debug("  checkInputCompatibility: compatible\n");
	log_debug("    Z inputs: %zu, Z5 inputs: %zu\n", z_vec.size(), z5_vec.size());
	log_debug("    Mapping: ");
	for (const auto &pair : z5_to_z_input_map) {
		log_debug("Z5[%d]->Z[%d] ", pair.first, pair.second);
	}
	log_debug("\n");
	log_debug("    Don't care indices: ");
	for (int idx : z_dont_care_indices) {
		log_debug("%d ", idx);
	}
	log_debug("\n");

	return true;
}

/**
 * 任务9.2: 计算结构化启发式分数（阶段1筛选用）⭐
 *
 * 功能：快速评估双输出候选的优劣（无需真值表）
 *
 * 实现：严格按照文档8 Section 3.7.4和文档7 Section 2.3.2
 * 日期：2025-10-05
 */
float GlobalMerger::computeStructuralScore(
	SigBit z_output,
	SigBit z5_output,
	const Cut &merged_inputs,
	SigBit selected_i5)
{
	float score = 0.0f;

	// ===== 因子1：合并后的输入数量（越少越好）=====
	score += merged_inputs.size() * heuristic_config.input_count_weight;

	// ===== 因子2：深度惩罚（如果深度增加，分数增加）=====
	int z_depth = timing->getDepth(z_output);
	int z5_depth = timing->getDepth(z5_output);
	int merged_depth = timing->getCutDepth(merged_inputs) + 1;

	if (merged_depth > z_depth || merged_depth > z5_depth) {
		// 深度增加，惩罚
		score += heuristic_config.depth_penalty_weight;
	}

	// ===== 因子3：面积流估算（粗略，不需要精确计算）=====
	int total_successors = countSuccessors(z_output) +
	                      countSuccessors(z5_output);
	float estimated_area_flow = (float)(merged_inputs.size() + 1) /
	                            std::max(1, total_successors);
	score += estimated_area_flow * heuristic_config.area_flow_weight;

	// ===== 因子4：输入共享度（共享越多越好，权重是负值）=====
	SingleCut z_cut = cut_mgr->getBestCut(z_output);
	SingleCut z5_cut = cut_mgr->getBestCut(z5_output);

	// 计算共享输入数量
	int shared_inputs = 0;
	for (SigBit input : z_cut.inputs) {
		if (z5_cut.inputs.count(input)) {
			shared_inputs++;
		}
	}

	score += shared_inputs * heuristic_config.input_sharing_weight;

	log_debug("  computeStructuralScore: Z=%s, Z5=%s, I5=%s\n",
	         log_signal(z_output), log_signal(z5_output), log_signal(selected_i5));
	log_debug("    merged_inputs=%zu, z_depth=%d, z5_depth=%d, merged_depth=%d\n",
	         merged_inputs.size(), z_depth, z5_depth, merged_depth);
	log_debug("    total_successors=%d, shared_inputs=%d, score=%.2f\n",
	         total_successors, shared_inputs, score);

	return score;
}

/**
 * 任务9.3: 验证真值表约束（修正2核心）⭐⭐⭐
 *
 * 功能：严谨验证Z与Z5是否满足双输出合并的真值表约束
 *
 * 实现：严格按照文档8 Section 3.7.5和文档7 Section 2.3.4
 * ⚠️⚠️⚠️ 绝不使用文档7 Section 2.3.3的简化版本
 * 日期：2025-10-05
 */
bool GlobalMerger::verifyTruthTableConstraint(
	const Const &z_init,
	const Const &z5_init,
	int z_num_inputs,
	int z5_num_inputs,
	const dict<int, int> &z5_to_z_input_map,  // 修正2新增 ⭐
	const vector<int> &z_dont_care_indices)   // 修正2新增 ⭐
{
	// ===== 情况1：Z 是 6 输入，Z5 是 ≤5 输入 =====
	if (z_num_inputs == 6) {

		int expected_z5_size = 1 << z5_num_inputs;
		int expected_z_size = 64;

		// 验证真值表大小
		if (z_init.size() != expected_z_size) {
			log_warning("Z init size mismatch: expected %d, got %d\n",
			           expected_z_size, (int)z_init.size());
			return false;
		}

		if (z5_init.size() != expected_z5_size) {
			log_warning("Z5 init size mismatch: expected %d, got %d\n",
			           expected_z5_size, (int)z5_init.size());
			return false;
		}

		// 提取 Z 的 INIT[31:0]（当 I5=0 时的投影）
		Const z_lower_half = z_init.extract(0, 32);

		// ===== 修正2：使用精确映射，不再假设Z5使用低位输入 ⭐⭐⭐ =====

		if (z5_num_inputs < 5) {
			// Z5 使用的输入少于 5 个
			// 需要验证 z_lower_half（5 输入）在未被 Z5 使用的输入上是独立的

			// ⭐ 修正2：使用传入的 dont_care_indices，而不是假设
			// dont_care_indices 是由 checkInputCompatibility 精确计算的

			// 检查 z_lower_half 是否不依赖于这些输入
			if (!truth_table->isIndependentOfInputs(
			        z_lower_half, 5, z_dont_care_indices))
			{
				log_debug("  z_lower_half depends on don't care inputs\n");
				return false;
			}

			// 投影 z_lower_half 到 Z5 的输入空间
			dict<int, bool> fixed_inputs;
			for (int dc_idx : z_dont_care_indices) {
				fixed_inputs[dc_idx] = false;  // 固定为 0
			}

			Const z_projected = truth_table->projectTruthTable(
			    z_lower_half, 5, fixed_inputs);

			// 比较投影后的真值表与 Z5 的真值表
			if (z_projected.size() != z5_init.size()) {
				log_warning("Projected size mismatch: %zu vs %zu\n",
				           z_projected.size(), z5_init.size());
				return false;
			}

			for (size_t i = 0; i < z_projected.size(); i++) {
				if (z_projected[i] != z5_init[i]) {
					log_debug("  Projected truth table mismatch at bit %zu\n", i);
					return false;
				}
			}

		} else {
			// Z5 使用 5 个输入，直接比较
			if (z_lower_half.size() != z5_init.size()) {
				return false;
			}

			for (size_t i = 0; i < z_lower_half.size(); i++) {
				if (z_lower_half[i] != z5_init[i]) {
					log_debug("  Truth table mismatch at bit %zu\n", i);
					return false;
				}
			}
		}

		log_debug("  verifyTruthTableConstraint: passed (6-input case)\n");
		return true;

	} else {
		// ===== 情况2：Z 和 Z5 的输入数都 ≤5 =====
		// 此时它们的真值表必须完全相同

		if (z_init.size() != z5_init.size()) {
			return false;
		}

		for (size_t i = 0; i < z_init.size(); i++) {
			if (z_init[i] != z5_init[i]) {
				return false;
			}
		}

		log_debug("  verifyTruthTableConstraint: passed (≤5-input case)\n");
		return true;
	}
}

/**
 * 任务9.4: 寻找最佳双输出割（两阶段过滤）⭐⭐⭐
 *
 * 功能：找到可以与Z合并的最佳Z5，实现双输出LUT合并
 *
 * 实现：严格按照文档8 Section 3.7.3和文档7 Section 2.2
 * 性能关键：两阶段过滤，阶段1筛选至≤5个候选
 * 日期：2025-10-05
 */
DoubleCut GlobalMerger::findBestDoubleCut(
	SigBit now,
	const std::multiset<SingleCut, CutComparator> &Q)
{
	SingleCut now_cut = cut_mgr->getBestCut(now);

	// ===== 前置检查 =====
	if (now_cut.inputs.size() < 2 || now_cut.inputs.size() > 6) {
		return DoubleCut{};  // Z 必须有 2-6 个输入
	}

	// ===== 阶段1: 快速结构化筛选（廉价）⭐⭐⭐ =====
	log_debug("findBestDoubleCut: Stage 1 filtering for Z=%s\n", log_signal(now));

	vector<CandidatePair> promising_candidates;

	for (const SingleCut &other : Q) {
		if (other.output == now) continue;

		// ⭐⭐⭐ 修复：防止自环 - Z5的输入不能包含Z5自己 ⭐⭐⭐
		if (other.inputs.count(other.output)) {
			log_debug("  Skipping candidate %s: self-loop detected\n",
			         log_signal(other.output));
			continue;
		}

		// Z5的输入数量约束
		if (other.inputs.size() > 5) continue;

		// ⭐ I5选择循环：尝试Z的每个输入作为I5（文档7 Section 3）
		for (SigBit potential_i5 : now_cut.inputs) {

			// 约束1：I5 不能是 Z5 的输入
			if (other.inputs.count(potential_i5)) {
				continue;
			}

			// 构建去除I5后的Z输入集合
			Cut z_remaining = now_cut.inputs;
			z_remaining.erase(potential_i5);

			// 约束2：检查输入兼容性（修正2：获取精确映射）⭐⭐⭐
			dict<int, int> z5_to_z_map;
			vector<int> dont_care_indices;

			if (!checkInputCompatibility(z_remaining, other.inputs,
			                            z5_to_z_map, dont_care_indices)) {
				continue;
			}

			// 约束3：合并后的总输入数 ≤ 6
			Cut merged_inputs = z_remaining;
			merged_inputs.insert(other.inputs.begin(), other.inputs.end());
			merged_inputs.insert(potential_i5);  // 重新加入I5

			if (merged_inputs.size() > 6) continue;

			// 计算启发式分数（廉价）
			float score = computeStructuralScore(
				now, other.output,
				merged_inputs,
				potential_i5
			);

			promising_candidates.push_back({
				other.output,
				other.inputs,
				potential_i5,
				z_remaining,
				score,
				z5_to_z_map,        // 修正2新增 ⭐
				dont_care_indices   // 修正2新增 ⭐
			});
		}
	}

	if (promising_candidates.empty()) {
		log_debug("  Stage 1: No candidates found\n");
		return DoubleCut{};
	}

	// 按启发式分数排序，只保留前5个
	std::sort(promising_candidates.begin(), promising_candidates.end(),
	          [](const CandidatePair &a, const CandidatePair &b) {
		          return a.structural_score < b.structural_score;
	          });

	const int MAX_CANDIDATES_FOR_VERIFICATION = 5;
	size_t original_count = promising_candidates.size();
	if (promising_candidates.size() > MAX_CANDIDATES_FOR_VERIFICATION) {
		promising_candidates.resize(MAX_CANDIDATES_FOR_VERIFICATION);
	}

	log_debug("  Stage 1: Filtered %zu candidates down to %zu\n",
	          original_count, promising_candidates.size());

	// ===== 阶段2: 精确真值表验证（昂贵，只对≤5个候选）⭐⭐⭐ =====
	log_debug("findBestDoubleCut: Stage 2 truth table verification\n");

	for (const CandidatePair &candidate : promising_candidates) {

		// ⚠️ 昂贵的操作：只对极少数候选执行

		log_debug("  Verifying candidate: Z5=%s, I5=%s\n",
		         log_signal(candidate.z5_output),
		         log_signal(candidate.selected_i5));

		// 1. 构建Z的输入向量（I5作为最后一个输入）
		vector<SigBit> z_input_vec;
		for (SigBit inp : candidate.z_remaining_inputs) {
			z_input_vec.push_back(inp);
		}
		z_input_vec.push_back(candidate.selected_i5);  // I5是最后一个

		// 2. 计算Z的真值表
		Const z_init = truth_table->computeLUTInit(now, z_input_vec);

		// 3. 计算Z5的真值表
		vector<SigBit> z5_input_vec(
			candidate.z5_inputs.begin(),
			candidate.z5_inputs.end()
		);
		Const z5_init = truth_table->computeLUTInit(
			candidate.z5_output,
			z5_input_vec
		);

		// 4. 验证约束：Z5 = Z[I5=0]（修正2：传入精确映射）⭐⭐⭐
		if (!verifyTruthTableConstraint(
		        z_init, z5_init,
		        z_input_vec.size(),
		        z5_input_vec.size(),
		        candidate.z5_to_z_input_map,  // 修正2新增 ⭐
		        candidate.z_dont_care_indices // 修正2新增 ⭐
		    ))
		{
			log_debug("  Truth table constraint failed for candidate %s\n",
			         log_signal(candidate.z5_output));
			continue;
		}

		// 5. 找到第一个满足约束的候选，构建DoubleCut
		DoubleCut dc;
		dc.inputs = candidate.z_remaining_inputs;
		dc.inputs.insert(candidate.z5_inputs.begin(), candidate.z5_inputs.end());
		dc.inputs.insert(candidate.selected_i5);
		dc.output1 = now;       // Z
		dc.output2 = candidate.z5_output;  // Z5
		dc.selected_i5 = candidate.selected_i5;  // 记录I5

		log_debug("  Found valid double-cut: Z=%s, Z5=%s, I5=%s\n",
		         log_signal(now),
		         log_signal(candidate.z5_output),
		         log_signal(candidate.selected_i5));

		return dc;
	}

	// 所有候选都未通过真值表验证
	log_debug("  Stage 2: All candidates failed verification\n");
	return DoubleCut{};
}

YOSYS_NAMESPACE_END
