/*
 * dual_output_mapper.cc
 *
 * DualOutputMapper 实现
 *
 * 作者：根据8号文档Section 3.9, 4.1实现
 * 日期：2025-10-04
 */

#include "dual_output_mapper.h"

YOSYS_NAMESPACE_BEGIN

DualOutputMapper::DualOutputMapper(Module *module, SigMap &sigmap)
	: module(module), sigmap(sigmap)
{
}

void DualOutputMapper::run()
{
	// ===== 调试：统计输入cell类型 =====
	dict<IdString, int> cell_type_count;
	for (auto cell : module->cells()) {
		cell_type_count[cell->type]++;
	}
	log("DualOutputMapper: Input module statistics:\n");
	log("  Total cells: %zu\n", module->cells().size());
	for (auto &pair : cell_type_count) {
		log("  %s: %d\n", log_id(pair.first), pair.second);
	}

	// ===== 阶段1: 创建所有模块 =====
	createModules();

	// ===== 阶段2: 第一趟 - 深度优化 =====
	log("Starting depth-oriented mapping...\n");
	runDepthMapping();

	// 注意：在第一趟映射后不生成网表，避免删除Cell导致后续迭代出错

	// ===== 阶段3: 多趟 - 面积流优化（迭代至收敛）=====
	log("Starting area-flow oriented mapping...\n");
	runAreaFlowMapping(10);  // 最多10次迭代

	// ===== 阶段4: 最后一趟 - 精确面积优化 =====
	log("Starting exact-area oriented mapping...\n");
	runExactAreaMapping();

	// ===== 阶段5: 局部优化（v1.3）=====
	// log("Starting local refinement...\n");
	// runLocalRefinement();

	// ===== 阶段6: 生成最终网表（只在所有映射完成后调用）=====
	log("Generating final netlist...\n");
	generateNetlist();
}

void DualOutputMapper::createModules()
{
	// ===== 修正4：使用 std::make_unique 创建对象 ⭐ =====
	//
	// 按依赖顺序创建，使用智能指针自动管理生命周期
	//
	graph = std::make_unique<GraphUtils>(module, sigmap);

	timing = std::make_unique<TimingAnalyzer>(module, sigmap, graph.get());
	timing->computeArrivalTimes();
	timing->computeRequiredTimes(timing->getCriticalPathDepth());

	context = std::make_unique<MappingContext>(module, sigmap, graph.get());
	truth_table = std::make_unique<TruthTableComputer>(module, sigmap);
	evaluator = std::make_unique<HeuristicEvaluator>(module, context.get(), timing.get());
	cut_mgr = std::make_unique<CutManager>(module, evaluator.get(), timing.get(), graph.get());
	merger = std::make_unique<GlobalMerger>(module, cut_mgr.get(), evaluator.get(),
	                                       timing.get(), truth_table.get(), graph.get());

	// ===== 启用双输出映射功能 ⭐⭐⭐ =====
	merger->setEnableDoubleOutput(true);  // Re-enabled for dual-output testing
	log("DualOutputMapper: Dual-output mapping ENABLED\n");

	// v1.3
	// refiner = std::make_unique<LocalRefiner>(module, timing.get());

	// 注意：所有子模块都会在 DualOutputMapper 析构时自动销毁，无需手动 delete
}

void DualOutputMapper::runDepthMapping()
{
	// Debug: Count combinational gates before depth mapping
	int comb_count = 0;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			comb_count++;
		}
	}
	log("DualOutputMapper: %d combinational gates before depth mapping\n", comb_count);

	evaluator->setMode(EvaluationMode::DEPTH);
	cut_mgr->computePriorityCuts(6, 20);
	merger->runGlobalMapping();

	// Debug: Count after depth mapping
	comb_count = 0;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			comb_count++;
		}
	}
	log("DualOutputMapper: %d combinational gates after depth mapping\n", comb_count);
}

void DualOutputMapper::runAreaFlowMapping(int max_iterations)
{
	// Debug: Count before area-flow mapping
	int comb_count = 0;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			comb_count++;
		}
	}
	log("DualOutputMapper: %d combinational gates before area-flow mapping\n", comb_count);

	evaluator->setMode(EvaluationMode::AREA_FLOW);

	int prev_area = INT_MAX;

	for (int iter = 0; iter < max_iterations; iter++) {
		log("  Iteration %d...\n", iter + 1);

		context->startNewIteration();

		// ⭐⭐⭐ 关键修复：合并单双输出映射信息 ⭐⭐⭐
		auto all_mappings = merger->getSingleMappings();

		for (const auto& pair : merger->getDoubleMappings()) {
			const DoubleCut& dcut = pair.second;

			// 双输出LUT的两个输出都需要记录
			// 注意：两个输出共享相同的输入集合（这是GTP_LUT6D的硬件特性）
			all_mappings[dcut.output1] = SingleCut(dcut.inputs, dcut.output1);
			all_mappings[dcut.output2] = SingleCut(dcut.inputs, dcut.output2);
		}

		context->recoverReferences(all_mappings);  // ✅ 传入完整映射

		cut_mgr->computePriorityCuts(6, 20);
		merger->runGlobalMapping();

		int current_area = merger->getSingleMappings().size() +
		                  merger->getDoubleMappings().size();

		log("    Area: %d LUTs\n", current_area);

		// 收敛检查
		if (std::abs(current_area - prev_area) <= 1) {
			log("  Converged after %d iterations\n", iter + 1);
			break;
		}

		prev_area = current_area;
	}

	// 打印缓存统计
	auto stats = context->getPerformanceStats();
	log("  Cache hit rate: %.2f%%\n", stats.cache_hit_rate() * 100);

	// Debug: Count after area-flow mapping
	comb_count = 0;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			comb_count++;
		}
	}
	log("DualOutputMapper: %d combinational gates after area-flow mapping\n", comb_count);
}

void DualOutputMapper::runExactAreaMapping()
{
	// Debug: Count before exact-area mapping
	int comb_count = 0;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			comb_count++;
		}
	}
	log("DualOutputMapper: %d combinational gates before exact-area mapping\n", comb_count);

	evaluator->setMode(EvaluationMode::EXACT_AREA);

	context->startNewIteration();

	// ⭐⭐⭐ 关键修复：合并单双输出映射信息 ⭐⭐⭐
	auto all_mappings = merger->getSingleMappings();

	for (const auto& pair : merger->getDoubleMappings()) {
		const DoubleCut& dcut = pair.second;

		// 双输出LUT的两个输出都需要记录
		// 注意：两个输出共享相同的输入集合（这是GTP_LUT6D的硬件特性）
		all_mappings[dcut.output1] = SingleCut(dcut.inputs, dcut.output1);
		all_mappings[dcut.output2] = SingleCut(dcut.inputs, dcut.output2);
	}

	context->recoverReferences(all_mappings);  // ✅ 传入完整映射

	cut_mgr->computePriorityCuts(6, 20);
	merger->runGlobalMapping();

	// Debug: Count after exact-area mapping
	comb_count = 0;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			comb_count++;
		}
	}
	log("DualOutputMapper: %d combinational gates after exact-area mapping\n", comb_count);
}

void DualOutputMapper::generateNetlist()
{
	log("DualOutputMapper: Generating LUT netlist...\n");

	// ===== 调试：检查当前模块中的组合逻辑门数量 =====
	int initial_comb_count = 0;
	for (auto cell : module->cells()) {
		if (cell->type.begins_with("$_")) {
			initial_comb_count++;
		}
	}
	log("DualOutputMapper: Found %d combinational gates before netlist generation.\n", initial_comb_count);

	const auto &single_mappings = merger->getSingleMappings();
	const auto &double_mappings = merger->getDoubleMappings();

	// ===== MVP: 处理单输出映射 =====

	for (const auto &pair : single_mappings) {
		SigBit output = pair.first;
		const SingleCut &cut = pair.second;

		// ⭐ 防御性检查：跳过双输出映射中的节点
		bool is_in_double = false;
		for (const auto &dpair : double_mappings) {
			if (dpair.first.first == output || dpair.first.second == output) {
				is_in_double = true;
				break;
			}
		}
		if (is_in_double) {
			log_debug("Skipping single mapping for %s: already in double mapping\n",
			          log_signal(output));
			continue;
		}

		// ⭐⭐⭐ 修复5：跳过平凡割（trivial cut），避免生成自环LUT ⭐⭐⭐
		// 平凡割：输入只包含输出自身，表示这个信号不需要用LUT实现
		// 这些gates由原始逻辑提供，不需要生成LUT
		if (cut.inputs.size() == 1 && cut.inputs.count(output)) {
			log_debug("  Skipping trivial cut for output %s\n", log_signal(output));
			continue;
		}

		// ===== 步骤1: 准备输入向量（有序，最多6个）=====
		vector<SigBit> inputs_vec;
		for (SigBit input : cut.inputs) {
			inputs_vec.push_back(input);
		}

		// 排序以保证确定性（重要：真值表计算依赖顺序）
		std::sort(inputs_vec.begin(), inputs_vec.end());

		// ===== 步骤2: 计算INIT参数 =====
		Const init_value;
		try {
			init_value = truth_table->computeLUTInit(output, inputs_vec);
		} catch (...) {
			log_error("Failed to compute LUT INIT for output %s\n",
			         log_signal(output));
		}

		// ===== 步骤3: 创建GTP_LUT6单元 =====
		IdString lut_name = module->uniquify(stringf("\\lut_%s", log_id(output.wire->name)));
		Cell *lut_cell = module->addCell(lut_name, ID(GTP_LUT6));

		// ===== 步骤4: 设置INIT参数 =====
		lut_cell->setParam(ID(INIT), init_value);

		// ===== 步骤5: 连接输入（I0~I5）=====
		// 补齐到6个输入（未使用的连接到常量0）
		for (int i = 0; i < 6; i++) {
			IdString port_name = stringf("\\I%d", i);
			if (i < (int)inputs_vec.size()) {
				lut_cell->setPort(port_name, inputs_vec[i]);
			} else {
				// 未使用的输入连接到常量0
				lut_cell->setPort(port_name, State::S0);
			}
		}

		// ===== 步骤6: 连接输出（Z）=====
		lut_cell->setPort(ID(Z), output);

		// ⭐⭐⭐ 删除步骤7: 不再标记旧Cell待删除 ⭐⭐⭐
	}

	// ===== v1.1: 处理双输出映射 ⭐⭐⭐ =====

	for (const auto &pair : double_mappings) {
		SigBit z_output = pair.first.first;   // Z输出
		SigBit z5_output = pair.first.second; // Z5输出
		const DoubleCut &dcut = pair.second;

		SigBit i5 = dcut.selected_i5;

		// ===== 步骤1: 构建输入向量（I0-I4, I5）=====
		// 提取非I5的输入（最多5个），排序确保确定性
		vector<SigBit> non_i5_inputs;
		for (SigBit inp : dcut.inputs) {
			if (inp != i5) {
				non_i5_inputs.push_back(inp);
			}
		}
		std::sort(non_i5_inputs.begin(), non_i5_inputs.end());

		// 构建Z的完整输入向量：[I0, I1, I2, I3, I4, I5]
		// I5必须是最后一个输入（与硬件MUX结构对应）
		vector<SigBit> z_inputs_vec = non_i5_inputs;
		z_inputs_vec.push_back(i5);

		// ===== 步骤2: 计算Z的真值表 =====
		Const z_init;
		try {
			z_init = truth_table->computeLUTInit(z_output, z_inputs_vec);
		} catch (...) {
			log_error("Failed to compute LUT INIT for Z output %s\n",
			         log_signal(z_output));
		}

		// ===== 步骤3: 准备INIT参数（必须是64位）=====
		// GTP_LUT6D硬件定义:
		//   INIT[31:0]  - 下半部分（I5=0时），也是Z5的输出
		//   INIT[63:32] - 上半部分（I5=1时）
		Const init_value;
		int z_num_inputs = z_inputs_vec.size();

		if (z_num_inputs == 6) {
			// 标准情况：Z是6输入
			// z_init应该是64位
			if (z_init.size() != 64) {
				log_error("Z init size mismatch for 6-input: expected 64, got %d\n",
				         z_init.size());
			}
			init_value = z_init;

		} else {
			// Z的输入数<6
			// 需要扩展到64位（上下两半相同）
			int table_size = 1 << z_num_inputs;
			vector<State> init_bits;

			// 复制下半部分
			for (int i = 0; i < table_size; i++) {
				init_bits.push_back(z_init[i]);
			}

			// 填充到32位
			while (init_bits.size() < 32) {
				init_bits.push_back(State::S0);
			}

			// 复制到上半部分（重复下半部分）
			for (int i = 0; i < 32; i++) {
				init_bits.push_back(init_bits[i]);
			}

			log_assert(init_bits.size() == 64);
			init_value = Const(init_bits);
		}

		// ===== 步骤4: 创建GTP_LUT6D单元 =====
		IdString lut_name = module->uniquify(
			stringf("\\lutd_%s_%s",
			        log_id(z_output.wire->name),
			        log_id(z5_output.wire->name)));
		Cell *lut_cell = module->addCell(lut_name, ID(GTP_LUT6D));

		// ===== 步骤5: 设置INIT参数 =====
		lut_cell->setParam(ID(INIT), init_value);

		// ===== 步骤6: 连接输入端口（I0-I5）=====
		// I0-I4: 连接非I5的输入（最多5个）
		for (int i = 0; i < 5; i++) {
			IdString port_name = stringf("\\I%d", i);
			if (i < (int)non_i5_inputs.size()) {
				lut_cell->setPort(port_name, non_i5_inputs[i]);
			} else {
				// 未使用的输入连接到常量0
				lut_cell->setPort(port_name, State::S0);
			}
		}

		// I5: 连接选中的I5输入（MUX选择信号）
		lut_cell->setPort(ID(I5), i5);

		// ===== 步骤7: 连接输出端口（Z, Z5）=====
		lut_cell->setPort(ID(Z), z_output);
		lut_cell->setPort(ID(Z5), z5_output);

		// ⭐⭐⭐ 删除步骤8: 不再标记旧Cell待删除 ⭐⭐⭐

		log_debug("  Created GTP_LUT6D: %s (Z=%s, Z5=%s, I5=%s, inputs=%zu)\n",
		         log_id(lut_name), log_signal(z_output), log_signal(z5_output),
		         log_signal(i5), non_i5_inputs.size());
	}

	// ⭐⭐⭐ 删除步骤9: 不再使用cells_to_remove统一删除 ⭐⭐⭐

	// ===== 步骤10: 最终清理 - 删除所有剩余的组合逻辑门 ⭐⭐⭐ =====
	// 修复策略：使用与单输出映射器完全相同的逻辑（synth_pango.cc:2679-2684）
	// 使用 module->cells_ 而不是 module->cells() 避免迭代器失效问题

	log("DualOutputMapper: Removing all combinational gates...\n");

	int removed_count = 0;
	for (auto c : module->cells_) {
		if (c.second->type.begins_with("$_")) {
			module->remove(c.second);
			removed_count++;
		}
	}

	log("DualOutputMapper: Removed %d combinational gates.\n", removed_count);

	log("DualOutputMapper: Generated %zu single-output LUTs, %zu double-output LUTs\n",
	    single_mappings.size(), double_mappings.size());

	// 保存最终结果
	final_result = merger->getResult();
}

MappingResult DualOutputMapper::getResult() const
{
	return merger->getResult();
}

dict<SigBit, float> DualOutputMapper::getBit2DepthMap() const
{
	dict<SigBit, float> result;

	// ⭐⭐⭐ 关键优化：直接获取 TimingAnalyzer 的内部 arrival_time map ⭐⭐⭐
	// 一次性转换：arrival_time -> depth（向上取整）
	const auto& arrival_map = timing->getArrivalTimeMap();

	for (const auto& pair : arrival_map) {
		result[pair.first] = std::ceil(pair.second);
	}

	log("DualOutputMapper: Exported %zu timing entries for lut_merge\n", result.size());

	return result;
}

YOSYS_NAMESPACE_END
