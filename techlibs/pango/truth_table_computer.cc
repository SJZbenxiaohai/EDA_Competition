/*
 * truth_table_computer.cc
 *
 * TruthTableComputer 实现
 *
 * 作者：根据8号文档Section 3.4实现
 * 日期：2025-10-04
 */

#include "truth_table_computer.h"

YOSYS_NAMESPACE_BEGIN

TruthTableComputer::TruthTableComputer(Module *module, SigMap &sigmap)
	: module(module), sigmap(sigmap)
{
	ct.setup_internals();
	ct.setup_stdcells();
}

Const TruthTableComputer::computeLUTInit(
	SigBit output,
	const vector<SigBit> &inputs)
{
	// 创建 ConstEval 实例
	ConstEval evaluator(module);

	// 标记边界（停止评估点）
	for (SigBit input : inputs) {
		evaluator.stop(input);
	}

	// 枚举所有输入组合
	int num_inputs = inputs.size();
	int num_combinations = 1 << num_inputs;
	vector<State> init_bits(num_combinations);

	for (int combo = 0; combo < num_combinations; combo++) {
		evaluator.clear();

		// 设置输入值
		for (int i = 0; i < num_inputs; i++) {
			State value = (combo & (1 << i)) ? State::S1 : State::S0;
			evaluator.set(inputs[i], Const(value));
		}

		// 评估输出
		SigSpec result(output);
		bool success = evaluator.eval(result);

		if (success && result.is_fully_const()) {
			init_bits[combo] = result.as_const()[0];
		} else {
			// ⭐ MVP策略：ConstEval失败则报错
			log_error("ConstEval failed for signal %s at combination %d\n",
			         log_signal(output), combo);
		}
	}

	return Const(init_bits);
}

bool TruthTableComputer::isIndependentOfInputs(
	const Const &init,
	int num_inputs,
	const vector<int> &dont_care_indices) const
{
	// ===== v1.1实现 - 8号文档Section 3.4.4 =====
	// 算法：对于每个 don't care 输入，翻转其位，检查输出是否相同

	// 边界情况：没有 don't care 输入
	if (dont_care_indices.empty()) {
		return true;
	}

	int table_size = 1 << num_inputs;

	// 遍历所有输入组合
	for (int combo = 0; combo < table_size; combo++) {
		// 对于每个 don't care 输入，翻转其值，检查输出是否相同
		for (int dc_index : dont_care_indices) {
			// 翻转 dc_index 位
			int flipped_combo = combo ^ (1 << dc_index);

			// 如果翻转后的组合已经检查过，跳过（避免重复）
			if (flipped_combo < combo) continue;

			// 比较两个组合的输出
			if (init[combo] != init[flipped_combo]) {
				// 输出不同，说明依赖于这个输入
				log_debug("  Truth table depends on input %d: "
				         "combo=0x%x -> %c, flipped=0x%x -> %c\n",
				         dc_index, combo,
				         init[combo] == State::S1 ? '1' : '0',
				         flipped_combo,
				         init[flipped_combo] == State::S1 ? '1' : '0');
				return false;
			}
		}
	}

	return true;  // 所有 don't care 输入都不影响输出
}

Const TruthTableComputer::projectTruthTable(
	const Const &init,
	int num_inputs,
	const dict<int, bool> &fixed_inputs) const
{
	// ===== v1.1实现 - 8号文档Section 3.4.5 =====
	// 算法：将高维真值表投影到低维空间，固定某些输入

	// 计算投影后的输入数量
	int remaining_inputs = num_inputs - fixed_inputs.size();
	int projected_size = 1 << remaining_inputs;
	vector<State> projected_bits(projected_size);

	// 遍历投影后的所有组合
	for (int proj_combo = 0; proj_combo < projected_size; proj_combo++) {
		// 构建完整的输入组合
		int full_combo = 0;
		int proj_bit = 0;  // 当前处理的投影位

		for (int i = 0; i < num_inputs; i++) {
			if (fixed_inputs.count(i)) {
				// 这是固定输入，使用固定值
				if (fixed_inputs.at(i)) {
					full_combo |= (1 << i);
				}
			} else {
				// 这是可变输入，从投影组合中取值
				if (proj_combo & (1 << proj_bit)) {
					full_combo |= (1 << i);
				}
				proj_bit++;
			}
		}

		// 从原始真值表查找输出
		projected_bits[proj_combo] = init[full_combo];

		log_debug("  Project: proj_combo=0x%x -> full_combo=0x%x -> output=%c\n",
		         proj_combo, full_combo,
		         projected_bits[proj_combo] == State::S1 ? '1' : '0');
	}

	return Const(projected_bits);
}

bool TruthTableComputer::verifySupportedGateTypes(Module *module)
{
	// MVP版本：ConstEval支持所有Yosys内置门类型
	// 只需检查是否有未知的Cell类型
	bool all_supported = true;

	for (auto cell : module->cells()) {
		if (!ct.cell_known(cell->type)) {
			log_warning("Unknown cell type: %s (cell %s)\n",
			           log_id(cell->type), log_id(cell->name));
			all_supported = false;
		}
	}

	if (all_supported) {
		log("TruthTableComputer: All gate types supported by ConstEval\n");
	}

	return all_supported;
}

YOSYS_NAMESPACE_END
