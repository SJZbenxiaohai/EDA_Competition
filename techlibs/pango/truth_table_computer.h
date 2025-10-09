/*
 * truth_table_computer.h
 *
 * TruthTableComputer - 真值表生成器
 * MVP版本：基于ConstEval实现
 * v1.1版本：添加don't care独立性检查和投影功能
 *
 * 功能：
 * - 计算LUT的INIT参数（真值表）
 * - 验证门类型支持
 * - [v1.1] 独立性检查和真值表投影
 *
 * 作者：根据8号文档Section 3.4实现
 * 日期：2025-10-04
 */

#ifndef TRUTH_TABLE_COMPUTER_H
#define TRUTH_TABLE_COMPUTER_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "kernel/consteval.h"
#include "kernel/celltypes.h"

YOSYS_NAMESPACE_BEGIN

class TruthTableComputer {
public:
	// ===== 构造和初始化 =====

	TruthTableComputer(Module *module, SigMap &sigmap);

	// ===== MVP 核心接口 =====

	/**
	 * 计算 LUT 的 INIT 参数（真值表）
	 * @param output 输出信号
	 * @param inputs 输入信号向量（有序，最多6个）
	 * @return INIT 常量（2^N 位，N = inputs.size()）
	 *
	 * @note MVP版本：仅依赖 ConstEval
	 * @note v1.2版本：ConstEval失败时回退到手动仿真
	 */
	Const computeLUTInit(
		SigBit output,
		const vector<SigBit> &inputs
	);

	// ===== v1.1 关键新增接口（文档7，Section 2.3.4）⭐⭐⭐ =====

	/**
	 * 检查真值表是否不依赖于某些输入位
	 *
	 * @param init 完整的真值表
	 * @param num_inputs 总输入数
	 * @param dont_care_indices 不应该依赖的输入索引列表
	 *                          （0-based，例如 {3, 4} 表示第4和第5个输入）
	 * @return true 如果真值表不依赖于这些输入
	 *
	 * @note 关键功能：验证 Z 的下半部分在"don't care"输入上的独立性
	 * @note 用于双输出合并的约束验证
	 *
	 * @example
	 *   假设 init 是5输入函数 F(i4,i3,i2,i1,i0)
	 *   如果 dont_care_indices = {3, 4}（即 i3, i4）
	 *   则验证：∀ i2,i1,i0, F(...,i3=0,...) == F(...,i3=1,...)
	 *                      且 F(...,i4=0,...) == F(...,i4=1,...)
	 */
	bool isIndependentOfInputs(
		const Const &init,
		int num_inputs,
		const vector<int> &dont_care_indices
	) const;

	/**
	 * 提取真值表的子函数（投影）
	 *
	 * @param init 完整的真值表
	 * @param num_inputs 总输入数
	 * @param fixed_inputs 固定的输入值映射 {输入索引 -> 固定值(0/1)}
	 * @return 投影后的真值表
	 *
	 * @note 用于将高维真值表投影到低维输入空间
	 *
	 * @example
	 *   假设 init 是5输入函数 F(i4,i3,i2,i1,i0)
	 *   fixed_inputs = {3: false, 4: false}（固定 i3=0, i4=0）
	 *   则返回3输入函数 G(i2,i1,i0) = F(0,0,i2,i1,i0)
	 */
	Const projectTruthTable(
		const Const &init,
		int num_inputs,
		const dict<int, bool> &fixed_inputs
	) const;

	// ===== 调试和验证 =====

	/**
	 * 验证是否支持模块中的所有门类型
	 * @note MVP版本依赖ConstEval，支持所有Yosys内置门
	 * @note v1.2版本手动仿真支持5种门：AND/OR/XOR/NOT/MUX
	 */
	bool verifySupportedGateTypes(Module *module);

private:
	Module *module;
	SigMap &sigmap;

	// ConstEval 相关
	CellTypes ct;

	// v1.2 新增：手动仿真（暂不实现）
	// dict<SigBit, Cell*> bit2driver;
	// bool simulateConeManually(...);
};

YOSYS_NAMESPACE_END

#endif // TRUTH_TABLE_COMPUTER_H
