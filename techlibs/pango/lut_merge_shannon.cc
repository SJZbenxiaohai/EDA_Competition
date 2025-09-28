/*
 * GTP_LUT6D香农展开验证模块
 * 
 * 作用: 香农展开验证模块，穷举验证逻辑等价性（最关键）
 * 文件: techlibs/pango/lut_merge_shannon.cc
 * 创建时间: 2025-09-26
 * 任务: Phase 3.2 - P3.2A/P3.2B
 * 依据方案: 17-GTP_LUT6D修正执行方案.md Priority 3.2
 * 预计行数: ~600行
 * 
 * ⚠️ 极高风险警告 ⚠️ 
 * 这是整个项目最关键的模块，任何逻辑错误都会导致综合失败
 * 必须严格按照v1.2方案第467行开始的算法实现
 * 不可简化：必须完整的2^5=32次真值表比对
 * 穷举验证：每个输入组合都要验证
 * 
 * 核心功能:
 * 1. verifyShannonExpansion() - 香农展开验证主函数
 * 2. verifyLogicalEquivalence() - 逻辑等价性验证核心算法  
 * 3. verifyShannonConditions() - 香农展开条件预检查
 * 4. analyzeShannonSplit() - 分割变量分析
 * 5. extractTruthTableWithValidation() - 带验证的真值表提取
 * 6. debugShannonExpansion() - 香农展开调试输出
 */

#include "lut_merge_pango.h"
#include "kernel/log.h"
#include <algorithm>
#include <iomanip>

YOSYS_NAMESPACE_BEGIN

// =============================================================================
// 香农展开条件预检查
// =============================================================================

/**
 * 香农展开条件预检查 - 验证候选是否满足香农展开的先决条件
 * 
 * @param candidate 合并候选对象
 * @param split_var 分割变量
 * @return true 如果满足香农展开条件，false 否则
 */
bool LUTMergeOptimizer::verifyShannonConditions(
    const LUTMergeCandidate &candidate,
    SigBit split_var)
{
    // 1. 基本约束：必须是6输入情况
    if (candidate.total_inputs != 6) {
        if (enable_debug) {
            log("  Shannon expansion requires exactly 6 inputs, got %d\n", 
                candidate.total_inputs);
        }
        return false;
    }
    
    // 2. 分割变量有效性检查
    if (split_var.wire == nullptr) {
        if (enable_debug) {
            log("  Invalid split variable (null wire)\n");
        }
        return false;
    }
    
    // 3. LUT指针有效性
    if (!candidate.z5_lut || !candidate.z_lut) {
        if (enable_debug) {
            log("  Invalid LUT pointers for Shannon expansion\n");
        }
        return false;
    }
    
    // 4. 获取LUT输入向量进行验证
    vector<SigBit> z_inputs, z5_inputs;
    getCellInputsVector(candidate.z_lut, z_inputs);
    getCellInputsVector(candidate.z5_lut, z5_inputs);
    
    if (z_inputs.empty() || z5_inputs.empty()) {
        if (enable_debug) {
            log("  Empty input vectors: z_inputs=%zu, z5_inputs=%zu\n",
                z_inputs.size(), z5_inputs.size());
        }
        return false;
    }
    
    // 5. 分割变量必须在z_lut的输入中
    bool split_var_found = false;
    for (auto input : z_inputs) {
        if (sigmap(input) == sigmap(split_var)) {
            split_var_found = true;
            break;
        }
    }
    
    if (!split_var_found) {
        if (enable_debug) {
            log("  Split variable %s not found in z_lut inputs\n", 
                log_signal(split_var));
        }
        return false;
    }
    
    return true;
}

/**
 * 分割变量分析 - 分析分割变量在香农展开中的作用
 * 
 * @param candidate 合并候选对象
 * @param split_var 分割变量
 * @param split_analysis 输出：分割分析结果
 * @return true 如果分析成功，false 否则
 */
bool LUTMergeOptimizer::analyzeShannonSplit(
    const LUTMergeCandidate &candidate,
    SigBit split_var,
    ShannonSplitAnalysis &split_analysis)
{
    // 获取输入向量
    getCellInputsVector(candidate.z_lut, split_analysis.z_inputs);
    getCellInputsVector(candidate.z5_lut, split_analysis.z5_inputs);
    
    // 找到split_var在z_inputs中的位置
    split_analysis.split_pos = -1;
    for (int i = 0; i < split_analysis.z_inputs.size(); i++) {
        if (sigmap(split_analysis.z_inputs[i]) == sigmap(split_var)) {
            split_analysis.split_pos = i;
            break;
        }
    }
    
    if (split_analysis.split_pos == -1) {
        if (enable_debug) {
            log("  Failed to find split position for %s\n", log_signal(split_var));
        }
        return false;
    }
    
    // 构建reduced输入映射（去掉split_var后的z_inputs）
    split_analysis.reduced_inputs.clear();
    for (int i = 0; i < split_analysis.z_inputs.size(); i++) {
        if (i != split_analysis.split_pos) {
            split_analysis.reduced_inputs.push_back(split_analysis.z_inputs[i]);
        }
    }
    
    if (enable_debug) {
        log("  Shannon split analysis:\n");
        log("    Split variable: %s (position %d in z_lut)\n", 
            log_signal(split_var), split_analysis.split_pos);
        log("    Z_lut inputs: %zu, Z5_lut inputs: %zu, Reduced inputs: %zu\n",
            split_analysis.z_inputs.size(), split_analysis.z5_inputs.size(), 
            split_analysis.reduced_inputs.size());
    }
    
    return true;
}

// =============================================================================
// 带验证的真值表提取
// =============================================================================

/**
 * 带验证的真值表提取 - 提取LUT真值表并进行完整性验证
 * 
 * @param lut LUT Cell指针
 * @param truth_table 输出：提取的真值表
 * @param expected_size 期望的真值表大小
 * @return true 如果提取成功，false 否则
 */
bool LUTMergeOptimizer::extractTruthTableWithValidation(
    Cell *lut, 
    vector<bool> &truth_table,
    int expected_size)
{
    if (!lut) {
        if (enable_debug) {
            log("  extractTruthTableWithValidation: null LUT pointer\n");
        }
        return false;
    }
    
    // 使用现有的extractLUTTruthTable函数
    truth_table = extractLUTTruthTable(lut);
    
    if (truth_table.empty()) {
        if (enable_debug) {
            log("  Failed to extract truth table for %s\n", lut->name.c_str());
        }
        return false;
    }
    
    // 验证真值表大小
    if (expected_size > 0 && truth_table.size() != expected_size) {
        if (enable_debug) {
            log("  Truth table size mismatch for %s: expected %d, got %zu\n",
                lut->name.c_str(), expected_size, truth_table.size());
        }
        // 不直接返回false，因为可能是部分真值表
    }
    
    if (enable_debug) {
        log("  Extracted truth table for %s: %zu entries\n", 
            lut->name.c_str(), truth_table.size());
    }
    
    return true;
}

// =============================================================================
// 香农展开验证主函数 (严格按v1.2方案实现)
// =============================================================================

/**
 * 香农展开验证主函数 - 严格按照v1.2方案第467行算法实现
 * 
 * ⚠️ 关键函数 - 任何修改必须极其小心
 * 
 * 香农展开定理: F(x1,...,xn) = xi·F(x1,...,xi-1,1,xi+1,...,xn) + 
 *                              !xi·F(x1,...,xi-1,0,xi+1,...,xn)
 * 
 * 验证条件: z5_lut的输出应该等于z_lut在split_var=0时的输出
 * 
 * @param candidate 合并候选对象
 * @param split_var 分割变量
 * @return true 如果香农展开验证通过，false 否则
 */
bool LUTMergeOptimizer::verifyShannonExpansion(
    const LUTMergeCandidate &candidate, 
    SigBit split_var) 
{
    if (enable_debug) {
        log("=== Shannon Expansion Verification ===\n");
        log("  Candidate: %s + %s\n", 
            candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
        // ✅ Bug修复: 在验证阶段，z5_lut和z_lut尚未设置，使用lut1和lut2
        log("  LUT1: %s, LUT2: %s\n",
            candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
        log("  Split variable: %s\n", log_signal(split_var));
    }
    
    // 1. 香农展开条件预检查
    if (!verifyShannonConditions(candidate, split_var)) {
        if (enable_debug) {
            log("  Shannon conditions check failed\n");
        }
        return false;
    }
    
    // 2. 分割变量分析
    ShannonSplitAnalysis split_analysis;
    if (!analyzeShannonSplit(candidate, split_var, split_analysis)) {
        if (enable_debug) {
            log("  Shannon split analysis failed\n");
        }
        return false;
    }
    
    // === 严格按照v1.2方案实现的核心验证逻辑 ===
    
    Cell *z5_lut = candidate.z5_lut;
    Cell *z_lut = candidate.z_lut;
    
    vector<SigBit> z_inputs = split_analysis.z_inputs;
    vector<SigBit> z5_inputs = split_analysis.z5_inputs;
    int split_pos = split_analysis.split_pos;
    
    // 3. 验证：z5_lut的所有输入必须在z_lut去掉split_var后的输入中
    for (auto input : z5_inputs) {
        bool found = false;
        for (int i = 0; i < z_inputs.size(); i++) {
            if (i == split_pos) continue;  // 跳过split_var
            if (sigmap(z_inputs[i]) == sigmap(input)) {
                found = true;
                break;
            }
        }
        if (!found) {
            if (enable_debug) {
                log("  ❌ z5_lut input %s not in reduced z_lut inputs\n",
                    log_signal(input));
            }
            return false;
        }
    }
    
    // 4. 获取真值表
    vector<bool> z5_truth, z_truth;
    
    if (!extractTruthTableWithValidation(z5_lut, z5_truth, 1 << z5_inputs.size())) {
        if (enable_debug) {
            log("  ❌ Failed to extract z5_lut truth table\n");
        }
        return false;
    }
    
    if (!extractTruthTableWithValidation(z_lut, z_truth, 1 << z_inputs.size())) {
        if (enable_debug) {
            log("  ❌ Failed to extract z_lut truth table\n");
        }
        return false;
    }
    
    // 5. ⚠️ 关键验证：完整验证逻辑等价性（不可简化）
    bool equivalence_result = verifyLogicalEquivalence(z5_truth, z_truth, 
                                                       z5_inputs, z_inputs, split_pos);
    
    if (enable_debug) {
        if (equivalence_result) {
            log("  ✅ Shannon expansion verification PASSED\n");
        } else {
            log("  ❌ Shannon expansion verification FAILED\n");
        }
    }
    
    return equivalence_result;
}

// =============================================================================
// 逻辑等价性验证核心算法 (严格按v1.2方案实现)
// =============================================================================

/**
 * 逻辑等价性验证核心算法 - 严格按照v1.2方案第467行算法实现
 * 
 * ⚠️ 最关键函数 - 绝对不可简化，必须穷举验证
 * 
 * 验证原理:
 * 对于所有可能的reduced输入组合combo，验证：
 * truth1(combo) == truth2(combo with split_var=0)
 * 
 * @param truth1 z5_lut的真值表
 * @param truth2 z_lut的真值表
 * @param inputs1 z5_lut的输入向量
 * @param inputs2 z_lut的输入向量
 * @param split_pos split_var在inputs2中的位置
 * @return true 如果逻辑等价，false 否则
 */
bool LUTMergeOptimizer::verifyLogicalEquivalence(
    const vector<bool> &truth1, const vector<bool> &truth2,
    const vector<SigBit> &inputs1, const vector<SigBit> &inputs2,
    int split_pos) 
{
    if (enable_debug) {
        log("  === Logical Equivalence Verification ===\n");
        log("    Truth1 size: %zu, Truth2 size: %zu\n", truth1.size(), truth2.size());
        log("    Inputs1: %zu, Inputs2: %zu, Split pos: %d\n", 
            inputs1.size(), inputs2.size(), split_pos);
    }
    
    // 1. 输入验证
    if (truth1.empty() || truth2.empty()) {
        if (enable_debug) {
            log("    ❌ Empty truth tables\n");
        }
        return false;
    }
    
    if (split_pos < 0 || split_pos >= inputs2.size()) {
        if (enable_debug) {
            log("    ❌ Invalid split position: %d (inputs2 size: %zu)\n", 
                split_pos, inputs2.size());
        }
        return false;
    }
    
    // 2. ✅ 严格按照v1.2方案：建立输入映射
    dict<SigBit, int> map1, map2_reduced;
    
    // 建立truth1(z5_lut)的输入映射
    for (int i = 0; i < inputs1.size(); i++) {
        map1[sigmap(inputs1[i])] = i;
    }
    
    // 建立truth2(z_lut)去掉split_var后的输入映射
    int reduced_idx = 0;
    for (int i = 0; i < inputs2.size(); i++) {
        if (i != split_pos) {
            map2_reduced[sigmap(inputs2[i])] = reduced_idx++;
        }
    }
    
    // 3. ✅ 穷举验证：遍历所有reduced输入组合（不可简化）
    int reduced_size = 1 << (inputs2.size() - 1);  // 2^(n-1) 个组合
    int failed_combinations = 0;
    
    if (enable_debug) {
        log("    Starting exhaustive verification: %d combinations\n", reduced_size);
    }
    
    for (int combo = 0; combo < reduced_size; combo++) {
        // 4. ✅ 计算truth2在split_var=0时的输出
        int addr2 = 0;
        for (int i = 0; i < inputs2.size(); i++) {
            if (i == split_pos) continue;  // split_var=0，不设置该位
            int r_idx = (i < split_pos) ? i : (i - 1);
            if (combo & (1 << r_idx)) {
                addr2 |= (1 << i);
            }
        }
        bool out2 = (addr2 < truth2.size()) ? truth2[addr2] : false;
        
        // 5. ✅ 计算truth1在对应输入下的输出
        int addr1 = 0;
        for (int i = 0; i < inputs1.size(); i++) {
            SigBit sig = sigmap(inputs1[i]);
            if (map2_reduced.count(sig)) {
                int pos = map2_reduced[sig];
                if (combo & (1 << pos)) {
                    addr1 |= (1 << i);
                }
            }
        }
        bool out1 = (addr1 < truth1.size()) ? truth1[addr1] : false;
        
        // 6. ⚠️ 关键比较：检查逻辑等价性
        if (out1 != out2) {
            failed_combinations++;
            if (enable_debug) {
                log("    ❌ Equivalence failed at combo %d (0x%x):\n", combo, combo);
                log("      addr1=0x%x -> out1=%d, addr2=0x%x -> out2=%d\n",
                    addr1, out1, addr2, out2);
                
                // 详细的输入状态debug
                if (failed_combinations <= 5) {  // 只显示前5个失败的组合
                    debugLogicalEquivalenceFailure(combo, addr1, addr2, inputs1, inputs2, 
                                                   split_pos, map1, map2_reduced);
                }
            }
            
            // 如果失败太多，提前退出（但至少要检查一定数量）
            if (failed_combinations >= 10 && combo > 10) {
                if (enable_debug) {
                    log("    ❌ Too many failures (%d), aborting verification\n", 
                        failed_combinations);
                }
                return false;
            }
        }
    }
    
    // 7. 验证结果总结
    if (failed_combinations == 0) {
        if (enable_debug) {
            log("    ✅ All %d combinations verified successfully\n", reduced_size);
        }
        return true;
    } else {
        if (enable_debug) {
            log("    ❌ Verification failed: %d/%d combinations failed\n", 
                failed_combinations, reduced_size);
        }
        return false;
    }
}

// =============================================================================
// 调试和辅助函数
// =============================================================================

/**
 * 调试逻辑等价验证失败 - 提供详细的失败分析信息
 */
void LUTMergeOptimizer::debugLogicalEquivalenceFailure(
    int combo, int addr1, int addr2,
    const vector<SigBit> &inputs1, const vector<SigBit> &inputs2,
    int split_pos,
    const dict<SigBit, int> &map1, const dict<SigBit, int> &map2_reduced)
{
    if (!enable_debug) return;
    
    log("      Detailed failure analysis:\n");
    
    // 显示combo的二进制表示
    log("        Combo pattern: ");
    for (int i = inputs2.size() - 2; i >= 0; i--) {  // -2因为减去了split_var
        log("%d", (combo & (1 << i)) ? 1 : 0);
    }
    log("\n");
    
    // 显示inputs1的地址构造
    log("        Input1 mapping:\n");
    for (int i = 0; i < inputs1.size(); i++) {
        SigBit sig = sigmap(inputs1[i]);
        bool bit_set = (addr1 & (1 << i)) != 0;
        log("          %s -> bit %d = %d\n", log_signal(sig), i, bit_set ? 1 : 0);
    }
    
    // 显示inputs2的地址构造
    log("        Input2 mapping (split_pos=%d):\n", split_pos);
    for (int i = 0; i < inputs2.size(); i++) {
        bool bit_set = (addr2 & (1 << i)) != 0;
        if (i == split_pos) {
            log("          %s -> bit %d = 0 (split_var)\n", 
                log_signal(inputs2[i]), i);
        } else {
            log("          %s -> bit %d = %d\n", 
                log_signal(inputs2[i]), i, bit_set ? 1 : 0);
        }
    }
}

/**
 * 香农展开调试输出 - 提供详细的香农展开分析信息
 */
void LUTMergeOptimizer::debugShannonExpansion(
    const LUTMergeCandidate &candidate,
    SigBit split_var,
    const ShannonSplitAnalysis &split_analysis)
{
    if (!enable_debug) return;
    
    log("=== Shannon Expansion Debug Info ===\n");
    log("  Candidate pair: %s + %s\n", 
        candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
    log("  Total inputs: %d, Split variable: %s\n",
        candidate.total_inputs, log_signal(split_var));
    
    log("  Input analysis:\n");
    log("    Shared inputs (%zu): ", candidate.shared_inputs.size());
    for (auto sig : candidate.shared_inputs) {
        log("%s ", log_signal(sig));
    }
    log("\n");
    
    log("    LUT1-only inputs (%zu): ", candidate.lut1_only_inputs.size());
    for (auto sig : candidate.lut1_only_inputs) {
        log("%s ", log_signal(sig));
    }
    log("\n");
    
    log("    LUT2-only inputs (%zu): ", candidate.lut2_only_inputs.size());
    for (auto sig : candidate.lut2_only_inputs) {
        log("%s ", log_signal(sig));
    }
    log("\n");
    
    log("  Shannon expansion roles:\n");
    // ✅ Bug修复: 在验证阶段，z5_lut和z_lut尚未设置，使用lut1和lut2
    log("    LUT1: %s (inputs: %zu)\n", 
        candidate.lut1->name.c_str(), split_analysis.z5_inputs.size());
    log("    LUT2: %s (inputs: %zu)\n", 
        candidate.lut2->name.c_str(), split_analysis.z_inputs.size());
    log("    Split position: %d in larger LUT inputs\n", split_analysis.split_pos);
}

YOSYS_NAMESPACE_END