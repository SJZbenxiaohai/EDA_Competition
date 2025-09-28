/*
 * GTP_LUT6D合并类型精确判断模块
 * 
 * 作用: 实现7种MergeType的精确识别逻辑，为LUT合并优化提供类型判断支持
 * 文件: techlibs/pango/lut_merge_types.cc
 * 创建时间: 2025-09-26
 * 任务: Phase 3.1 - P3.1
 * 依据方案: 17-GTP_LUT6D修正执行方案.md Priority 3.1
 * 预计行数: ~600行
 * 
 * 核心功能:
 * 1. checkLogicContainment() - 逻辑包含关系检查
 * 2. checkInputSubsetRelation() - 输入子集关系检查  
 * 3. checkPartialSharingFeasibility() - 部分共享可行性检查
 * 4. findOptimalSplitVariable() - 最优分割变量选择
 * 5. checkBasicMergeConstraints() - 基础合并约束检查
 * 6. determineMergeType() - 完整的合并类型判断框架
 * 
 * 重要: 严格按照v1.2方案的7种MergeType优先级实现
 */

#include "lut_merge_pango.h"
#include "kernel/log.h"
#include <algorithm>

YOSYS_NAMESPACE_BEGIN

// =============================================================================
// 基础约束检查函数
// =============================================================================

/**
 * 基础合并约束检查 - 验证候选是否满足合并的基本条件
 * 
 * @param candidate 合并候选对象
 * @return true 如果满足基础约束，false 否则
 */
bool LUTMergeOptimizer::checkBasicMergeConstraints(LUTMergeCandidate &candidate)
{
    // 1. 检查LUT指针有效性
    if (!candidate.lut1 || !candidate.lut2) {
        candidate.failure_reason = "Invalid LUT pointers";
        return false;
    }
    
    // 2. 检查是否是相同的LUT（自合并检查）
    if (candidate.lut1 == candidate.lut2) {
        candidate.failure_reason = "Cannot merge LUT with itself";
        return false;
    }
    
    // 3. 检查是否都是单输出LUT
    if (!isSingleOutputLUT(candidate.lut1) || !isSingleOutputLUT(candidate.lut2)) {
        candidate.failure_reason = "Only single-output LUTs can be merged";
        return false;
    }
    
    // 4. 检查总输入数量约束 (GTP_LUT6D最多6输入)
    if (candidate.total_inputs > 6) {
        candidate.failure_reason = stringf("Total inputs %d exceeds GTP_LUT6D limit (6)", 
                                          candidate.total_inputs);
        return false;
    }
    
    // 5. 检查输入数量一致性
    int expected_total = candidate.shared_inputs.size() + 
                        candidate.lut1_only_inputs.size() + 
                        candidate.lut2_only_inputs.size();
    if (expected_total != candidate.total_inputs) {
        candidate.failure_reason = stringf("Input count inconsistency: expected %d, got %d", 
                                          expected_total, candidate.total_inputs);
        return false;
    }
    
    // 6. 基础LUT类型检查
    auto lut1_type = candidate.lut1->type;
    auto lut2_type = candidate.lut2->type;
    
    if (!lut1_type.in(ID(GTP_LUT1), ID(GTP_LUT2), ID(GTP_LUT3), 
                      ID(GTP_LUT4), ID(GTP_LUT5), ID(GTP_LUT6))) {
        candidate.failure_reason = stringf("LUT1 type %s is not a valid GTP_LUT", lut1_type.c_str());
        return false;
    }
    
    if (!lut2_type.in(ID(GTP_LUT1), ID(GTP_LUT2), ID(GTP_LUT3), 
                      ID(GTP_LUT4), ID(GTP_LUT5), ID(GTP_LUT6))) {
        candidate.failure_reason = stringf("LUT2 type %s is not a valid GTP_LUT", lut2_type.c_str());
        return false;
    }
    
    return true;
}

// =============================================================================
// 逻辑包含关系检查 (最高优先级)
// =============================================================================

/**
 * 逻辑包含关系检查 - 检查一个LUT的逻辑是否完全包含另一个
 * 
 * 逻辑包含定义: 如果对于所有可能的输入组合，一个LUT的输出总是能由另一个LUT推导出来，
 * 则称存在逻辑包含关系。这是最高优先级的合并类型。
 * 
 * @param candidate 合并候选对象
 * @return true 如果存在逻辑包含关系，false 否则
 */
bool LUTMergeOptimizer::checkLogicContainment(LUTMergeCandidate &candidate)
{
    // 获取两个LUT的真值表
    vector<bool> lut1_truth = extractLUTTruthTable(candidate.lut1);
    vector<bool> lut2_truth = extractLUTTruthTable(candidate.lut2);
    
    if (lut1_truth.empty() || lut2_truth.empty()) {
        candidate.failure_reason = "Cannot extract truth tables for logic containment check";
        return false;
    }
    
    // 获取输入信号向量
    vector<SigBit> lut1_inputs, lut2_inputs;
    getCellInputsVector(candidate.lut1, lut1_inputs);
    getCellInputsVector(candidate.lut2, lut2_inputs);
    
    // 检查输入子集关系是否存在（逻辑包含的先决条件）
    bool lut1_inputs_subset_of_lut2 = candidate.lut1_only_inputs.empty();
    bool lut2_inputs_subset_of_lut1 = candidate.lut2_only_inputs.empty();
    
    if (!lut1_inputs_subset_of_lut2 && !lut2_inputs_subset_of_lut1) {
        // 没有输入子集关系，不可能有逻辑包含
        return false;
    }
    
    // 情况1: LUT1的输入是LUT2输入的子集，检查LUT1是否逻辑包含于LUT2
    if (lut1_inputs_subset_of_lut2) {
        if (checkLogicalContainmentCore(lut1_truth, lut2_truth, lut1_inputs, lut2_inputs, false)) {
            candidate.merge_strategy = "LUT1 logically contained in LUT2";
            candidate.z5_lut = candidate.lut1;  // 包含的LUT作为Z5
            candidate.z_lut = candidate.lut2;   // 包含的LUT作为Z
            return true;
        }
    }
    
    // 情况2: LUT2的输入是LUT1输入的子集，检查LUT2是否逻辑包含于LUT1
    if (lut2_inputs_subset_of_lut1) {
        if (checkLogicalContainmentCore(lut2_truth, lut1_truth, lut2_inputs, lut1_inputs, true)) {
            candidate.merge_strategy = "LUT2 logically contained in LUT1";
            candidate.z5_lut = candidate.lut2;  // 包含的LUT作为Z5
            candidate.z_lut = candidate.lut1;   // 包含的LUT作为Z
            return true;
        }
    }
    
    return false;
}

/**
 * 逻辑包含关系核心检查算法
 * 
 * @param contained_truth 被包含LUT的真值表
 * @param container_truth 包含LUT的真值表  
 * @param contained_inputs 被包含LUT的输入
 * @param container_inputs 包含LUT的输入
 * @param reverse_role 是否需要反转LUT1和LUT2的角色
 * @return true 如果存在逻辑包含关系
 */
bool LUTMergeOptimizer::checkLogicalContainmentCore(
    const vector<bool> &contained_truth,
    const vector<bool> &container_truth,
    const vector<SigBit> &contained_inputs,
    const vector<SigBit> &container_inputs,
    bool reverse_role)
{
    // 建立输入映射：contained_inputs到container_inputs的映射
    dict<SigBit, int> contained_pos, container_pos;
    for (int i = 0; i < contained_inputs.size(); i++) {
        contained_pos[contained_inputs[i]] = i;
    }
    for (int i = 0; i < container_inputs.size(); i++) {
        container_pos[container_inputs[i]] = i;
    }
    
    // 遍历被包含LUT的所有输入组合
    int contained_combinations = 1 << contained_inputs.size();
    
    for (int contained_addr = 0; contained_addr < contained_combinations; contained_addr++) {
        // 计算被包含LUT在这个地址的输出
        bool contained_output = (contained_addr < contained_truth.size()) ? 
                               contained_truth[contained_addr] : false;
        
        // 将contained地址映射到container地址空间
        int container_addr = 0;
        for (int i = 0; i < contained_inputs.size(); i++) {
            if (contained_addr & (1 << i)) {
                SigBit sig = contained_inputs[i];
                if (container_pos.count(sig)) {
                    int container_bit = container_pos[sig];
                    container_addr |= (1 << container_bit);
                }
            }
        }
        
        // 计算包含LUT在对应地址的输出
        bool container_output = (container_addr < container_truth.size()) ? 
                               container_truth[container_addr] : false;
        
        // 检查逻辑包含条件：被包含LUT输出为1时，包含LUT输出也必须为1
        if (contained_output && !container_output) {
            if (enable_debug) {
                log("  Logic containment failed at contained_addr=%d, container_addr=%d: "
                    "contained=%d, container=%d\n",
                    contained_addr, container_addr, contained_output, container_output);
            }
            return false;
        }
    }
    
    return true;
}

// =============================================================================
// 输入子集关系检查 (中等优先级)
// =============================================================================

/**
 * 输入子集关系检查 - 检查一个LUT的输入是否是另一个的严格子集
 * 
 * @param candidate 合并候选对象
 * @return true 如果存在输入子集关系，false 否则
 */
bool LUTMergeOptimizer::checkInputSubsetRelation(LUTMergeCandidate &candidate)
{
    if (candidate.lut1_only_inputs.empty()) {
        // LUT1的输入是LUT2输入的子集
        if (!candidate.lut2_only_inputs.empty()) {
            candidate.merge_strategy = "LUT1 inputs are strict subset of LUT2";
            candidate.z5_lut = candidate.lut1;  // 子集LUT作为Z5
            candidate.z_lut = candidate.lut2;   // 超集LUT作为Z
            return true;
        }
    }
    
    if (candidate.lut2_only_inputs.empty()) {
        // LUT2的输入是LUT1输入的子集
        if (!candidate.lut1_only_inputs.empty()) {
            candidate.merge_strategy = "LUT2 inputs are strict subset of LUT1";
            candidate.z5_lut = candidate.lut2;  // 子集LUT作为Z5
            candidate.z_lut = candidate.lut1;   // 超集LUT作为Z
            return true;
        }
    }
    
    // 如果两个LUT的输入完全相同，不算子集关系
    if (candidate.lut1_only_inputs.empty() && candidate.lut2_only_inputs.empty()) {
        candidate.failure_reason = "LUTs have identical inputs, not a subset relation";
        return false;
    }
    
    return false;
}

// =============================================================================
// 部分共享可行性检查 (中低优先级)
// =============================================================================

/**
 * 部分共享可行性检查 - 检查5输入部分共享合并的可行性
 * 
 * @param candidate 合并候选对象
 * @return true 如果可以进行部分共享合并，false 否则
 */
bool LUTMergeOptimizer::checkPartialSharingFeasibility(LUTMergeCandidate &candidate)
{
    // 必须满足总输入数<=5的约束
    if (candidate.total_inputs > 5) {
        candidate.failure_reason = "Total inputs exceed 5 for partial sharing";
        return false;
    }
    
    // 必须有共享输入
    if (candidate.shared_inputs.empty()) {
        candidate.failure_reason = "No shared inputs for partial sharing";
        return false;
    }
    
    // 两个LUT都必须有独有输入（否则是子集关系，不是部分共享）
    if (candidate.lut1_only_inputs.empty() || candidate.lut2_only_inputs.empty()) {
        candidate.failure_reason = "Partial sharing requires both LUTs to have unique inputs";
        return false;
    }
    
    // 检查共享输入比例（可选的质量检查）
    int shared_count = candidate.shared_inputs.size();
    int lut1_total_inputs = shared_count + candidate.lut1_only_inputs.size();
    int lut2_total_inputs = shared_count + candidate.lut2_only_inputs.size();
    
    float lut1_sharing_ratio = (float)shared_count / lut1_total_inputs;
    float lut2_sharing_ratio = (float)shared_count / lut2_total_inputs;
    
    // 共享比例太低的话合并收益可能不高
    if (lut1_sharing_ratio < 0.3 && lut2_sharing_ratio < 0.3) {
        if (enable_debug) {
            log("  Warning: Low input sharing ratio (LUT1: %.1f%%, LUT2: %.1f%%)\n",
                lut1_sharing_ratio * 100, lut2_sharing_ratio * 100);
        }
        // 不阻止合并，但给出警告
    }
    
    candidate.merge_strategy = stringf("5-input partial sharing (%d shared, %zu unique)",
                                      shared_count, 
                                      candidate.lut1_only_inputs.size() + candidate.lut2_only_inputs.size());
    
    // 对于部分共享，两个LUT地位相等，按输入数量选择z5_lut和z_lut
    if (lut1_total_inputs <= lut2_total_inputs) {
        candidate.z5_lut = candidate.lut1;
        candidate.z_lut = candidate.lut2;
    } else {
        candidate.z5_lut = candidate.lut2;
        candidate.z_lut = candidate.lut1;
    }
    
    return true;
}

// =============================================================================
// 最优分割变量选择 (用于香农展开)
// =============================================================================

/**
 * 寻找最优分割变量 - 为6输入香农展开选择最佳的分割变量
 * 
 * 选择策略:
 * 1. 优先选择只属于其中一个LUT的输入（非共享输入）
 * 2. 在非共享输入中，优先选择使香农展开更平衡的变量
 * 3. 如果只有共享输入，选择对逻辑影响最小的变量
 * 
 * @param candidate 合并候选对象
 * @return 最优的分割变量，SigBit() 如果无法找到
 */
SigBit LUTMergeOptimizer::findOptimalSplitVariable(const LUTMergeCandidate &candidate)
{
    if (candidate.total_inputs != 6) {
        if (enable_debug) {
            log("  findOptimalSplitVariable called for %d-input case (should be 6)\n", 
                candidate.total_inputs);
        }
        return SigBit();
    }
    
    // 策略1: 优先选择非共享输入
    
    // 首先尝试选择LUT2的独有输入（使LUT1成为Z5的子函数）
    if (!candidate.lut2_only_inputs.empty()) {
        SigBit best_split = selectBestSplitFromCandidates(candidate, candidate.lut2_only_inputs, false);
        if (best_split != SigBit()) {
            if (enable_debug) {
                log("  Selected split variable from LUT2-only inputs: %s\n", log_signal(best_split));
            }
            return best_split;
        }
    }
    
    // 然后尝试选择LUT1的独有输入
    if (!candidate.lut1_only_inputs.empty()) {
        SigBit best_split = selectBestSplitFromCandidates(candidate, candidate.lut1_only_inputs, true);
        if (best_split != SigBit()) {
            if (enable_debug) {
                log("  Selected split variable from LUT1-only inputs: %s\n", log_signal(best_split));
            }
            return best_split;
        }
    }
    
    // 策略2: 如果只有共享输入可选，选择对逻辑分割最均匀的
    if (!candidate.shared_inputs.empty()) {
        SigBit best_split = selectBestSplitFromCandidates(candidate, candidate.shared_inputs, false);
        if (best_split != SigBit()) {
            if (enable_debug) {
                log("  Selected split variable from shared inputs: %s\n", log_signal(best_split));
            }
            return best_split;
        }
    }
    
    // 无法找到合适的分割变量
    if (enable_debug) {
        log("  No suitable split variable found for 6-input Shannon expansion\n");
    }
    return SigBit();
}

/**
 * 从候选集合中选择最佳分割变量
 * 
 * @param candidate 合并候选对象
 * @param candidates 候选分割变量集合
 * @param prefer_lut1 是否优先考虑有利于LUT1作为子函数的分割
 * @return 最佳分割变量，SigBit() 如果无法选择
 */
SigBit LUTMergeOptimizer::selectBestSplitFromCandidates(
    const LUTMergeCandidate &candidate,
    const pool<SigBit> &candidates, 
    bool prefer_lut1)
{
    if (candidates.empty()) return SigBit();
    
    // 简单策略：选择第一个候选变量
    // 在更复杂的实现中，可以基于香农展开的均匀性来选择
    SigBit first_candidate = *candidates.begin();
    
    // 进行基本的有效性检查
    if (first_candidate.wire == nullptr) {
        if (enable_debug) {
            log("  Warning: Selected split variable has null wire\n");
        }
        return SigBit();
    }
    
    return first_candidate;
}

// =============================================================================
// 完整的合并类型判断框架
// =============================================================================

/**
 * 完整的合并类型判断框架 - 按v1.2方案的优先级顺序判断合并类型
 * 
 * 优先级顺序 (从高到低):
 * 1. LOGIC_CONTAINMENT - 逻辑包含关系
 * 2. SIX_INPUT_SHANNON - 6输入香农展开  
 * 3. INPUT_SUBSET - 输入子集关系
 * 4. PARTIAL_SHARING_5INPUT - 5输入部分共享
 * 5. INDEPENDENT_REUSE - 独立功能复用
 * 6. FUNCTION_MULTIPLEXING - 功能复用
 * 
 * @param candidate 合并候选对象
 * @return 识别出的合并类型，INVALID 如果无法合并
 */
MergeType LUTMergeOptimizer::determineMergeType(LUTMergeCandidate &candidate)
{
    if (enable_debug) {
        log("=== Determining merge type for LUT pair ===\n");
        log("  LUT1: %s (type: %s)\n", candidate.lut1->name.c_str(), candidate.lut1->type.c_str());
        log("  LUT2: %s (type: %s)\n", candidate.lut2->name.c_str(), candidate.lut2->type.c_str());
        log("  Total inputs: %d, Shared: %zu, LUT1-only: %zu, LUT2-only: %zu\n",
            candidate.total_inputs, candidate.shared_inputs.size(),
            candidate.lut1_only_inputs.size(), candidate.lut2_only_inputs.size());
    }
    
    // 1. 基础约束预检查
    if (!checkBasicMergeConstraints(candidate)) {
        if (enable_debug) {
            log("  Basic constraints failed: %s\n", candidate.failure_reason.c_str());
        }
        return MergeType::INVALID;
    }
    
    // 2. 按v1.2方案的优先级顺序进行精确判断
    
    // 最高优先级: 逻辑包含关系 (一个LUT的逻辑完全包含另一个)
    if (checkLogicContainment(candidate)) {
        if (enable_debug) {
            log("  ✅ LOGIC_CONTAINMENT: %s\n", candidate.merge_strategy.c_str());
        }
        return MergeType::LOGIC_CONTAINMENT;
    }
    
    // 次高优先级: 6输入香农展开 (总输入=6，且满足香农展开条件)
    if (candidate.total_inputs == 6) {
        SigBit split_var = findOptimalSplitVariable(candidate);
        if (split_var != SigBit()) {
            // ✅ 调用完整的香农展开验证
            if (verifyShannonExpansion(candidate, split_var)) {
                candidate.split_variable = split_var;
                candidate.merge_strategy = "6-input Shannon expansion verified";
                
                // ✅ Bug修复: 在验证成功后设置z5_lut和z_lut
                // 根据香农展开逻辑，z5_lut应该是5输入函数，z_lut是6输入函数
                if (candidate.lut1_only_inputs.size() < candidate.lut2_only_inputs.size()) {
                    candidate.z5_lut = candidate.lut1;  // 输入较少的LUT作为Z5
                    candidate.z_lut = candidate.lut2;   // 输入较多的LUT作为Z
                } else {
                    candidate.z5_lut = candidate.lut2;  // 输入较少的LUT作为Z5
                    candidate.z_lut = candidate.lut1;   // 输入较多的LUT作为Z
                }
                
                if (enable_debug) {
                    log("  ✅ SIX_INPUT_SHANNON verified: split_var=%s\n", log_signal(split_var));
                    log("    Z5_LUT: %s, Z_LUT: %s\n", 
                        candidate.z5_lut->name.c_str(), candidate.z_lut->name.c_str());
                }
                
                return MergeType::SIX_INPUT_SHANNON;
            } else {
                if (enable_debug) {
                    log("  ❌ 6-input Shannon expansion verification failed\n");
                }
            }
        } else {
            if (enable_debug) {
                log("  ❌ 6-input Shannon expansion: no suitable split variable\n");
            }
        }
    }
    
    // 中等优先级: 输入子集关系 (一个LUT的输入是另一个的子集)
    if (checkInputSubsetRelation(candidate)) {
        if (enable_debug) {
            log("  ✅ INPUT_SUBSET: %s\n", candidate.merge_strategy.c_str());
        }
        return MergeType::INPUT_SUBSET;
    }
    
    // 中低优先级: 5输入部分共享
    if (candidate.total_inputs <= 5 && !candidate.shared_inputs.empty()) {
        if (checkPartialSharingFeasibility(candidate)) {
            if (enable_debug) {
                log("  ✅ PARTIAL_SHARING_5INPUT: %s\n", candidate.merge_strategy.c_str());
            }
            return MergeType::PARTIAL_SHARING_5INPUT;
        }
    }
    
    // 低优先级: 独立功能复用 (总输入数较少，可以作为独立函数复用)
    if (candidate.total_inputs <= 4) {
        candidate.merge_strategy = stringf("Independent reuse (%d inputs, low complexity)", 
                                          candidate.total_inputs);
        if (enable_debug) {
            log("  ✅ INDEPENDENT_REUSE: %s\n", candidate.merge_strategy.c_str());
        }
        
        // 为独立复用选择合适的z5和z角色
        int lut1_inputs = candidate.shared_inputs.size() + candidate.lut1_only_inputs.size();
        int lut2_inputs = candidate.shared_inputs.size() + candidate.lut2_only_inputs.size();
        
        if (lut1_inputs <= lut2_inputs) {
            candidate.z5_lut = candidate.lut1;
            candidate.z_lut = candidate.lut2;
        } else {
            candidate.z5_lut = candidate.lut2;
            candidate.z_lut = candidate.lut1;
        }
        
        return MergeType::INDEPENDENT_REUSE;
    }
    
    // 最低优先级: 功能复用 (5输入以内的其他情况)
    if (candidate.total_inputs <= 5) {
        candidate.merge_strategy = stringf("Function multiplexing fallback (%d inputs)", 
                                          candidate.total_inputs);
        if (enable_debug) {
            log("  ✅ FUNCTION_MULTIPLEXING: %s\n", candidate.merge_strategy.c_str());
        }
        
        // 为功能复用选择合适的z5和z角色
        int lut1_inputs = candidate.shared_inputs.size() + candidate.lut1_only_inputs.size();
        int lut2_inputs = candidate.shared_inputs.size() + candidate.lut2_only_inputs.size();
        
        if (lut1_inputs <= lut2_inputs) {
            candidate.z5_lut = candidate.lut1;
            candidate.z_lut = candidate.lut2;
        } else {
            candidate.z5_lut = candidate.lut2;
            candidate.z_lut = candidate.lut1;
        }
        
        return MergeType::FUNCTION_MULTIPLEXING;
    }
    
    // 无法合并
    candidate.failure_reason = stringf("No valid merge type identified for %d-input case", 
                                      candidate.total_inputs);
    if (enable_debug) {
        log("  ❌ INVALID: %s\n", candidate.failure_reason.c_str());
    }
    
    return MergeType::INVALID;
}

YOSYS_NAMESPACE_END