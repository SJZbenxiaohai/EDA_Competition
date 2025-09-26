/*
 * GTP_LUT6D合并执行器模块
 * 
 * 作用: LUT合并执行器，负责执行具体的合并操作和最优匹配选择
 * 文件: techlibs/pango/lut_merge_executor.cc
 * 创建时间: 2025-09-26
 * 任务: Phase 3.4 - P3.4
 * 依据方案: 17-GTP_LUT6D修正执行方案.md Priority 3.4
 * 预计行数: ~600行
 * 
 * 核心功能:
 * 1. executeSingleMerge() - 执行单个LUT合并操作
 * 2. selectOptimalMatching() - 最优匹配选择算法
 * 3. createGTP_LUT6D() - 创建GTP_LUT6D硬件实例
 * 4. updateModuleConnections() - 更新模块连接
 * 5. cleanupOriginalLUTs() - 清理原始LUT
 * 
 * ⚠️ 风险级别: 🟠 高风险 
 * 硬件实例化和连接错误会导致综合失败
 * 必须确保信号连接的完整性和正确性
 */

#include "lut_merge_pango.h"
#include "kernel/log.h"

YOSYS_NAMESPACE_BEGIN

// =============================================================================
// 最优匹配选择算法
// =============================================================================

/**
 * 从候选列表中选择最优匹配
 * 
 * 策略：
 * 1. 按照收益评分排序
 * 2. 避免LUT冲突（同一个LUT不能参与多个合并）
 * 3. 优先选择高优先级的合并类型
 * 4. 确保时序约束满足
 * 
 * @param candidates 候选合并列表
 * @return 选择的最优合并列表
 */
vector<LUTMergeCandidate> LUTMergeOptimizer::selectOptimalMatching(
    const vector<LUTMergeCandidate> &candidates) 
{
    vector<LUTMergeCandidate> selected;
    
    if (enable_debug) {
        log("=== Optimal Matching Selection ===\n");
        log("  Total candidates: %zu\n", candidates.size());
    }
    
    // 1. 过滤有效候选
    vector<LUTMergeCandidate> valid_candidates;
    for (const auto &candidate : candidates) {
        if (candidate.merge_type != MergeType::INVALID && 
            candidate.total_benefit > benefit_threshold) {
            valid_candidates.push_back(candidate);
        }
    }
    
    if (valid_candidates.empty()) {
        if (enable_debug) {
            log("  No valid candidates found\n");
        }
        return selected;
    }
    
    // 2. 按收益评分排序（降序）
    sort(valid_candidates.begin(), valid_candidates.end(),
         [](const LUTMergeCandidate &a, const LUTMergeCandidate &b) {
             // 首先按合并类型优先级排序
             float priority_a = getMergeTypePriority(a.merge_type);
             float priority_b = getMergeTypePriority(b.merge_type);
             if (abs(priority_a - priority_b) > 0.1) {
                 return priority_a > priority_b;
             }
             // 然后按收益排序
             return a.total_benefit > b.total_benefit;
         });
    
    // 3. 贪心选择，避免LUT冲突
    pool<Cell*> used_luts;
    
    for (const auto &candidate : valid_candidates) {
        // 检查LUT冲突
        if (used_luts.count(candidate.lut1) || used_luts.count(candidate.lut2)) {
            if (enable_debug) {
                log("  Skipping candidate due to LUT conflict: %s + %s\n",
                    candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
            }
            continue;
        }
        
        // 时序约束检查
        if (strategy == CONSERVATIVE && candidate.timing_impact > 0.1) {
            if (enable_debug) {
                log("  Skipping candidate due to timing impact: %.3f\n", 
                    candidate.timing_impact);
            }
            continue;
        }
        
        // 选择该候选
        selected.push_back(candidate);
        used_luts.insert(candidate.lut1);
        used_luts.insert(candidate.lut2);
        
        if (enable_debug) {
            log("  Selected: %s + %s (benefit=%.3f, type=%s)\n",
                candidate.lut1->name.c_str(), candidate.lut2->name.c_str(),
                candidate.total_benefit, getMergeTypeString(candidate.merge_type).c_str());
        }
        
        // 限制单次迭代的合并数量
        if (selected.size() >= 10) {  // 避免过多合并导致的复杂度
            break;
        }
    }
    
    if (enable_debug) {
        log("  Final selection: %zu merges\n", selected.size());
    }
    
    return selected;
}

// =============================================================================
// 单个合并执行
// =============================================================================

/**
 * 执行单个LUT合并操作
 * 
 * 步骤：
 * 1. 验证合并候选的有效性
 * 2. 计算合并后的INIT值
 * 3. 创建GTP_LUT6D实例
 * 4. 更新信号连接
 * 5. 清理原始LUT
 * 
 * @param candidate 合并候选对象
 * @return 合并是否成功
 */
bool LUTMergeOptimizer::executeSingleMerge(const LUTMergeCandidate &candidate) 
{
    if (enable_debug) {
        log("=== Executing Single Merge ===\n");
        log("  LUT1: %s, LUT2: %s\n", 
            candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
        log("  Merge type: %s\n", getMergeTypeString(candidate.merge_type).c_str());
        log("  Total benefit: %.3f\n", candidate.total_benefit);
    }
    
    // 1. 最终验证
    if (!candidate.lut1 || !candidate.lut2) {
        log_error("Invalid LUT pointers in merge candidate\n");
        return false;
    }
    
    if (candidate.merge_type == MergeType::INVALID) {
        log_error("Invalid merge type for candidate\n");
        return false;
    }
    
    // 2. 确定输入引脚顺序
    vector<SigBit> input_order = arrangeInputPins(candidate);
    if (input_order.size() > 6) {
        log_error("Too many inputs for GTP_LUT6D: %zu\n", input_order.size());
        return false;
    }
    
    // 3. 计算INIT值
    vector<bool> init_value = computeGTP_LUT6D_INIT(candidate, input_order);
    if (init_value.size() != 64) {
        log_error("Invalid INIT value size: %zu\n", init_value.size());
        return false;
    }
    
    // 4. 创建GTP_LUT6D实例
    Cell *merged_lut = createGTP_LUT6D(candidate, input_order, init_value);
    if (!merged_lut) {
        log_error("Failed to create GTP_LUT6D instance\n");
        return false;
    }
    
    // 5. 更新信号连接
    if (!updateMergedConnections(candidate, merged_lut, input_order)) {
        log_error("Failed to update merged connections\n");
        current_module->remove(merged_lut);
        return false;
    }
    
    // 6. 清理原始LUT
    if (!cleanupOriginalLUTs(candidate)) {
        log_error("Failed to cleanup original LUTs\n");
        return false;
    }
    
    if (enable_debug) {
        log("  Merge completed successfully: %s\n", merged_lut->name.c_str());
    }
    
    return true;
}

// =============================================================================
// GTP_LUT6D硬件实例创建
// =============================================================================

/**
 * 创建GTP_LUT6D硬件实例
 * 
 * @param candidate 合并候选对象
 * @param input_order 输入引脚顺序
 * @param init_value 64位INIT值
 * @return 创建的GTP_LUT6D实例
 */
Cell* LUTMergeOptimizer::createGTP_LUT6D(
    const LUTMergeCandidate &candidate,
    const vector<SigBit> &input_order,
    const vector<bool> &init_value)
{
    if (!current_module) {
        log_error("No current module for GTP_LUT6D creation\n");
        return nullptr;
    }
    
    // 生成唯一的LUT名称
    string merged_name = stringf("\\merged_lut_%s_%s", 
                                candidate.lut1->name.c_str() + 1,  // 跳过反斜杠
                                candidate.lut2->name.c_str() + 1);
    
    // 创建GTP_LUT6D实例
    Cell *merged_lut = current_module->addCell(merged_name, ID(GTP_LUT6D));
    if (!merged_lut) {
        log_error("Failed to add GTP_LUT6D cell: %s\n", merged_name.c_str());
        return nullptr;
    }
    
    // 设置INIT参数
    string init_str = formatInitValue(init_value);
    merged_lut->setParam(ID(INIT), Const::from_string(init_str));
    
    // 连接输入引脚 I0-I5
    for (size_t i = 0; i < input_order.size() && i < 6; i++) {
        string port_name = stringf("I%zu", i);
        merged_lut->setPort(RTLIL::escape_id(port_name), input_order[i]);
    }
    
    // 未使用的输入引脚连接到常量0
    for (size_t i = input_order.size(); i < 6; i++) {
        string port_name = stringf("I%zu", i);
        merged_lut->setPort(RTLIL::escape_id(port_name), State::S0);
    }
    
    // 输出引脚将在updateMergedConnections中连接
    
    if (enable_debug) {
        log("  Created GTP_LUT6D: %s\n", merged_name.c_str());
        log("  INIT: %s\n", init_str.c_str());
        log("  Inputs: %zu\n", input_order.size());
    }
    
    return merged_lut;
}

// =============================================================================
// 信号连接更新
// =============================================================================

/**
 * 更新合并后的信号连接
 * 
 * @param candidate 合并候选对象
 * @param merged_lut 合并后的LUT
 * @param input_order 输入顺序
 * @return 连接更新是否成功
 */
bool LUTMergeOptimizer::updateMergedConnections(
    const LUTMergeCandidate &candidate,
    Cell *merged_lut,
    const vector<SigBit> &input_order)
{
    if (!merged_lut) {
        return false;
    }
    
    // 获取原始LUT的输出信号
    SigBit output1 = getCellOutput(candidate.lut1);
    SigBit output2 = getCellOutput(candidate.lut2);
    
    if (!output1.wire || !output2.wire) {
        log_error("Invalid output signals from original LUTs\n");
        return false;
    }
    
    // 根据合并类型确定输出连接策略
    SigBit z_output, z5_output;
    
    switch (candidate.merge_type) {
        case MergeType::SIX_INPUT_SHANNON:
        case MergeType::SIX_INPUT_SHANNON_REVERSE:
            // 香农展开：Z输出连接到完整函数，Z5输出连接到简化函数
            z_output = output1;   // 完整的6输入函数
            z5_output = output2;  // 简化的5输入函数
            break;
            
        case MergeType::LOGIC_CONTAINMENT:
            // 逻辑包含：根据包含关系分配输出
            if (candidate.z_lut == candidate.lut1) {
                z_output = output1;
                z5_output = output2;
            } else {
                z_output = output2;
                z5_output = output1;
            }
            break;
            
        case MergeType::INPUT_SUBSET:
            // 输入子集：超集LUT连接Z，子集LUT连接Z5
            if (candidate.lut1_only_inputs.empty()) {
                z5_output = output1;  // LUT1是子集
                z_output = output2;   // LUT2是超集
            } else {
                z5_output = output2;  // LUT2是子集
                z_output = output1;   // LUT1是超集
            }
            break;
            
        default:
            // 其他类型：简单分配
            z_output = output1;
            z5_output = output2;
            break;
    }
    
    // 连接输出引脚
    merged_lut->setPort(ID(Z), z_output);
    merged_lut->setPort(ID(Z5), z5_output);
    
    if (enable_debug) {
        log("  Connected outputs: Z=%s, Z5=%s\n", 
            log_signal(z_output), log_signal(z5_output));
    }
    
    return true;
}

// =============================================================================
// 原始LUT清理
// =============================================================================

/**
 * 清理原始LUT
 * 
 * @param candidate 合并候选对象
 * @return 清理是否成功
 */
bool LUTMergeOptimizer::cleanupOriginalLUTs(const LUTMergeCandidate &candidate)
{
    if (!current_module) {
        return false;
    }
    
    if (enable_debug) {
        log("  Removing original LUTs: %s, %s\n", 
            candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
    }
    
    // 移除原始LUT
    current_module->remove(candidate.lut1);
    current_module->remove(candidate.lut2);
    
    return true;
}

// =============================================================================
// 辅助函数
// =============================================================================

/**
 * 格式化INIT值为字符串
 * 
 * @param init 64位INIT值
 * @return INIT字符串表示
 */
string formatInitValue(const vector<bool> &init) 
{
    if (init.size() != 64) {
        return "0";
    }
    
    string result = "64'h";
    
    // 转换为十六进制字符串
    for (int i = 60; i >= 0; i -= 4) {
        int nibble = 0;
        for (int j = 0; j < 4; j++) {
            if (i + j < 64 && init[i + j]) {
                nibble |= (1 << j);
            }
        }
        result += stringf("%x", nibble);
    }
    
    return result;
}

/**
 * 获取合并类型优先级
 * 
 * @param type 合并类型
 * @return 优先级值（越大优先级越高）
 */
float getMergeTypePriority(MergeType type) 
{
    switch (type) {
        case MergeType::LOGIC_CONTAINMENT:        return 5.0f;  // 最高优先级
        case MergeType::SIX_INPUT_SHANNON:        return 4.0f;
        case MergeType::SIX_INPUT_SHANNON_REVERSE: return 4.0f;
        case MergeType::INPUT_SUBSET:             return 3.0f;
        case MergeType::PARTIAL_SHARING_5INPUT:   return 2.0f;
        case MergeType::INDEPENDENT_REUSE:        return 1.0f;
        case MergeType::FUNCTION_MULTIPLEXING:    return 1.0f;  // 最低优先级
        default:                                  return 0.0f;
    }
}

YOSYS_NAMESPACE_END