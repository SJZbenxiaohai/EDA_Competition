/*
 * lut_merge_analyzer.cc - GTP_LUT6D合并分析器专用功能
 * 
 * 文件作用：实现LUT合并候选的详细分析功能
 * 创建日期：2025-09-25
 * 版本：v1.2 - 修复重复定义问题
 * 
 * 主要功能：
 * 1. analyzeInputRelationships() - 输入关系分析
 * 2. checkBasicMergeConstraints() - 基础约束检查  
 * 3. evaluateTimingImpact() - 时序影响评估
 * 
 * 注意：
 * - 主框架函数(identifyMergeCandidates, analyzeMergeCandidate等)在lut_merge_optimizer.cc中实现
 * - 此文件只包含分析器特有的辅助功能，避免重复定义
 */

#include "lut_merge_pango.h"
#include "kernel/log.h"
#include "kernel/utils.h"
#include <algorithm>
#include <set>

using std::set;

YOSYS_NAMESPACE_BEGIN

// analyzeInputRelationships() - 分析两个LUT的输入关系
// 严格按照v1.2方案中的输入分析逻辑
bool LUTMergeOptimizer::analyzeInputRelationships(const vector<SigBit> &lut1_inputs,
                                                 const vector<SigBit> &lut2_inputs,
                                                 LUTMergeCandidate &candidate)
{
    candidate.shared_inputs.clear();
    candidate.lut1_only_inputs.clear();
    candidate.lut2_only_inputs.clear();
    
    // 使用sigmap标准化信号
    set<SigBit> lut1_set, lut2_set;
    for (auto input : lut1_inputs) {
        lut1_set.insert(sigmap(input));
    }
    for (auto input : lut2_inputs) {
        lut2_set.insert(sigmap(input));
    }
    
    // 识别共享输入
    for (auto input : lut1_set) {
        if (lut2_set.count(input)) {
            candidate.shared_inputs.insert(input);
        } else {
            candidate.lut1_only_inputs.insert(input);
        }
    }
    
    // 识别LUT2独有输入
    for (auto input : lut2_set) {
        if (!lut1_set.count(input)) {
            candidate.lut2_only_inputs.insert(input);
        }
    }
    
    // 计算总输入数
    candidate.total_inputs = candidate.shared_inputs.size() + 
                            candidate.lut1_only_inputs.size() + 
                            candidate.lut2_only_inputs.size();
    
    if (enable_debug && candidate.total_inputs <= 6) {
        log("    Input analysis: %zu shared, %zu lut1-only, %zu lut2-only, %d total\n",
            candidate.shared_inputs.size(),
            candidate.lut1_only_inputs.size(), 
            candidate.lut2_only_inputs.size(),
            candidate.total_inputs);
    }
    
    // 基础约束：总输入数不能超过6
    if (candidate.total_inputs > 6) {
        candidate.failure_reason = stringf("Total inputs %d exceeds GTP_LUT6D limit", 
                                          candidate.total_inputs);
        return false;
    }
    
    // 基础约束：必须有一些有意义的输入关系
    if (candidate.total_inputs == 0) {
        candidate.failure_reason = "No valid inputs found";
        return false;
    }
    
    return true;
}

// checkBasicMergeConstraints() - 基础合并约束检查
// 严格按照v1.2方案中的约束条件
bool LUTMergeOptimizer::checkBasicMergeConstraints(const LUTMergeCandidate &candidate)
{
    Cell *lut1 = candidate.lut1;
    Cell *lut2 = candidate.lut2;
    
    // 约束1：输出数量检查（已确保是单输出LUT）
    // 这个检查在analyzeMergeCandidate开始时已完成
    
    // 约束2：LUT类型兼容性检查
    int lut1_inputs = getLUTInputCount(lut1);
    int lut2_inputs = getLUTInputCount(lut2);
    
    if (lut1_inputs < 1 || lut1_inputs > 6 || lut2_inputs < 1 || lut2_inputs > 6) {
        return false;
    }
    
    // 约束3：输出信号连接检查
    // 确保两个LUT的输出没有直接连接（避免组合环）
    SigBit lut1_output = sigmap(lut1->getPort(ID::O)[0]);
    SigBit lut2_output = sigmap(lut2->getPort(ID::O)[0]);
    
    if (lut1_output == lut2_output) {
        return false;
    }
    
    // 约束4：检查是否有输出到输入的连接
    vector<SigBit> lut1_inputs_vec, lut2_inputs_vec;
    getCellInputsVector(lut1, lut1_inputs_vec);
    getCellInputsVector(lut2, lut2_inputs_vec);
    
    // LUT1的输出不能连接到LUT2的输入
    for (auto input : lut2_inputs_vec) {
        if (sigmap(input) == lut1_output) {
            return false; // 会形成依赖关系，不能合并
        }
    }
    
    // LUT2的输出不能连接到LUT1的输入  
    for (auto input : lut1_inputs_vec) {
        if (sigmap(input) == lut2_output) {
            return false; // 会形成依赖关系，不能合并
        }
    }
    
    // 约束5：策略相关的额外约束
    if (strategy == CONSERVATIVE) {
        // 保守策略：只允许输入数较少的合并
        if (candidate.total_inputs > 4) {
            return false;
        }
        // 保守策略：只允许有足够共享输入的合并
        if (candidate.shared_inputs.size() == 0) {
            return false;
        }
    }
    
    return true;
}

// evaluateTimingImpact() - 时序影响评估
// 基于bit2depth数据评估合并对时序的影响
bool LUTMergeOptimizer::evaluateTimingImpact(LUTMergeCandidate &candidate)
{
    candidate.timing_impact = 0.0;
    candidate.depth1 = 0.0;
    candidate.depth2 = 0.0;
    
    // 如果没有时序数据，跳过时序评估
    if (!bit2depth_ref || bit2depth_ref->empty()) {
        if (enable_debug) {
            log("    No timing data available, skipping timing evaluation\n");
        }
        return true;
    }
    
    // 获取LUT1的逻辑深度
    SigBit lut1_output = sigmap(candidate.lut1->getPort(ID::O)[0]);
    if (bit2depth_ref->count(lut1_output)) {
        candidate.depth1 = bit2depth_ref->at(lut1_output);
    }
    
    // 获取LUT2的逻辑深度  
    SigBit lut2_output = sigmap(candidate.lut2->getPort(ID::O)[0]);
    if (bit2depth_ref->count(lut2_output)) {
        candidate.depth2 = bit2depth_ref->at(lut2_output);
    }
    
    // 估算合并后的逻辑深度
    // GTP_LUT6D会增加一定的延迟，通常是单LUT延迟的1.1-1.2倍
    float merged_depth = max(candidate.depth1, candidate.depth2) * 1.15f;
    
    // 计算时序影响
    candidate.timing_impact = merged_depth - max(candidate.depth1, candidate.depth2);
    
    // 时序约束检查
    if (strategy == CONSERVATIVE && candidate.timing_impact > 0.5) {
        candidate.failure_reason = stringf("Timing impact %.2f too high for conservative strategy", 
                                          candidate.timing_impact);
        return false;
    }
    
    if (candidate.timing_impact > 2.0) {
        candidate.failure_reason = stringf("Timing impact %.2f exceeds limit", 
                                          candidate.timing_impact);
        return false;
    }
    
    if (enable_debug) {
        log("    Timing: depth1=%.2f, depth2=%.2f, impact=%.2f\n",
            candidate.depth1, candidate.depth2, candidate.timing_impact);
    }
    
    return true;
}

YOSYS_NAMESPACE_END