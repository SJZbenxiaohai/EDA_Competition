/*
 * yosys -- Yosys Open SYnthesis Suite
 *
 * Copyright (C) 2025  Shenzhen Pango Microsystems Co., Ltd.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * LUT合并优化器主实现文件
 * 
 * 功能：实现GTP_LUT6D合并优化的主框架，包括：
 * - 优化器配置和初始化
 * - 主优化流程控制
 * - 收敛性判断和早停机制
 * - 统计信息收集和报告生成
 * - 通用辅助函数
 */

#include "lut_merge_pango.h"
#include "kernel/log.h"
#include <algorithm>

YOSYS_NAMESPACE_BEGIN

// 构造函数 - 初始化默认配置
LUTMergeOptimizer::LUTMergeOptimizer() :
    strategy(BALANCED),
    benefit_threshold(3.0),
    max_iterations(3),
    enable_debug(false),
    bit2depth_ref(nullptr),
    current_module(nullptr),
    initial_lut_count(0),
    final_lut_count(0),
    successful_merges(0)
{
    merge_type_count.clear();
}

// 析构函数
LUTMergeOptimizer::~LUTMergeOptimizer()
{
    // 清理资源（当前为空，预留扩展）
}

// 设置优化策略（字符串版本）
void LUTMergeOptimizer::setStrategy(const string &s)
{
    if (s == "conservative") {
        strategy = CONSERVATIVE;
        // 保守策略：更高的阈值，更少的迭代
        if (benefit_threshold < 5.0) benefit_threshold = 5.0;
    } else if (s == "balanced") {
        strategy = BALANCED;
        // 平衡策略：默认配置
    } else if (s == "aggressive") {
        strategy = AGGRESSIVE;
        // 激进策略：更低的阈值，更多的迭代
        if (benefit_threshold > 2.0) benefit_threshold = 2.0;
    } else {
        log_warning("Unknown LUT merge strategy '%s', using 'balanced'\n", s.c_str());
        strategy = BALANCED;
    }
}

// 主优化接口 - 核心算法流程控制
bool LUTMergeOptimizer::optimize(Module *module)
{
    if (!module) {
        log_error("LUTMergeOptimizer::optimize(): module is null\n");
        return false;
    }
    
    current_module = module;
    sigmap.set(module);
    
    // 验证bit2depth数据可用性
    if (!bit2depth_ref) {
        log_warning("LUTMergeOptimizer: bit2depth data not available, timing analysis disabled\n");
    }
    
    log("=== Starting LUT merge optimization (v1.2) ===\n");
    log("Strategy: %s, Threshold: %.2f, Max iterations: %d\n",
        (strategy == CONSERVATIVE) ? "conservative" :
        (strategy == BALANCED) ? "balanced" : "aggressive",
        benefit_threshold, max_iterations);
    
    // 初始化统计
    initial_lut_count = countLUTs(module);
    final_lut_count = initial_lut_count;
    successful_merges = 0;
    merge_type_count.clear();
    
    if (initial_lut_count == 0) {
        log("No LUTs found in module, skipping optimization\n");
        return false;
    }
    
    log("Initial LUT count: %d\n", initial_lut_count);
    
    // 多轮迭代优化（收敛性控制）
    int prev_lut_count = initial_lut_count;
    
    for (int iter = 0; iter < max_iterations; iter++) {
        log("=== Iteration %d ===\n", iter + 1);
        
        // 步骤1：识别合并候选
        vector<LUTMergeCandidate> candidates;
        if (!identifyMergeCandidates(candidates)) {
            log("Failed to identify merge candidates\n");
            break;
        }
        
        if (candidates.empty()) {
            log("No merge candidates found\n");
            break;
        }
        
        log("Found %lu merge candidates\n", candidates.size());
        
        // 步骤2：选择最优匹配
        auto selected = selectOptimalMatching(candidates);
        
        if (selected.empty()) {
            log("No beneficial merges in this iteration\n");
            break;
        }
        
        log("Selected %lu merges for execution\n", selected.size());
        
        // 步骤3：执行合并
        int merges_executed = 0;
        for (const auto &candidate : selected) {
            if (enable_debug) {
                printCandidateInfo(candidate);
            }
            
            if (executeSingleMerge(candidate)) {
                merges_executed++;
                successful_merges++;
                merge_type_count[candidate.merge_type]++;
                
                if (enable_debug) {
                    log("  Successfully merged %s + %s (type: %s, benefit: %.2f)\n",
                        candidate.lut1->name.c_str(), candidate.lut2->name.c_str(),
                        getMergeTypeString(candidate.merge_type).c_str(),
                        candidate.total_benefit);
                }
            } else {
                if (enable_debug) {
                    log("  Failed to merge %s + %s: %s\n",
                        candidate.lut1->name.c_str(), candidate.lut2->name.c_str(),
                        candidate.failure_reason.c_str());
                }
            }
        }
        
        log("Executed %d merges in this iteration\n", merges_executed);
        
        // 步骤4：收敛性检查
        int current_lut_count = countLUTs(module);
        
        if (hasConverged(prev_lut_count, current_lut_count)) {
            log("Optimization converged, stopping early\n");
            break;
        }
        
        prev_lut_count = current_lut_count;
        updateIterationStats(selected);
    }
    
    // 最终统计
    final_lut_count = countLUTs(module);
    generateOptimizationReport();
    
    return successful_merges > 0;
}

// 收敛性判断
bool LUTMergeOptimizer::hasConverged(int prev_lut_count, int current_lut_count)
{
    // 如果LUT数量不再减少，认为已收敛
    if (current_lut_count >= prev_lut_count) {
        return true;
    }
    
    // 如果改善幅度很小（<1%），也认为接近收敛
    float improvement_rate = (float)(prev_lut_count - current_lut_count) / prev_lut_count;
    if (improvement_rate < 0.01) {  // 改善小于1%
        return true;
    }
    
    return false;
}

// 更新迭代统计信息
void LUTMergeOptimizer::updateIterationStats(const vector<LUTMergeCandidate> &selected)
{
    if (enable_debug) {
        log("Iteration stats:\n");
        for (const auto &pair : merge_type_count) {
            log("  %s: %d merges\n", 
                getMergeTypeString(pair.first).c_str(), pair.second);
        }
    }
}

// 生成最终优化报告
void LUTMergeOptimizer::generateOptimizationReport()
{
    log("=== LUT Merge Optimization Results ===\n");
    log("Initial LUT count: %d\n", initial_lut_count);
    log("Final LUT count: %d\n", final_lut_count);
    
    if (initial_lut_count > 0) {
        int luts_saved = initial_lut_count - final_lut_count;
        float savings_percent = 100.0f * luts_saved / initial_lut_count;
        log("LUTs saved: %d (%.1f%%)\n", luts_saved, savings_percent);
    }
    
    log("Total successful merges: %d\n", successful_merges);
    
    if (!merge_type_count.empty()) {
        log("Merge type breakdown:\n");
        for (const auto &pair : merge_type_count) {
            log("  %s: %d\n", 
                getMergeTypeString(pair.first).c_str(), pair.second);
        }
    }
    
    // 性能评估
    if (initial_lut_count > 0) {
        float merge_rate = 100.0f * successful_merges * 2 / initial_lut_count;
        log("Merge rate: %.1f%% (merged LUTs / total LUTs)\n", merge_rate);
    }
}

// 统计LUT数量（包括单输出和双输出）
int LUTMergeOptimizer::countLUTs(Module *module)
{
    int count = 0;
    
    for (auto cell : module->cells()) {
        if (isSingleOutputLUT(cell)) {
            count++;
        } else if (isGTP_LUT6D(cell)) {
            count += 2;  // 双输出LUT等价于2个单输出LUT
        }
    }
    
    return count;
}

// 判断是否为单输出LUT
bool LUTMergeOptimizer::isSingleOutputLUT(Cell *cell)
{
    const char *type_str = cell->type.c_str();
    
    // GTP_LUT1-6
    if (strncmp(type_str, "\\GTP_LUT", 8) == 0) {
        if (strlen(type_str) == 9) {  // "\\GTP_LUT" + 一个数字
            char digit = type_str[8];
            if (digit >= '1' && digit <= '6') {
                return true;
            }
        }
    }
    
    return false;
}

// 判断是否为双输出LUT6D
bool LUTMergeOptimizer::isGTP_LUT6D(Cell *cell)
{
    return cell->type == RTLIL::escape_id("GTP_LUT6D");
}

// 获取LUT输入数量
int LUTMergeOptimizer::getLUTInputCount(Cell *cell)
{
    if (!isSingleOutputLUT(cell)) {
        return 0;
    }
    
    const char *type_str = cell->type.c_str();
    if (strncmp(type_str, "\\GTP_LUT", 8) == 0 && strlen(type_str) == 9) {
        return type_str[8] - '0';
    }
    
    return 0;
}

// 获取LUT的输入信号
void LUTMergeOptimizer::getCellInputsVector(Cell *cell, vector<SigBit> &inputs)
{
    inputs.clear();
    
    if (!isSingleOutputLUT(cell)) {
        return;
    }
    
    int input_count = getLUTInputCount(cell);
    
    for (int i = 0; i < input_count; i++) {
        string port_name = stringf("\\I%d", i);
        if (cell->hasPort(RTLIL::escape_id(port_name))) {
            SigSpec sig = cell->getPort(RTLIL::escape_id(port_name));
            sig = sigmap(sig);
            if (sig.size() == 1) {
                inputs.push_back(sig[0]);
            }
        }
    }
}

// 获取LUT的输出信号
SigBit LUTMergeOptimizer::getCellOutput(Cell *cell)
{
    if (cell->hasPort(RTLIL::escape_id("Z"))) {
        SigSpec sig = cell->getPort(RTLIL::escape_id("Z"));
        sig = sigmap(sig);
        if (sig.size() == 1) {
            return sig[0];
        }
    }
    
    return SigBit();
}

// 提取LUT的真值表
vector<bool> LUTMergeOptimizer::extractLUTTruthTable(Cell *lut)
{
    vector<bool> truth_table;
    
    if (!lut->hasParam(RTLIL::escape_id("INIT"))) {
        return truth_table;
    }
    
    Const init_param = lut->getParam(RTLIL::escape_id("INIT"));
    
    for (int i = 0; i < init_param.size(); i++) {
        truth_table.push_back(init_param.bits().at(i) == RTLIL::State::S1);
    }
    
    return truth_table;
}

// 获取合并类型的字符串表示
string LUTMergeOptimizer::getMergeTypeString(MergeType type)
{
    switch (type) {
        case MergeType::LOGIC_CONTAINMENT:        return "LOGIC_CONTAINMENT";
        case MergeType::SIX_INPUT_SHANNON:        return "SIX_INPUT_SHANNON";
        case MergeType::SIX_INPUT_SHANNON_REVERSE: return "SIX_INPUT_SHANNON_REVERSE";
        case MergeType::INPUT_SUBSET:             return "INPUT_SUBSET";
        case MergeType::PARTIAL_SHARING_5INPUT:   return "PARTIAL_SHARING_5INPUT";
        case MergeType::INDEPENDENT_REUSE:        return "INDEPENDENT_REUSE";
        case MergeType::FUNCTION_MULTIPLEXING:    return "FUNCTION_MULTIPLEXING";
        default:                                  return "INVALID";
    }
}

// 打印候选信息（调试用）
void LUTMergeOptimizer::printCandidateInfo(const LUTMergeCandidate &candidate)
{
    log("  Candidate: %s + %s\n", 
        candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
    log("    Type: %s\n", getMergeTypeString(candidate.merge_type).c_str());
    log("    Total inputs: %d\n", candidate.total_inputs);
    log("    Shared inputs: %lu\n", candidate.shared_inputs.size());
    log("    Benefit: %.2f\n", candidate.total_benefit);
    
    if (!candidate.split_variable.wire) {
        log("    Split variable: %s (pos %d)\n", 
            log_signal(candidate.split_variable), candidate.split_bit_position);
    }
}

// === 全局辅助函数实现 ===

// 格式化INIT值为可读字符串
string formatInitValue(const vector<bool> &init)
{
    string result = "64'b";
    
    // 从高位到低位
    for (int i = init.size() - 1; i >= 0; i--) {
        result += init[i] ? '1' : '0';
    }
    
    return result;
}

// 检查合并类型是否有效
bool isValidMergeType(MergeType type)
{
    return type != MergeType::INVALID;
}

// 获取合并类型的优先级分数
float getMergeTypePriority(MergeType type)
{
    switch (type) {
        case MergeType::LOGIC_CONTAINMENT:        return 5.0f;
        case MergeType::SIX_INPUT_SHANNON:        return 4.0f;
        case MergeType::SIX_INPUT_SHANNON_REVERSE: return 4.0f;
        case MergeType::INPUT_SUBSET:             return 3.0f;
        case MergeType::PARTIAL_SHARING_5INPUT:   return 2.5f;
        case MergeType::INDEPENDENT_REUSE:        return 2.0f;
        case MergeType::FUNCTION_MULTIPLEXING:    return 1.0f;
        default:                                  return 0.0f;
    }
}

// === 核心算法函数实现（基础框架版本）===
// 这些函数实现了基础的框架，算法细节将在后续完善

// 识别合并候选 - 核心函数1
bool LUTMergeOptimizer::identifyMergeCandidates(vector<LUTMergeCandidate> &candidates)
{
    candidates.clear();
    
    if (!current_module) {
        log_error("identifyMergeCandidates: current_module is null\n");
        return false;
    }
    
    if (enable_debug) {
        log("Identifying merge candidates...\n");
    }
    
    // 收集所有单输出LUT
    vector<Cell*> lut_cells;
    for (auto cell : current_module->cells()) {
        if (isSingleOutputLUT(cell)) {
            lut_cells.push_back(cell);
        }
    }
    
    if (enable_debug) {
        log("Found %lu LUT cells for analysis\n", lut_cells.size());
    }
    
    // 两两配对检查合并可能性
    for (size_t i = 0; i < lut_cells.size(); i++) {
        for (size_t j = i + 1; j < lut_cells.size(); j++) {
            LUTMergeCandidate candidate;
            
            if (analyzeMergeCandidate(lut_cells[i], lut_cells[j], candidate)) {
                if (candidate.total_benefit >= benefit_threshold) {
                    candidates.push_back(candidate);
                }
            }
        }
    }
    
    if (enable_debug) {
        log("Identified %lu merge candidates\n", candidates.size());
    }
    
    return !candidates.empty();
}

// 分析合并候选 - 核心函数2
bool LUTMergeOptimizer::analyzeMergeCandidate(Cell *lut1, Cell *lut2, 
                                             LUTMergeCandidate &candidate)
{
    if (!lut1 || !lut2) {
        return false;
    }
    
    // 初始化候选信息
    candidate.lut1 = lut1;
    candidate.lut2 = lut2;
    candidate.merge_type = MergeType::INVALID;
    candidate.total_benefit = 0.0;
    candidate.failure_reason = "";
    
    // 获取输入分析
    vector<SigBit> inputs1, inputs2;
    getCellInputsVector(lut1, inputs1);
    getCellInputsVector(lut2, inputs2);
    
    // 分析输入关系
    candidate.shared_inputs.clear();
    candidate.lut1_only_inputs.clear();
    candidate.lut2_only_inputs.clear();
    
    pool<SigBit> all_inputs1(inputs1.begin(), inputs1.end());
    pool<SigBit> all_inputs2(inputs2.begin(), inputs2.end());
    
    // 寻找共享输入
    for (auto bit : all_inputs1) {
        if (all_inputs2.count(bit)) {
            candidate.shared_inputs.insert(bit);
        } else {
            candidate.lut1_only_inputs.insert(bit);
        }
    }
    
    for (auto bit : all_inputs2) {
        if (!all_inputs1.count(bit)) {
            candidate.lut2_only_inputs.insert(bit);
        }
    }
    
    candidate.total_inputs = candidate.shared_inputs.size() + 
                           candidate.lut1_only_inputs.size() + 
                           candidate.lut2_only_inputs.size();
    
    // 基础约束检查
    if (candidate.total_inputs > 6) {
        candidate.failure_reason = "Total inputs > 6";
        return false;
    }
    
    // 确定合并类型（简化版）
    candidate.merge_type = determineMergeType(candidate);
    
    if (candidate.merge_type == MergeType::INVALID) {
        candidate.failure_reason = "No valid merge type found";
        return false;
    }
    
    // 计算收益
    candidate.total_benefit = calculateMergeBenefit(candidate);
    
    // 时序信息（如果可用）
    if (bit2depth_ref) {
        SigBit out1 = getCellOutput(lut1);
        SigBit out2 = getCellOutput(lut2);
        
        if (out1.wire && bit2depth_ref->count(out1)) {
            candidate.depth1 = bit2depth_ref->at(out1);
        }
        if (out2.wire && bit2depth_ref->count(out2)) {
            candidate.depth2 = bit2depth_ref->at(out2);
        }
    }
    
    return candidate.total_benefit > 0;
}

// 确定合并类型 - 核心函数3（简化实现）
MergeType LUTMergeOptimizer::determineMergeType(LUTMergeCandidate &candidate)
{
    // 简化的合并类型判断逻辑
    
    // 逻辑包含：一个LUT的输入是另一个的子集
    if (candidate.lut1_only_inputs.empty()) {
        return MergeType::LOGIC_CONTAINMENT;
    }
    if (candidate.lut2_only_inputs.empty()) {
        return MergeType::LOGIC_CONTAINMENT;
    }
    
    // 6输入香农展开：总输入恰好6个
    if (candidate.total_inputs == 6) {
        // 选择一个非共享输入作为分割变量
        if (!candidate.lut1_only_inputs.empty()) {
            candidate.split_variable = *candidate.lut1_only_inputs.begin();
            candidate.split_bit_position = 5;  // 默认放在I5
            return MergeType::SIX_INPUT_SHANNON;
        }
        if (!candidate.lut2_only_inputs.empty()) {
            candidate.split_variable = *candidate.lut2_only_inputs.begin();
            candidate.split_bit_position = 5;
            return MergeType::SIX_INPUT_SHANNON_REVERSE;
        }
    }
    
    // 输入子集：总输入 <= 5
    if (candidate.total_inputs <= 5) {
        return MergeType::INPUT_SUBSET;
    }
    
    // 独立复用：输入较少的情况
    if (candidate.total_inputs <= 4) {
        return MergeType::INDEPENDENT_REUSE;
    }
    
    return MergeType::INVALID;
}

// 计算合并收益 - 核心函数4
float LUTMergeOptimizer::calculateMergeBenefit(const LUTMergeCandidate &candidate)
{
    if (candidate.merge_type == MergeType::INVALID) {
        return 0.0;
    }
    
    // 基础收益：每次合并节省1个LUT
    float base_benefit = 1.0;
    
    // 类型优先级奖励
    float type_bonus = getMergeTypePriority(candidate.merge_type);
    
    // 时序惩罚（如果时序数据可用）
    float timing_penalty = 0.0;
    if (bit2depth_ref && strategy == CONSERVATIVE) {
        // 保守策略下，深度增加会有惩罚
        float max_depth = max(candidate.depth1, candidate.depth2);
        if (max_depth > 5.0) {  // 深度阈值
            timing_penalty = (max_depth - 5.0) * 0.5;
        }
    }
    
    return base_benefit + type_bonus - timing_penalty;
}

// 选择最优匹配 - 核心函数5
vector<LUTMergeCandidate> LUTMergeOptimizer::selectOptimalMatching(
    const vector<LUTMergeCandidate> &candidates)
{
    vector<LUTMergeCandidate> selected;
    
    if (candidates.empty()) {
        return selected;
    }
    
    // 简化的贪心选择算法
    // 按收益排序，依次选择不冲突的候选
    vector<LUTMergeCandidate> sorted_candidates = candidates;
    sort(sorted_candidates.begin(), sorted_candidates.end(),
         [](const LUTMergeCandidate &a, const LUTMergeCandidate &b) {
             return a.total_benefit > b.total_benefit;
         });
    
    pool<Cell*> used_cells;  // 已经被选中的LUT
    
    for (const auto &candidate : sorted_candidates) {
        // 检查是否与已选择的候选冲突
        if (used_cells.count(candidate.lut1) || used_cells.count(candidate.lut2)) {
            continue;  // 冲突，跳过
        }
        
        // 收益阈值检查
        if (candidate.total_benefit < benefit_threshold) {
            break;  // 后续的收益更低，可以停止
        }
        
        // 选择这个候选
        selected.push_back(candidate);
        used_cells.insert(candidate.lut1);
        used_cells.insert(candidate.lut2);
    }
    
    return selected;
}

// 执行单个合并 - 核心函数6（基础实现）
bool LUTMergeOptimizer::executeSingleMerge(const LUTMergeCandidate &candidate)
{
    if (!current_module || !candidate.lut1 || !candidate.lut2) {
        return false;
    }
    
    if (enable_debug) {
        log("Executing merge: %s + %s\n", 
            candidate.lut1->name.c_str(), candidate.lut2->name.c_str());
    }
    
    try {
        // 创建新的GTP_LUT6D单元
        Cell *merged_cell = current_module->addCell(
            current_module->uniquify(stringf("$lut6d_merged_%d", successful_merges)),
            RTLIL::escape_id("GTP_LUT6D")
        );
        
        // 安排输入引脚
        vector<SigBit> input_order = arrangeInputPins(candidate);
        
        // 连接输入
        for (size_t i = 0; i < input_order.size() && i < 6; i++) {
            string port_name = stringf("\\I%lu", i);
            merged_cell->setPort(RTLIL::escape_id(port_name), input_order[i]);
        }
        
        // 计算INIT值
        vector<bool> init_values = computeGTP_LUT6D_INIT(candidate, input_order);
        
        // 转换为RTLIL::Const格式
        Const init_const;
        for (bool val : init_values) {
            init_const.bits().push_back(val ? RTLIL::State::S1 : RTLIL::State::S0);
        }
        merged_cell->setParam(RTLIL::escape_id("INIT"), init_const);
        
        // 连接输出
        SigBit out1 = getCellOutput(candidate.lut1);
        SigBit out2 = getCellOutput(candidate.lut2);
        
        merged_cell->setPort(RTLIL::escape_id("Z5"), out1);
        merged_cell->setPort(RTLIL::escape_id("Z"), out2);
        
        // 移除原始LUT
        current_module->remove(candidate.lut1);
        current_module->remove(candidate.lut2);
        
        if (enable_debug) {
            log("Successfully created merged LUT: %s\n", merged_cell->name.c_str());
        }
        
        return true;
        
    } catch (const std::exception &e) {
        if (enable_debug) {
            log("Failed to execute merge: %s\n", e.what());
        }
        return false;
    }
}

YOSYS_NAMESPACE_END