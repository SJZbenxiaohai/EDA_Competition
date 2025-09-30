/*
 * GTP_LUT6D INIT值精确计算模块
 * 
 * 作用: INIT值精确计算模块，严格对齐GTP_LUT6D硬件行为
 * 文件: techlibs/pango/lut_merge_init.cc
 * 创建时间: 2025-09-26
 * 任务: Phase 3.3 - P3.3
 * 依据方案: 17-GTP_LUT6D修正执行方案.md Priority 3.3
 * 预计行数: ~800行
 * 
 * ⚠️ 极高风险警告 ⚠️ 
 * INIT值错误会导致硬件功能错误
 * 必须严格按照pango_sim.v:988硬件行为实现
 * 输入引脚映射必须100%准确
 * 地址计算逻辑不能有任何偏差
 * 
 * 硬件真相（基于pango_sim.v:988）:
 * // Z5 = 5输入LUT，INIT[31:0]
 * $lut #(.WIDTH(5),.LUT(INIT[31:0])) luta_cell(.A({I4,I3,I2,I1,I0}),.Y(z5a));
 * // 辅助5输入LUT，INIT[63:32]  
 * $lut #(.WIDTH(5),.LUT(INIT[63:32])) lutb_cell(.A({I4,I3,I2,I1,I0}),.Y(z5b));
 * // Z = MUX(z5a, z5b, I5)
 * $mux #(.WIDTH(1)) u1(.A(z5a),.B(z5b),.S(I5),.Y(Z));
 * 
 * 核心功能:
 * 1. computeGTP_LUT6D_INIT() - 主INIT计算接口
 * 2. computeINIT_Shannon() - 香农展开INIT计算
 * 3. computeINIT_LogicContainment() - 逻辑包含INIT计算
 * 4. computeINIT_InputSubset() - 输入子集INIT计算
 * 5. computeINIT_PartialSharing() - 部分共享INIT计算
 * 6. computeINIT_IndependentReuse() - 独立复用INIT计算
 * 7. computeINIT_FunctionMux() - 功能复用INIT计算
 * 8. arrangeInputPins() - 输入引脚排序
 */

#include "lut_merge_pango.h"
#include "kernel/log.h"
#include <algorithm>

YOSYS_NAMESPACE_BEGIN

// =============================================================================
// 输入引脚排序 - 确保与GTP_LUT6D硬件引脚对应
// =============================================================================

/**
 * 输入引脚排序 - 为合并后的LUT安排输入引脚顺序
 * 
 * 排序策略：
 * 1. 香农展开：split_var放在I5位置
 * 2. 其他类型：按信号重要性和时序优先级排序
 * 
 * @param candidate 合并候选对象
 * @return 排序后的输入引脚顺序 (对应I0-I5)
 */
vector<SigBit> LUTMergeOptimizer::arrangeInputPins(const LUTMergeCandidate &candidate)
{
    vector<SigBit> input_order;
    
    // 收集所有输入信号
    pool<SigBit> all_inputs = candidate.shared_inputs;
    all_inputs.insert(candidate.lut1_only_inputs.begin(), candidate.lut1_only_inputs.end());
    all_inputs.insert(candidate.lut2_only_inputs.begin(), candidate.lut2_only_inputs.end());
    
    if (enable_debug) {
        log("=== Input Pin Arrangement ===\n");
        log("  Merge type: %s\n", getMergeTypeString(candidate.merge_type).c_str());
        log("  Total inputs: %zu\n", all_inputs.size());
    }
    
    // 根据合并类型采用不同的排序策略
    switch (candidate.merge_type) {
        case MergeType::SIX_INPUT_SHANNON:
        case MergeType::SIX_INPUT_SHANNON_REVERSE:
            input_order = arrangePinsForShannon(candidate, all_inputs);
            break;
            
        case MergeType::LOGIC_CONTAINMENT:
            input_order = arrangePinsForLogicContainment(candidate, all_inputs);
            break;
            
        case MergeType::INPUT_SUBSET:
            input_order = arrangePinsForInputSubset(candidate, all_inputs);
            break;
            
        case MergeType::PARTIAL_SHARING_5INPUT:
            input_order = arrangePinsForPartialSharing(candidate, all_inputs);
            break;
            
        case MergeType::INDEPENDENT_REUSE:
        case MergeType::FUNCTION_MULTIPLEXING:
            input_order = arrangePinsForGeneralCase(candidate, all_inputs);
            break;
            
        default:
            log_error("Unsupported merge type for input pin arrangement: %s\n",
                     getMergeTypeString(candidate.merge_type).c_str());
            break;
    }
    
    // 验证结果
    if (input_order.size() != all_inputs.size()) {
        log_error("Input pin arrangement size mismatch: expected %zu, got %zu\n",
                 all_inputs.size(), input_order.size());
    }
    
    if (enable_debug) {
        log("  Arranged input order:\n");
        for (int i = 0; i < input_order.size(); i++) {
            log("    I%d: %s\n", i, log_signal(input_order[i]));
        }
    }
    
    return input_order;
}

/**
 * 香农展开的输入引脚排序 - split_var必须在I5位置
 */
vector<SigBit> LUTMergeOptimizer::arrangePinsForShannon(
    const LUTMergeCandidate &candidate, 
    const pool<SigBit> &all_inputs)
{
    vector<SigBit> input_order;
    
    // 确保是6输入情况
    if (all_inputs.size() != 6) {
        log_error("Shannon expansion requires exactly 6 inputs, got %zu\n", all_inputs.size());
        return input_order;
    }
    
    // split_var必须放在I5位置（根据pango_sim.v硬件定义）
    SigBit split_var = candidate.split_variable;
    if (split_var.wire == nullptr) {
        log_error("Invalid split variable for Shannon expansion\n");
        return input_order;
    }
    
    // 收集非split_var的输入
    vector<SigBit> other_inputs;
    for (auto sig : all_inputs) {
        if (sigmap(sig) != sigmap(split_var)) {
            other_inputs.push_back(sig);
        }
    }
    
    if (other_inputs.size() != 5) {
        log_error("Expected 5 non-split inputs, got %zu\n", other_inputs.size());
        return input_order;
    }
    
    // 对非split_var输入进行优先级排序
    sort(other_inputs.begin(), other_inputs.end(), 
         [&](const SigBit &a, const SigBit &b) {
             return getSignalPriority(a) > getSignalPriority(b);
         });
    
    // 构造最终顺序：I0-I4为other_inputs，I5为split_var
    for (int i = 0; i < 5; i++) {
        input_order.push_back(other_inputs[i]);
    }
    input_order.push_back(split_var);  // I5 = split_var
    
    return input_order;
}

/**
 * 逻辑包含的输入引脚排序
 * Fix for issue #47: 正确处理LOGIC_CONTAINMENT的输入排序
 *
 * 策略：
 * 1. 共享输入放在I0-I4位置（优先级排序）
 * 2. 容器LUT独有的输入放在剩余位置
 * 3. I5保留作为选择信号（虽然不在input_order中体现）
 */
vector<SigBit> LUTMergeOptimizer::arrangePinsForLogicContainment(
    const LUTMergeCandidate &candidate,
    const pool<SigBit> &all_inputs)
{
    vector<SigBit> input_order;

    if (enable_debug) {
        log("  === Arranging pins for LOGIC_CONTAINMENT ===\n");
        log("    Shared inputs: %zu\n", candidate.shared_inputs.size());
        log("    LUT1 only: %zu\n", candidate.lut1_only_inputs.size());
        log("    LUT2 only: %zu\n", candidate.lut2_only_inputs.size());
    }

    // Fix for issue #47: 确保共享输入存在
    if (candidate.shared_inputs.empty()) {
        log_error("LOGIC_CONTAINMENT requires shared inputs\n");
        return input_order;
    }

    // 1. 首先添加共享输入（这些是被包含LUT的所有输入）
    vector<SigBit> shared_inputs(candidate.shared_inputs.begin(),
                                 candidate.shared_inputs.end());

    // 按优先级排序共享输入
    sort(shared_inputs.begin(), shared_inputs.end(),
         [&](const SigBit &a, const SigBit &b) {
             return getSignalPriority(a) > getSignalPriority(b);
         });

    for (const auto &sig : shared_inputs) {
        input_order.push_back(sig);
    }

    // 2. 然后添加容器LUT独有的输入
    vector<SigBit> unique_inputs;

    // 容器LUT的独有输入
    for (const auto &sig : candidate.lut1_only_inputs) {
        unique_inputs.push_back(sig);
    }
    for (const auto &sig : candidate.lut2_only_inputs) {
        unique_inputs.push_back(sig);
    }

    // 按优先级排序独有输入
    sort(unique_inputs.begin(), unique_inputs.end(),
         [&](const SigBit &a, const SigBit &b) {
             return getSignalPriority(a) > getSignalPriority(b);
         });

    for (const auto &sig : unique_inputs) {
        input_order.push_back(sig);
    }

    // 限制最多6个输入（GTP_LUT6D硬件限制）
    if (input_order.size() > 6) {
        log_warning("LOGIC_CONTAINMENT has %zu inputs, truncating to 6\n",
                   input_order.size());
        input_order.resize(6);
    }

    if (enable_debug) {
        log("    Final input order (%zu pins):\n", input_order.size());
        for (size_t i = 0; i < input_order.size(); i++) {
            log("      I%zu: %s\n", i, log_signal(input_order[i]));
        }
    }

    return input_order;
}

/**
 * 输入子集的引脚排序
 */
vector<SigBit> LUTMergeOptimizer::arrangePinsForInputSubset(
    const LUTMergeCandidate &candidate,
    const pool<SigBit> &all_inputs)
{
    // 优先排列子集LUT的输入，然后是独有输入
    vector<SigBit> input_order;
    
    // 首先添加共享输入
    for (auto sig : candidate.shared_inputs) {
        input_order.push_back(sig);
    }
    
    // 然后添加独有输入
    for (auto sig : candidate.lut1_only_inputs) {
        input_order.push_back(sig);
    }
    for (auto sig : candidate.lut2_only_inputs) {
        input_order.push_back(sig);
    }
    
    return input_order;
}

/**
 * 部分共享的引脚排序
 */
vector<SigBit> LUTMergeOptimizer::arrangePinsForPartialSharing(
    const LUTMergeCandidate &candidate,
    const pool<SigBit> &all_inputs)
{
    return arrangePinsForInputSubset(candidate, all_inputs);
}

/**
 * 通用情况的引脚排序
 */
vector<SigBit> LUTMergeOptimizer::arrangePinsForGeneralCase(
    const LUTMergeCandidate &candidate,
    const pool<SigBit> &all_inputs)
{
    vector<SigBit> input_order(all_inputs.begin(), all_inputs.end());
    
    // 按信号优先级排序
    sort(input_order.begin(), input_order.end(),
         [&](const SigBit &a, const SigBit &b) {
             return getSignalPriority(a) > getSignalPriority(b);
         });
    
    return input_order;
}

/**
 * 获取信号优先级 - 用于输入排序
 */
int LUTMergeOptimizer::getSignalPriority(const SigBit &signal)
{
    // 优先级策略：
    // 1. 时序深度浅的信号优先级高
    // 2. 共享输入优先级高
    // 3. 常用信号优先级高
    
    int priority = 1000;  // 基础优先级
    
    // 时序深度考虑
    if (bit2depth_ref && bit2depth_ref->count(sigmap(signal))) {
        float depth = bit2depth_ref->at(sigmap(signal));
        priority += (int)(100 * (10.0 - depth));  // 深度越小，优先级越高
    }
    
    // 信号名称考虑（简单策略）
    if (signal.wire) {
        string name = signal.wire->name.str();
        if (name.find("clk") != string::npos) priority += 500;    // 时钟信号
        if (name.find("rst") != string::npos) priority += 400;    // 复位信号
        if (name.find("en") != string::npos) priority += 300;     // 使能信号
    }
    
    return priority;
}

// =============================================================================
// 主INIT计算接口
// =============================================================================

/**
 * 主INIT计算接口 - 根据合并类型分别计算INIT值
 * 
 * ⚠️ 关键函数 - 严格按pango_sim.v:988硬件行为实现
 * 
 * @param candidate 合并候选对象
 * @param input_order 输入引脚顺序 (对应I0-I5)
 * @return 64位INIT值 (INIT[31:0]=z5a, INIT[63:32]=z5b)
 */
vector<bool> LUTMergeOptimizer::computeGTP_LUT6D_INIT(
    const LUTMergeCandidate &candidate,
    const vector<SigBit> &input_order) 
{
    if (enable_debug) {
        log("=== GTP_LUT6D INIT Computation ===\n");
        log("  Merge type: %s\n", getMergeTypeString(candidate.merge_type).c_str());
        log("  Input order size: %zu\n", input_order.size());
        log("  LUT1: %s, LUT2: %s\n", 
            candidate.lut1 ? candidate.lut1->name.c_str() : "null",
            candidate.lut2 ? candidate.lut2->name.c_str() : "null");
    }
    
    vector<bool> init(64, false);
    
    // ✅ 按合并类型分别处理，确保逻辑自洽
    switch (candidate.merge_type) {
        case MergeType::SIX_INPUT_SHANNON:
        case MergeType::SIX_INPUT_SHANNON_REVERSE:
            init = computeINIT_Shannon(candidate, input_order);
            break;
            
        case MergeType::LOGIC_CONTAINMENT:
            init = computeINIT_LogicContainment(candidate, input_order);
            break;
            
        case MergeType::INPUT_SUBSET:
            init = computeINIT_InputSubset(candidate, input_order);
            break;
            
        case MergeType::PARTIAL_SHARING_5INPUT:
            init = computeINIT_PartialSharing(candidate, input_order);
            break;
            
        case MergeType::INDEPENDENT_REUSE:
            init = computeINIT_IndependentReuse(candidate, input_order);
            break;
            
        case MergeType::FUNCTION_MULTIPLEXING:
            init = computeINIT_FunctionMux(candidate, input_order);
            break;
            
        default:
            log_error("Unsupported merge type %s for INIT computation\n",
                     getMergeTypeString(candidate.merge_type).c_str());
            break;
    }
    
    // 验证结果
    if (enable_debug) {
        log("  INIT computation completed: %zu bits\n", init.size());
        if (init.size() != 64) {
            log("  ❌ WARNING: INIT size is not 64 bits! merge_type=%s\n", 
                getMergeTypeString(candidate.merge_type).c_str());
        }
        debugINITValue(init);
    }
    
    return init;
}

// =============================================================================
// 香农展开INIT计算 - 严格按v1.2方案实现
// =============================================================================

/**
 * 香农展开INIT计算 - 严格按照v1.2方案第467行算法实现
 * 
 * ⚠️ 最关键函数 - 必须与硬件行为完全一致
 * 
 * 硬件真相:
 * INIT[31:0] ← Z5输出的真值表（5输入函数）
 * INIT[63:32] ← Z在I5=1时的辅助真值表
 * 
 * @param candidate 合并候选对象
 * @param input_order 输入顺序 (I5必须是split_var)
 * @return 64位INIT值
 */
vector<bool> LUTMergeOptimizer::computeINIT_Shannon(
    const LUTMergeCandidate &candidate, 
    const vector<SigBit> &input_order) 
{
    vector<bool> init(64, false);
    
    if (enable_debug) {
        log("  === Shannon INIT Computation ===\n");
    }
    
    // 确保是6输入香农展开
    if (input_order.size() != 6) {
        log_error("Shannon expansion requires exactly 6 inputs, got %zu\n", input_order.size());
        return init;
    }
    
    Cell *z5_lut = candidate.z5_lut;
    Cell *z_lut = candidate.z_lut;
    
    if (!z5_lut || !z_lut) {
        log_error("Invalid LUT pointers for Shannon INIT computation\n");
        return init;
    }
    
    // 获取原始LUT的真值表和输入
    vector<bool> z5_truth = extractLUTTruthTable(z5_lut);
    vector<bool> z_truth = extractLUTTruthTable(z_lut);
    
    vector<SigBit> z5_inputs, z_inputs;
    getCellInputsVector(z5_lut, z5_inputs);
    getCellInputsVector(z_lut, z_inputs);
    
    if (z5_truth.empty() || z_truth.empty()) {
        log_error("Failed to extract truth tables for Shannon INIT computation\n");
        return init;
    }
    
    // 【关键】建立merged输入顺序到原始LUT输入的映射
    // input_order[i] 对应 GTP_LUT6D的Ii引脚
    dict<SigBit, int> merged_input_pos;
    for (int i = 0; i < input_order.size(); i++) {
        merged_input_pos[sigmap(input_order[i])] = i;
    }
    
    // ✅ 计算INIT[31:0] - Z5输出（对应I5=0的情况）
    for (int addr = 0; addr < 32; addr++) {
        bool z5_output = computeLUTOutputAtMergedAddress(z5_lut, z5_truth, z5_inputs, 
                                                        input_order, merged_input_pos, addr);
        init[addr] = z5_output;
    }
    
    // ✅ 计算INIT[63:32] - Z在I5=1时的辅助LUT
    int split_pos = 5;  // I5位置（按照硬件定义）
    
    for (int addr = 0; addr < 32; addr++) {
        // 构造6输入地址：split_var(I5)=1，其余由addr确定
        int full_addr = addr | (1 << split_pos);
        
        bool z_output = computeLUTOutputAtMergedAddress(z_lut, z_truth, z_inputs,
                                                       input_order, merged_input_pos, full_addr);
        init[addr + 32] = z_output;
    }
    
    if (enable_debug) {
        log("    Shannon INIT computed: split at I5\n");
        log("    init vector size: %zu bits\n", init.size());
        // ✅ Bug修复: 检查空指针避免段错误（虽然理论上这里不会是null）
        log("    Z5_LUT: %s (%zu inputs)\n", 
            z5_lut ? z5_lut->name.c_str() : "null", z5_inputs.size());
        log("    Z_LUT: %s (%zu inputs)\n", 
            z_lut ? z_lut->name.c_str() : "null", z_inputs.size());
    }
    
    return init;
}

// =============================================================================
// 其他合并类型的INIT计算
// =============================================================================

/**
 * 逻辑包含INIT计算
 * Fix for issue #47: 正确实现LOGIC_CONTAINMENT的INIT计算
 *
 * LOGIC_CONTAINMENT的特点：
 * 1. 一个LUT的输入是另一个LUT输入的子集
 * 2. 被包含LUT可以通过设置额外输入为常量来复现
 * 3. GTP_LUT6D的I5用于选择两个LUT的输出
 */
vector<bool> LUTMergeOptimizer::computeINIT_LogicContainment(
    const LUTMergeCandidate &candidate,
    const vector<SigBit> &input_order)
{
    vector<bool> init(64, false);

    if (enable_debug) {
        log("  === Logic Containment INIT Computation ===\n");
    }

    Cell *contained_lut = candidate.z5_lut;  // 被包含的LUT（输入较少）
    Cell *container_lut = candidate.z_lut;   // 包含的LUT（输入较多）

    // 如果z5_lut或z_lut为空，说明角色分配有问题
    if (!contained_lut || !container_lut) {
        log_error("Invalid LUT assignment for LOGIC_CONTAINMENT\n");
        return init;
    }

    vector<bool> contained_truth = extractLUTTruthTable(contained_lut);
    vector<bool> container_truth = extractLUTTruthTable(container_lut);

    vector<SigBit> contained_inputs, container_inputs;
    getCellInputsVector(contained_lut, contained_inputs);
    getCellInputsVector(container_lut, container_inputs);

    if (enable_debug) {
        log("    Contained LUT: %s (%zu inputs)\n",
            contained_lut->name.c_str(), contained_inputs.size());
        log("    Container LUT: %s (%zu inputs)\n",
            container_lut->name.c_str(), container_inputs.size());
        log("    Input order size: %zu\n", input_order.size());
    }

    // 建立输入映射
    dict<SigBit, int> merged_input_pos;
    for (int i = 0; i < input_order.size(); i++) {
        merged_input_pos[sigmap(input_order[i])] = i;
    }

    // Fix for issue #47: 使用GTP_LUT6D的硬件特性
    // I5=0时输出Z5（被包含的LUT）
    // I5=1时输出Z（包含的LUT）

    // 计算INIT[31:0] - I5=0时的输出（对应被包含LUT）
    for (int addr = 0; addr < 32; addr++) {
        // addr只使用低5位（I0-I4）
        bool output = computeLUTOutputAtMergedAddress(contained_lut, contained_truth, contained_inputs,
                                                     input_order, merged_input_pos, addr);
        init[addr] = output;
    }

    // 计算INIT[63:32] - I5=1时的输出（对应包含LUT）
    for (int addr = 0; addr < 32; addr++) {
        // 构造6位地址，I5=1
        int full_addr = addr | (1 << 5);
        bool output = computeLUTOutputAtMergedAddress(container_lut, container_truth, container_inputs,
                                                     input_order, merged_input_pos, full_addr);
        init[addr + 32] = output;
    }

    if (enable_debug) {
        log("    LOGIC_CONTAINMENT INIT computed\n");
        log("    Using I5 as selector: I5=0 -> contained LUT, I5=1 -> container LUT\n");
    }

    return init;
}

/**
 * 输入子集INIT计算
 */
vector<bool> LUTMergeOptimizer::computeINIT_InputSubset(
    const LUTMergeCandidate &candidate,
    const vector<SigBit> &input_order)
{
    vector<bool> init(64, false);
    
    if (enable_debug) {
        log("  === Input Subset INIT Computation ===\n");
    }
    
    // 确定哪个是子集LUT，哪个是超集LUT
    Cell *subset_lut, *superset_lut;
    vector<SigBit> subset_inputs, superset_inputs;
    vector<bool> subset_truth, superset_truth;
    
    if (candidate.lut1_only_inputs.empty()) {
        // LUT1的输入是LUT2的子集
        subset_lut = candidate.lut1;
        superset_lut = candidate.lut2;
    } else {
        // LUT2的输入是LUT1的子集
        subset_lut = candidate.lut2;
        superset_lut = candidate.lut1;
    }
    
    getCellInputsVector(subset_lut, subset_inputs);
    getCellInputsVector(superset_lut, superset_inputs);
    subset_truth = extractLUTTruthTable(subset_lut);
    superset_truth = extractLUTTruthTable(superset_lut);
    
    dict<SigBit, int> merged_input_pos;
    for (int i = 0; i < input_order.size(); i++) {
        merged_input_pos[sigmap(input_order[i])] = i;
    }
    
    // INIT[31:0] - 子集LUT的输出
    for (int addr = 0; addr < 32; addr++) {
        bool output = computeLUTOutputAtMergedAddress(subset_lut, subset_truth, subset_inputs,
                                                     input_order, merged_input_pos, addr);
        init[addr] = output;
    }
    
    // INIT[63:32] - 超集LUT的输出
    for (int addr = 0; addr < 32; addr++) {
        bool output = computeLUTOutputAtMergedAddress(superset_lut, superset_truth, superset_inputs,
                                                     input_order, merged_input_pos, addr);
        init[addr + 32] = output;
    }
    
    return init;
}

/**
 * 部分共享INIT计算
 */
vector<bool> LUTMergeOptimizer::computeINIT_PartialSharing(
    const LUTMergeCandidate &candidate,
    const vector<SigBit> &input_order)
{
    vector<bool> init(64, false);
    
    if (enable_debug) {
        log("  === Partial Sharing INIT Computation ===\n");
    }
    
    Cell *lut1 = candidate.lut1;
    Cell *lut2 = candidate.lut2;
    
    vector<bool> truth1 = extractLUTTruthTable(lut1);
    vector<bool> truth2 = extractLUTTruthTable(lut2);
    
    vector<SigBit> inputs1, inputs2;
    getCellInputsVector(lut1, inputs1);
    getCellInputsVector(lut2, inputs2);
    
    dict<SigBit, int> merged_input_pos;
    for (int i = 0; i < input_order.size(); i++) {
        merged_input_pos[sigmap(input_order[i])] = i;
    }
    
    // 选择输入较少的作为Z5输出源
    Cell *z5_source = (inputs1.size() <= inputs2.size()) ? lut1 : lut2;
    Cell *z_source = (inputs1.size() <= inputs2.size()) ? lut2 : lut1;
    
    vector<bool> z5_truth = (inputs1.size() <= inputs2.size()) ? truth1 : truth2;
    vector<bool> z_truth = (inputs1.size() <= inputs2.size()) ? truth2 : truth1;
    
    vector<SigBit> z5_inputs = (inputs1.size() <= inputs2.size()) ? inputs1 : inputs2;
    vector<SigBit> z_inputs = (inputs1.size() <= inputs2.size()) ? inputs2 : inputs1;
    
    // INIT[31:0] - Z5输出
    for (int addr = 0; addr < 32; addr++) {
        bool output = computeLUTOutputAtMergedAddress(z5_source, z5_truth, z5_inputs,
                                                     input_order, merged_input_pos, addr);
        init[addr] = output;
    }
    
    // INIT[63:32] - Z输出
    for (int addr = 0; addr < 32; addr++) {
        bool output = computeLUTOutputAtMergedAddress(z_source, z_truth, z_inputs,
                                                     input_order, merged_input_pos, addr);
        init[addr + 32] = output;
    }
    
    return init;
}

/**
 * 独立复用INIT计算
 */
vector<bool> LUTMergeOptimizer::computeINIT_IndependentReuse(
    const LUTMergeCandidate &candidate,
    const vector<SigBit> &input_order)
{
    vector<bool> init(64, false);
    
    if (enable_debug) {
        log("  === Independent Reuse INIT Computation ===\n");
    }
    
    // 对于独立复用，两个LUT功能相对独立
    Cell *z5_lut = candidate.z5_lut;
    Cell *z_lut = candidate.z_lut;
    
    vector<bool> z5_truth = extractLUTTruthTable(z5_lut);
    vector<bool> z_truth = extractLUTTruthTable(z_lut);
    
    vector<SigBit> z5_inputs, z_inputs;
    getCellInputsVector(z5_lut, z5_inputs);
    getCellInputsVector(z_lut, z_inputs);
    
    dict<SigBit, int> merged_input_pos;
    for (int i = 0; i < input_order.size(); i++) {
        merged_input_pos[sigmap(input_order[i])] = i;
    }
    
    // INIT[31:0] - Z5 LUT输出
    for (int addr = 0; addr < 32; addr++) {
        bool output = computeLUTOutputAtMergedAddress(z5_lut, z5_truth, z5_inputs,
                                                     input_order, merged_input_pos, addr);
        init[addr] = output;
    }
    
    // INIT[63:32] - Z LUT输出（或复用Z5）
    for (int addr = 0; addr < 32; addr++) {
        bool output = computeLUTOutputAtMergedAddress(z_lut, z_truth, z_inputs,
                                                     input_order, merged_input_pos, addr);
        init[addr + 32] = output;
    }
    
    return init;
}

/**
 * 功能复用INIT计算
 */
vector<bool> LUTMergeOptimizer::computeINIT_FunctionMux(
    const LUTMergeCandidate &candidate,
    const vector<SigBit> &input_order)
{
    // 对于功能复用，实现与独立复用类似
    return computeINIT_IndependentReuse(candidate, input_order);
}

// =============================================================================
// 辅助函数
// =============================================================================

/**
 * 在合并地址空间计算LUT输出
 * 
 * @param lut LUT单元
 * @param truth_table LUT真值表
 * @param lut_inputs LUT输入向量
 * @param merged_order 合并后的输入顺序
 * @param merged_pos_map 合并输入位置映射
 * @param merged_addr 合并地址空间的地址
 * @return LUT在该地址的输出值
 */
bool LUTMergeOptimizer::computeLUTOutputAtMergedAddress(
    Cell *lut,
    const vector<bool> &truth_table,
    const vector<SigBit> &lut_inputs,
    const vector<SigBit> &merged_order,
    const dict<SigBit, int> &merged_pos_map,
    int merged_addr)
{
    // 将merged地址转换到LUT的原始地址空间
    int lut_addr = 0;
    
    for (int i = 0; i < lut_inputs.size(); i++) {
        SigBit lut_input_sig = sigmap(lut_inputs[i]);
        
        if (merged_pos_map.count(lut_input_sig)) {
            int bit_pos = merged_pos_map.at(lut_input_sig);
            if (merged_addr & (1 << bit_pos)) {
                lut_addr |= (1 << i);
            }
        }
        // 如果lut_input_sig不在merged_order中，默认为0
    }
    
    // 从真值表读取输出
    if (lut_addr < truth_table.size()) {
        return truth_table[lut_addr];
    } else {
        return false;  // 超出范围默认为0
    }
}

/**
 * 调试INIT值输出
 */
void LUTMergeOptimizer::debugINITValue(const vector<bool> &init)
{
    if (!enable_debug || init.size() != 64) return;
    
    log("    INIT[31:0]  (Z5): ");
    for (int i = 31; i >= 0; i--) {
        log("%d", init[i] ? 1 : 0);
        if (i % 8 == 0 && i > 0) log("_");
    }
    log("\n");
    
    log("    INIT[63:32] (ZB): ");
    for (int i = 63; i >= 32; i--) {
        log("%d", init[i] ? 1 : 0);
        if (i % 8 == 0 && i > 32) log("_");
    }
    log("\n");
}

YOSYS_NAMESPACE_END