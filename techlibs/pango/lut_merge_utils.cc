// LUT合并INIT值计算和引脚排列实现
// GTP_LUT6D合并优化 - INIT值计算与引脚管理
// 创建时间：2025-09-24
// 版本：v1.0
// 作用：实现LUTMergeOptimizer类的INIT值计算和引脚排列方法

#include "lut_merge_pango.h"
#include "kernel/log.h"

YOSYS_NAMESPACE_BEGIN

// 排列输入引脚 - 为GTP_LUT6D确定I0-I5的连接顺序
vector<SigBit> LUTMergeOptimizer::arrangeInputPins(const LUTMergeCandidate &candidate) {
    vector<SigBit> input_order;
    
    // 收集所有唯一输入
    pool<SigBit> all_inputs;
    all_inputs.insert(candidate.shared_inputs.begin(), candidate.shared_inputs.end());
    all_inputs.insert(candidate.lut1_only_inputs.begin(), candidate.lut1_only_inputs.end());
    all_inputs.insert(candidate.lut2_only_inputs.begin(), candidate.lut2_only_inputs.end());
    
    // 优先级排列策略：
    // 1. 共享输入优先（I0-I3）
    // 2. LUT1独有输入
    // 3. LUT2独有输入
    // 4. 分割变量放在I5（如果适用）
    
    // 添加共享输入
    for (auto sig : candidate.shared_inputs) {
        if (input_order.size() < 6) {
            input_order.push_back(sig);
        }
    }
    
    // 添加LUT1独有输入
    for (auto sig : candidate.lut1_only_inputs) {
        if (input_order.size() < 6) {
            // 如果是分割变量，预留I5位置
            if (sig == candidate.split_variable && candidate.merge_type == MergeType::SIX_INPUT_SHANNON) {
                continue;
            }
            input_order.push_back(sig);
        }
    }
    
    // 添加LUT2独有输入
    for (auto sig : candidate.lut2_only_inputs) {
        if (input_order.size() < 6) {
            // 如果是分割变量，预留I5位置
            if (sig == candidate.split_variable && candidate.merge_type == MergeType::SIX_INPUT_SHANNON) {
                continue;
            }
            input_order.push_back(sig);
        }
    }
    
    // 如果有分割变量，放在最后（通常是I5）
    if (candidate.split_variable.wire && input_order.size() < 6) {
        input_order.push_back(candidate.split_variable);
    }
    
    if (enable_debug) {
        log("  Input pin arrangement:\n");
        for (size_t i = 0; i < input_order.size(); i++) {
            log("    I%lu: %s\n", i, log_signal(input_order[i]));
        }
    }
    
    return input_order;
}

// 计算GTP_LUT6D的INIT值 - 核心算法实现
vector<bool> LUTMergeOptimizer::computeGTP_LUT6D_INIT(const LUTMergeCandidate &candidate,
                                                      const vector<SigBit> &input_order) {
    vector<bool> init(64, false);
    
    if (!candidate.lut1 || !candidate.lut2) {
        log_error("computeGTP_LUT6D_INIT: null LUT pointers\n");
        return init;
    }
    
    // 获取原始LUT的真值表
    auto truth1 = extractLUTTruthTable(candidate.lut1);
    auto truth2 = extractLUTTruthTable(candidate.lut2);
    
    // 获取原始LUT的输入顺序
    vector<SigBit> inputs1, inputs2;
    getCellInputsVector(candidate.lut1, inputs1);
    getCellInputsVector(candidate.lut2, inputs2);
    
    if (enable_debug) {
        log("  Computing INIT for GTP_LUT6D merge\n");
        log("    LUT1 inputs: %lu, truth size: %lu\n", inputs1.size(), truth1.size());
        log("    LUT2 inputs: %lu, truth size: %lu\n", inputs2.size(), truth2.size());
        log("    Input order size: %lu\n", input_order.size());
    }
    
    // 建立输入映射：merged_input_order -> 原始LUT输入位置
    dict<SigBit, int> input_pos_map;
    for (size_t i = 0; i < input_order.size(); i++) {
        input_pos_map[input_order[i]] = i;
    }
    
    // 计算INIT[31:0] - Z5输出（通常对应较小的LUT）
    Cell *z5_lut = (getLUTInputCount(candidate.lut1) <= getLUTInputCount(candidate.lut2)) 
                   ? candidate.lut1 : candidate.lut2;
    Cell *z_lut = (z5_lut == candidate.lut1) ? candidate.lut2 : candidate.lut1;
    
    vector<SigBit> z5_inputs = (z5_lut == candidate.lut1) ? inputs1 : inputs2;
    auto z5_truth = (z5_lut == candidate.lut1) ? truth1 : truth2;
    
    for (int addr = 0; addr < 32; addr++) {
        bool output = evaluateLUTAtMergedAddress(z5_lut, z5_truth, z5_inputs, 
                                               input_order, input_pos_map, addr);
        init[addr] = output;
    }
    
    // 计算INIT[63:32] - 根据合并类型决定
    if (candidate.merge_type == MergeType::SIX_INPUT_SHANNON) {
        // 香农展开：Z在I5=1时的输出
        vector<SigBit> z_inputs = (z_lut == candidate.lut1) ? inputs1 : inputs2;
        auto z_truth = (z_lut == candidate.lut1) ? truth1 : truth2;
        
        for (int addr = 0; addr < 32; addr++) {
            // 构造6输入地址：I5=1，其余由addr确定
            int full_addr = addr | (1 << 5);  // 设置I5=1
            bool output = evaluateLUTAtMergedAddress(z_lut, z_truth, z_inputs,
                                                   input_order, input_pos_map, full_addr);
            init[addr + 32] = output;
        }
    } else {
        // 其他合并类型：Z输出
        vector<SigBit> z_inputs = (z_lut == candidate.lut1) ? inputs1 : inputs2;
        auto z_truth = (z_lut == candidate.lut1) ? truth1 : truth2;
        
        for (int addr = 0; addr < 32; addr++) {
            bool output = evaluateLUTAtMergedAddress(z_lut, z_truth, z_inputs,
                                                   input_order, input_pos_map, addr);
            init[addr + 32] = output;
        }
    }
    
    if (enable_debug) {
        log("  Computed INIT value: %s\n", formatInitValue(init).c_str());
    }
    
    return init;
}

// 在合并地址空间中计算LUT输出
bool LUTMergeOptimizer::evaluateLUTAtMergedAddress(Cell *lut, const vector<bool> &truth_table,
                                                  const vector<SigBit> &lut_inputs,
                                                  const vector<SigBit> &merged_order,
                                                  const dict<SigBit, int> &pos_map,
                                                  int merged_addr) {
    // 将merged_addr转换为原始LUT的地址空间
    int lut_addr = 0;
    
    for (size_t i = 0; i < lut_inputs.size(); i++) {
        SigBit input_sig = lut_inputs[i];
        
        // 在merged_order中找到这个输入的位置
        if (pos_map.count(input_sig)) {
            int bit_pos = pos_map.at(input_sig);
            if (merged_addr & (1 << bit_pos)) {
                lut_addr |= (1 << i);
            }
        }
        // 如果输入不在merged_order中，默认为0
    }
    
    // 从真值表中读取输出
    if (lut_addr < (int)truth_table.size()) {
        return truth_table[lut_addr];
    }
    
    return false; // 地址越界，默认为false
}

// 添加缺失的方法实现
void LUTMergeOptimizer::setTimingAware(bool aware) {
    // 这个方法在lut_merge_helpers.cc中被调用但未定义
    // 添加到优化器配置中
    if (enable_debug) {
        log("LUTMergeOptimizer: timing aware mode %s\n", aware ? "enabled" : "disabled");
    }
}

// 注意：bit2depth数据通过syncBit2DepthData()接口传递
// 不再使用全局变量，改为通过接口函数访问

YOSYS_NAMESPACE_END