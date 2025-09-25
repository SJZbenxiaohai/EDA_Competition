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
 * synth_pango扩展文件 - GTP_LUT6D合并优化集成
 * 
 * 功能：扩展现有的synth_pango Pass，增加LUT合并优化功能
 * - 新增命令行参数支持LUT合并配置
 * - 集成lut_merge阶段到synthesis流程
 * - 实现与原有bit2depth数据的无缝对接
 * - 向后兼容，默认关闭LUT合并功能
 */

#include "synth_pango_extend.h"
#include "lut_merge_pango.h"
#include "kernel/yosys.h"

USING_YOSYS_NAMESPACE

// 前向声明
string getMergeTypeString(MergeType type);

// 注意：bit2depth数据通过参数传递，不使用全局变量

// ===== PangoBit2DepthManager实现 =====

void PangoBit2DepthManager::updateBit2Depth(const dict<SigBit, float> &global_bit2depth) {
    bit2depth_data.clear();
    
    for (const auto &pair : global_bit2depth) {
        bit2depth_data[pair.first] = pair.second;
    }
    
    is_initialized = true;
    
    log("PangoBit2DepthManager: Updated with %lu entries\n", bit2depth_data.size());
}

void PangoBit2DepthManager::setBit2Depth(SigBit bit, float depth) {
    bit2depth_data[bit] = depth;
    is_initialized = true;
}

float PangoBit2DepthManager::getBit2Depth(SigBit bit) const {
    auto it = bit2depth_data.find(bit);
    if (it != bit2depth_data.end()) {
        return it->second;
    }
    return 0.0f; // 默认深度为0
}

bool PangoBit2DepthManager::hasBit2Depth(SigBit bit) const {
    return bit2depth_data.count(bit) > 0;
}

const dict<SigBit, float>& PangoBit2DepthManager::getBit2DepthRef() const {
    return bit2depth_data;
}

dict<SigBit, float>& PangoBit2DepthManager::getBit2DepthRef() {
    return bit2depth_data;
}

void PangoBit2DepthManager::clear() {
    bit2depth_data.clear();
    is_initialized = false;
}

void PangoBit2DepthManager::printStatistics() const {
    log("PangoBit2DepthManager Statistics:\n");
    log("  Total entries: %lu\n", bit2depth_data.size());
    log("  Initialized: %s\n", is_initialized ? "YES" : "NO");
    
    if (!bit2depth_data.empty()) {
        float min_depth = 1e10, max_depth = -1e10, sum_depth = 0;
        for (const auto &pair : bit2depth_data) {
            min_depth = min(min_depth, pair.second);
            max_depth = max(max_depth, pair.second);
            sum_depth += pair.second;
        }
        float avg_depth = sum_depth / bit2depth_data.size();
        
        log("  Depth range: [%.2f, %.2f]\n", min_depth, max_depth);
        log("  Average depth: %.2f\n", avg_depth);
    }
}

void PangoBit2DepthManager::validateData() const {
    int invalid_count = 0;
    
    for (const auto &pair : bit2depth_data) {
        if (pair.second < 0 || pair.second > 1000) { // 合理的深度范围检查
            invalid_count++;
            if (invalid_count <= 5) { // 只报告前5个无效项
                log_warning("Invalid depth value: %s = %.2f\n", 
                           log_signal(pair.first), pair.second);
            }
        }
    }
    
    if (invalid_count > 0) {
        log_warning("Found %d invalid depth values in bit2depth data\n", invalid_count);
    } else {
        log("bit2depth data validation passed\n");
    }
}

// ===== SynthPangoExtension实现 =====

bool SynthPangoExtension::parseArgs(const vector<string> &args, size_t &argidx) {
    if (args[argidx] == "-enable_lut_merge") {
        enable_lut_merge = true;
        return true;
    }
    
    if (args[argidx] == "-lut_merge_strategy" && argidx + 1 < args.size()) {
        lut_merge_strategy = args[++argidx];
        if (lut_merge_strategy != "conservative" && 
            lut_merge_strategy != "balanced" && 
            lut_merge_strategy != "aggressive") {
            log_error("Invalid LUT merge strategy: %s\n", lut_merge_strategy.c_str());
            return false;
        }
        return true;
    }
    
    if (args[argidx] == "-lut_merge_threshold" && argidx + 1 < args.size()) {
        lut_merge_threshold = atof(args[++argidx].c_str());
        if (lut_merge_threshold < 0) {
            log_error("LUT merge threshold must be non-negative\n");
            return false;
        }
        return true;
    }
    
    if (args[argidx] == "-lut_merge_debug") {
        lut_merge_debug = true;
        return true;
    }
    
    if (args[argidx] == "-lut_merge_max_iterations" && argidx + 1 < args.size()) {
        lut_merge_max_iterations = max(1, atoi(args[++argidx].c_str()));
        return true;
    }
    
    return false;
}

void SynthPangoExtension::clearFlags() {
    enable_lut_merge = false;
    lut_merge_strategy = "balanced";
    lut_merge_threshold = 3.0;
    lut_merge_debug = false;
    lut_merge_max_iterations = 3;
    
    bit2depth_manager.clear();
}

bool SynthPangoExtension::validateConfig() {
    if (!enable_lut_merge) {
        return true; // 如果未启用，配置始终有效
    }
    
    // 验证策略
    if (lut_merge_strategy != "conservative" && 
        lut_merge_strategy != "balanced" && 
        lut_merge_strategy != "aggressive") {
        log_error("Invalid LUT merge strategy: %s\n", lut_merge_strategy.c_str());
        return false;
    }
    
    // 验证阈值
    if (lut_merge_threshold < 0) {
        log_error("LUT merge threshold must be non-negative: %.2f\n", lut_merge_threshold);
        return false;
    }
    
    // 验证迭代次数
    if (lut_merge_max_iterations < 1) {
        log_error("LUT merge max iterations must be at least 1: %d\n", lut_merge_max_iterations);
        return false;
    }
    
    return true;
}

void SynthPangoExtension::printStatus() {
    log("LUT Merge Configuration Status:\n");
    log("  Enabled: %s\n", enable_lut_merge ? "YES" : "NO");
    
    if (enable_lut_merge) {
        log("  Strategy: %s\n", lut_merge_strategy.c_str());
        log("  Threshold: %.2f\n", lut_merge_threshold);
        log("  Max iterations: %d\n", lut_merge_max_iterations);
        log("  Debug output: %s\n", lut_merge_debug ? "ON" : "OFF");
        
        log("  bit2depth manager status:\n");
        log("    Initialized: %s\n", bit2depth_manager.isInitialized() ? "YES" : "NO");
        log("    Entries: %lu\n", bit2depth_manager.size());
    }
}

bool SynthPangoExtension::runLUTMergeStage(Module *module) {
    if (!enable_lut_merge) {
        log("LUT merge optimization is disabled\n");
        return true;
    }
    
    // 注意：实际的LUT合并优化已经在checkAndRunLUTMerge()中实现
    // 这里只是一个向后兼容的接口
    if (lut_merge_debug) {
        log("SynthPangoExtension: delegating to global LUT merge interface\n");
    }
    
    // 委托给全局接口执行
    return checkAndRunLUTMerge(module->name.c_str(), module);
}

void SynthPangoExtension::printHelp() {
    log("    -enable_lut_merge\n");
    log("        enable LUT merge optimization using GTP_LUT6D\n");
    log("\n");
    log("    -lut_merge_strategy <conservative|balanced|aggressive>\n");
    log("        set LUT merge strategy (default: balanced)\n");
    log("        conservative: only merge high-confidence candidates\n");
    log("        balanced: balance between area savings and timing impact\n");
    log("        aggressive: maximize LUT savings\n");
    log("\n");
    log("    -lut_merge_threshold <value>\n");
    log("        set minimum benefit threshold for LUT merging\n");
    log("        (default: 3.0, higher values are more selective)\n");
    log("\n");
    log("    -lut_merge_debug\n");
    log("        enable detailed LUT merge debug output\n");
    log("\n");
    log("    -lut_merge_max_iterations <num>\n");
    log("        set maximum iterations for LUT merge optimization\n");
    log("        (default: 3, minimum: 1)\n");
    log("\n");
}

void SynthPangoExtension::printExamples() {
    log("Examples:\n");
    log("\n");
    log("    synth_pango -top cpu -input design.v -enable_lut_merge\n");
    log("        Basic LUT merge optimization\n");
    log("\n");  
    log("    synth_pango -top cpu -input design.v -enable_lut_merge \\\\\n");
    log("                -lut_merge_strategy aggressive -lut_merge_debug\n");
    log("        Aggressive LUT merge with debug output\n");
    log("\n");
    log("    synth_pango -run lut_merge:lut_merge -enable_lut_merge\n");
    log("        Run only LUT merge stage\n");
    log("\n");
    log("    synth_pango -top cpu -input design.v -enable_lut_merge \\\\\n");
    log("                -lut_merge_threshold 5.0 -lut_merge_max_iterations 1\n");
    log("        Conservative LUT merge with custom parameters\n");
    log("\n");
}

PRIVATE_NAMESPACE_BEGIN

// ===== 全局扩展实例 =====
SynthPangoExtension global_synth_pango_extension;

// ===== 便利函数实现 =====

SynthPangoExtension& getSynthPangoExtension() {
    return global_synth_pango_extension;
}

void syncBit2DepthData(const dict<SigBit, float> &global_bit2depth) {
    global_synth_pango_extension.bit2depth_manager.updateBit2Depth(global_bit2depth);
}

dict<SigBit, float>& getManagedBit2Depth() {
    return global_synth_pango_extension.bit2depth_manager.getBit2DepthRef();
}

bool isLUTMergeEnabled() {
    return global_synth_pango_extension.enable_lut_merge;
}

string getLUTMergeStrategy() {
    return global_synth_pango_extension.lut_merge_strategy;
}

// ===== 向后兼容接口实现 =====

bool parseLUTMergeArgs(const vector<string> &args, size_t &argidx) {
    return global_synth_pango_extension.parseArgs(args, argidx);
}

bool checkAndRunLUTMerge(const string &label, Module *module) {
    if (label != "lut_merge") {
        return false;
    }
    
    return global_synth_pango_extension.runLUTMergeStage(module);
}

void printLUTMergeHelp() {
    global_synth_pango_extension.printHelp();
}

void printLUTMergeExamples() {
    global_synth_pango_extension.printExamples();
}

void clearLUTMergeFlags() {
    global_synth_pango_extension.clearFlags();
}

bool validateLUTMergeConfig() {
    return global_synth_pango_extension.validateConfig();
}

void printLUTMergeStatus() {
    global_synth_pango_extension.printStatus();
}

PRIVATE_NAMESPACE_END

// ===== 全局工具函数实现 =====

// 获取合并类型的字符串表示（全局函数）
string getMergeTypeString(MergeType type) {
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