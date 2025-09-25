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
 * LUT合并接口函数实现 - synth_pango.cc接口层
 * 
 * 功能：为synth_pango.cc提供LUT合并功能的接口函数
 * - 命令行帮助和参数解析
 * - 全局配置管理和标志清理
 * - bit2depth数据同步机制
 * - 主优化流程调用接口
 * 
 * 创建时间：2025-09-24
 * 版本：v1.0 (修复链接错误版本)
 */

#include "lut_merge_pango.h"
#include "kernel/log.h"
#include "kernel/yosys.h"

YOSYS_NAMESPACE_BEGIN

// ===== 全局配置变量 =====
// LUT合并功能全局开关和配置参数

namespace {
    // LUT合并配置参数（内部使用，避免全局污染）
    struct LUTMergeGlobalConfig {
        bool enable_lut_merge = false;
        std::string merge_strategy = "balanced";
        float merge_threshold = 3.0;
        bool debug_output = false;
        int max_iterations = 3;
        bool timing_aware = true;
        
        void reset() {
            enable_lut_merge = false;
            merge_strategy = "balanced";
            merge_threshold = 3.0;
            debug_output = false;
            max_iterations = 3;
            timing_aware = true;
        }
    } lut_merge_config;
    
    // bit2depth数据存储（临时方案）
    dict<SigBit, float> global_bit2depth_data;
}

// ===== 帮助信息接口函数 =====

void printLUTMergeHelp() {
    log("\n");
    log("LUT Merge Optimization Options:\n");
    log("    -enable_lut_merge\n");
    log("        enable LUT merge optimization for GTP_LUT6D (default: disabled)\n");
    log("\n");
    log("    -lut_merge_strategy <strategy>\n");
    log("        set merge strategy: conservative, balanced, aggressive (default: balanced)\n");
    log("        conservative: only high-confidence merges, preserve timing\n");
    log("        balanced: moderate optimization with timing consideration\n");
    log("        aggressive: maximum LUT reduction, may impact timing\n");
    log("\n");
    log("    -lut_merge_threshold <float>\n");
    log("        set minimum benefit threshold for merging (default: 3.0)\n");
    log("        higher values = fewer but safer merges\n");
    log("\n");
    log("    -lut_merge_debug\n");
    log("        enable detailed debug output for LUT merge process\n");
    log("\n");
    log("    -lut_merge_max_iterations <int>\n");
    log("        set maximum optimization iterations (default: 3, minimum: 1)\n");
    log("\n");
    log("    -lut_merge_timing_aware\n");
    log("        enable timing-aware optimization (default: enabled)\n");
}

void printLUTMergeExamples() {
    log("\n");
    log("LUT Merge Usage Examples:\n");
    log("    synth_pango -top design -input design.v -enable_lut_merge\n");
    log("        basic LUT merge with default settings\n");
    log("\n");
    log("    synth_pango -top design -input design.v -enable_lut_merge \\\n");
    log("                -lut_merge_strategy aggressive -lut_merge_threshold 2.0\n");
    log("        aggressive optimization with lower threshold\n");
    log("\n");
    log("    synth_pango -top design -input design.v -enable_lut_merge \\\n");
    log("                -lut_merge_debug -lut_merge_max_iterations 5\n");
    log("        debug mode with extended iterations\n");
    log("\n");
    log("    synth_pango -top design -input design.v -enable_lut_merge \\\n");
    log("                -run begin:lut_merge\n");
    log("        run only up to LUT merge stage\n");
}

// ===== 配置管理接口函数 =====

void clearLUTMergeFlags() {
    lut_merge_config.reset();
    global_bit2depth_data.clear();
    
    if (lut_merge_config.debug_output) {
        log("LUTMerge: flags and data cleared\n");
    }
}

bool parseLUTMergeArgs(const std::vector<std::string> &args, size_t &argidx) {
    const std::string &arg = args[argidx];
    
    if (arg == "-enable_lut_merge") {
        lut_merge_config.enable_lut_merge = true;
        return true;
    }
    
    if (arg == "-lut_merge_strategy" && argidx + 1 < args.size()) {
        std::string strategy = args[++argidx];
        if (strategy == "conservative" || strategy == "balanced" || strategy == "aggressive") {
            lut_merge_config.merge_strategy = strategy;
        } else {
            log_error("Invalid LUT merge strategy '%s'. Valid options: conservative, balanced, aggressive\n", 
                     strategy.c_str());
        }
        return true;
    }
    
    if (arg == "-lut_merge_threshold" && argidx + 1 < args.size()) {
        float threshold = atof(args[++argidx].c_str());
        if (threshold >= 0.0) {
            lut_merge_config.merge_threshold = threshold;
        } else {
            log_error("Invalid LUT merge threshold '%s'. Must be >= 0.0\n", args[argidx].c_str());
        }
        return true;
    }
    
    if (arg == "-lut_merge_debug") {
        lut_merge_config.debug_output = true;
        return true;
    }
    
    if (arg == "-lut_merge_max_iterations" && argidx + 1 < args.size()) {
        int iterations = atoi(args[++argidx].c_str());
        if (iterations >= 1) {
            lut_merge_config.max_iterations = iterations;
        } else {
            log_error("Invalid max iterations '%s'. Must be >= 1\n", args[argidx].c_str());
        }
        return true;
    }
    
    if (arg == "-lut_merge_timing_aware") {
        lut_merge_config.timing_aware = true;
        return true;
    }
    
    return false; // 不是LUT merge相关的参数
}

bool validateLUTMergeConfig() {
    if (!lut_merge_config.enable_lut_merge) {
        return true; // 未启用，无需验证
    }
    
    // 验证策略
    if (lut_merge_config.merge_strategy != "conservative" && 
        lut_merge_config.merge_strategy != "balanced" && 
        lut_merge_config.merge_strategy != "aggressive") {
        log_error("Invalid merge strategy: %s\n", lut_merge_config.merge_strategy.c_str());
        return false;
    }
    
    // 验证阈值
    if (lut_merge_config.merge_threshold < 0.0) {
        log_error("Invalid merge threshold: %.2f (must be >= 0.0)\n", 
                 lut_merge_config.merge_threshold);
        return false;
    }
    
    // 验证迭代次数
    if (lut_merge_config.max_iterations < 1) {
        log_error("Invalid max iterations: %d (must be >= 1)\n", 
                 lut_merge_config.max_iterations);
        return false;
    }
    
    if (lut_merge_config.debug_output) {
        log("LUTMerge: configuration validated successfully\n");
        log("  Strategy: %s\n", lut_merge_config.merge_strategy.c_str());
        log("  Threshold: %.2f\n", lut_merge_config.merge_threshold);
        log("  Max iterations: %d\n", lut_merge_config.max_iterations);
        log("  Timing aware: %s\n", lut_merge_config.timing_aware ? "enabled" : "disabled");
    }
    
    return true;
}

// ===== bit2depth数据同步接口 =====

void syncBit2DepthData(const dict<SigBit, float> &bit2depth_source) {
    global_bit2depth_data.clear();
    
    for (const auto &pair : bit2depth_source) {
        global_bit2depth_data[pair.first] = pair.second;
    }
    
    if (lut_merge_config.debug_output) {
        log("LUTMerge: synchronized %lu bit2depth entries\n", global_bit2depth_data.size());
    }
}

// ===== 主优化流程接口 =====

bool checkAndRunLUTMerge(const std::string &module_name, RTLIL::Module *module) {
    if (!lut_merge_config.enable_lut_merge) {
        if (lut_merge_config.debug_output) {
            log("LUTMerge: optimization disabled, skipping\n");
        }
        return true;
    }
    
    if (!module) {
        log_error("LUTMerge: null module pointer\n");
        return false;
    }
    
    log("=== Running LUT merge optimization (v1.2) ===\n");
    log("Module: %s\n", module_name.c_str());
    log("Strategy: %s, Threshold: %.2f, Debug: %s\n",
        lut_merge_config.merge_strategy.c_str(), 
        lut_merge_config.merge_threshold,
        lut_merge_config.debug_output ? "ON" : "OFF");
    
    try {
        // 创建和配置优化器
        LUTMergeOptimizer optimizer;
        
        // 设置配置参数
        if (lut_merge_config.merge_strategy == "conservative") {
            optimizer.setStrategy(LUTMergeOptimizer::CONSERVATIVE);
        } else if (lut_merge_config.merge_strategy == "balanced") {
            optimizer.setStrategy(LUTMergeOptimizer::BALANCED);
        } else if (lut_merge_config.merge_strategy == "aggressive") {
            optimizer.setStrategy(LUTMergeOptimizer::AGGRESSIVE);
        }
        
        optimizer.setBenefitThreshold(lut_merge_config.merge_threshold);
        optimizer.setMaxIterations(lut_merge_config.max_iterations);
        optimizer.setDebugOutput(lut_merge_config.debug_output);
        
        // 传递bit2depth数据
        optimizer.setBit2DepthRef(global_bit2depth_data);
        
        // 执行优化
        bool success = optimizer.optimize(module);
        
        if (success) {
            log("LUT merge optimization completed successfully\n");
            log("Successful merges: %d\n", optimizer.getSuccessfulMerges());
            
            // 显示合并类型统计
            auto type_breakdown = optimizer.getMergeTypeBreakdown();
            if (!type_breakdown.empty()) {
                log("Merge type breakdown:\n");
                for (const auto &pair : type_breakdown) {
                    log("  %s: %d\n", 
                        getMergeTypeString(pair.first).c_str(), pair.second);
                }
            }
        } else {
            log("No beneficial LUT merges found\n");
        }
        
        return true;
        
    } catch (const std::exception &e) {
        log_error("LUT merge optimization failed: %s\n", e.what());
        return false;
    } catch (...) {
        log_error("LUT merge optimization failed: unknown exception\n");
        return false;
    }
}

// ===== 辅助函数 =====

std::string getMergeTypeString(MergeType type) {
    switch (type) {
        case MergeType::LOGIC_CONTAINMENT:      return "Logic Containment";
        case MergeType::SIX_INPUT_SHANNON:      return "Shannon Expansion (6-input)";
        case MergeType::SIX_INPUT_SHANNON_REVERSE: return "Shannon Expansion Reverse";
        case MergeType::INPUT_SUBSET:           return "Input Subset";
        case MergeType::PARTIAL_SHARING_5INPUT: return "Partial Sharing (5-input)";
        case MergeType::INDEPENDENT_REUSE:      return "Independent Reuse";
        case MergeType::FUNCTION_MULTIPLEXING:  return "Function Multiplexing";
        case MergeType::INVALID:
        default:                                return "Invalid";
    }
}

// ===== 配置查询接口（供其他模块使用）=====

bool isLUTMergeEnabled() {
    return lut_merge_config.enable_lut_merge;
}

std::string getLUTMergeStrategy() {
    return lut_merge_config.merge_strategy;
}

float getLUTMergeThreshold() {
    return lut_merge_config.merge_threshold;
}

bool isLUTMergeDebugEnabled() {
    return lut_merge_config.debug_output;
}

int getLUTMergeMaxIterations() {
    return lut_merge_config.max_iterations;
}

bool isLUTMergeTimingAware() {
    return lut_merge_config.timing_aware;
}

YOSYS_NAMESPACE_END