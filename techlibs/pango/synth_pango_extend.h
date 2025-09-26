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
 * ⚠️  DEPRECATED - 此文件已停用 ⚠️ 
 * 
 * 状态: 已停用，保留备用
 * 停用日期: 2025-09-26
 * 停用原因: 架构重构，功能已迁移到synth_pango.cc的SynthPangoPass类
 * 替代方案: 直接在synth_pango.cc中实现LUT合并功能
 * 
 * 注意: 此文件暂时保留用于:
 * 1. 代码回滚备用
 * 2. 功能实现参考
 * 3. 接口设计参考
 * 
 * 如需重新启用，请联系开发团队
 */

#ifndef SYNTH_PANGO_EXTEND_H
#define SYNTH_PANGO_EXTEND_H

#include "kernel/yosys.h"

YOSYS_NAMESPACE_BEGIN

// ===== 前向声明 =====
class Module;

// ===== LUT合并扩展接口 =====

// 命令行参数解析扩展
bool parseLUTMergeArgs(const vector<string> &args, size_t &argidx);

// Script执行扩展
bool checkAndRunLUTMerge(const string &label, Module *module);

// 帮助文档扩展
void printLUTMergeHelp();
void printLUTMergeExamples();

// 配置管理扩展
void clearLUTMergeFlags();
bool validateLUTMergeConfig();
void printLUTMergeStatus();

// ===== bit2depth数据管理类 =====

class PangoBit2DepthManager {
private:
    dict<SigBit, float> bit2depth_data;
    bool is_initialized;
    
public:
    PangoBit2DepthManager() : is_initialized(false) {}
    
    // 初始化和更新数据
    void updateBit2Depth(const dict<SigBit, float> &global_bit2depth);
    void setBit2Depth(SigBit bit, float depth);
    
    // 数据访问
    float getBit2Depth(SigBit bit) const;
    bool hasBit2Depth(SigBit bit) const;
    const dict<SigBit, float>& getBit2DepthRef() const;
    dict<SigBit, float>& getBit2DepthRef();
    
    // 状态管理
    bool isInitialized() const { return is_initialized; }
    void clear();
    size_t size() const { return bit2depth_data.size(); }
    
    // 调试和统计
    void printStatistics() const;
    void validateData() const;
};

// ===== 全局管理器实例 =====
extern PangoBit2DepthManager global_bit2depth_manager;

// ===== SynthPangoPass扩展接口 =====

struct SynthPangoExtension {
    // LUT合并配置参数
    bool enable_lut_merge;
    string lut_merge_strategy;
    float lut_merge_threshold;
    bool lut_merge_debug;
    int lut_merge_max_iterations;
    
    // bit2depth管理器
    PangoBit2DepthManager bit2depth_manager;
    
    // 构造函数
    SynthPangoExtension() : 
        enable_lut_merge(false),
        lut_merge_strategy("balanced"),
        lut_merge_threshold(3.0),
        lut_merge_debug(false),
        lut_merge_max_iterations(3) {}
    
    // 参数解析
    bool parseArgs(const vector<string> &args, size_t &argidx);
    
    // 配置管理
    void clearFlags();
    bool validateConfig();
    void printStatus();
    
    // 执行接口
    bool runLUTMergeStage(Module *module);
    
    // 帮助文档
    void printHelp();
    void printExamples();
};

// ===== 便利函数 =====

// 获取当前扩展实例
SynthPangoExtension& getSynthPangoExtension();

// bit2depth数据同步
void syncBit2DepthData(const dict<SigBit, float> &global_bit2depth);
dict<SigBit, float>& getManagedBit2Depth();

// LUT合并状态查询
bool isLUTMergeEnabled();
string getLUTMergeStrategy();

YOSYS_NAMESPACE_END

#endif // SYNTH_PANGO_EXTEND_H