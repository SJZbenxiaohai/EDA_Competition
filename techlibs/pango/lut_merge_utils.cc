// LUT合并工具函数实现
// GTP_LUT6D合并优化 - 工具和配置函数
// 创建时间：2025-09-24
// 版本：v2.0 - 重构版本，移除重复函数定义
// 作用：实现LUTMergeOptimizer类的工具和配置方法
//
// 注意：主要的INIT计算函数已移动到lut_merge_init.cc
// 本文件只保留不重复的工具函数

#include "lut_merge_pango.h"
#include "kernel/log.h"

YOSYS_NAMESPACE_BEGIN

// 设置时序感知模式
void LUTMergeOptimizer::setTimingAware(bool aware) {
    // 这个方法用于配置时序感知模式
    if (enable_debug) {
        log("LUTMergeOptimizer: timing aware mode %s\n", aware ? "enabled" : "disabled");
    }
}

// 注意：以下函数的实现已移动到其他专门文件：
// - arrangeInputPins() -> lut_merge_init.cc
// - computeGTP_LUT6D_INIT() -> lut_merge_init.cc
// - computeINIT_Shannon() -> lut_merge_init.cc
// - computeINIT_LogicContainment() -> lut_merge_init.cc
// - computeINIT_InputSubset() -> lut_merge_init.cc
// - computeINIT_PartialSharing() -> lut_merge_init.cc
// - computeINIT_IndependentReuse() -> lut_merge_init.cc
// - computeINIT_FunctionMux() -> lut_merge_init.cc

YOSYS_NAMESPACE_END