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

#ifndef LUT_MERGE_PANGO_H
#define LUT_MERGE_PANGO_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"

YOSYS_NAMESPACE_BEGIN

// 合并类型枚举（基于v1.2方案修正）
enum class MergeType {
    INVALID = 0,
    LOGIC_CONTAINMENT,        // 逻辑包含（最高优先级⭐⭐⭐⭐⭐）
    SIX_INPUT_SHANNON,        // 6输入香农展开（⭐⭐⭐⭐）
    SIX_INPUT_SHANNON_REVERSE,// 6输入香农展开反向（⭐⭐⭐⭐）
    INPUT_SUBSET,             // 输入子集关系（⭐⭐⭐）
    PARTIAL_SHARING_5INPUT,   // 5输入部分共享（⭐⭐⭐）
    INDEPENDENT_REUSE,        // 独立功能复用≤4输入（⭐⭐）
    FUNCTION_MULTIPLEXING     // 功能复用用I[5]选择（⭐）
};

// 合并候选结构（严格按照v1.2设计）
struct LUTMergeCandidate {
    // === 基本信息 ===
    Cell *lut1, *lut2;                    // 待合并的两个LUT
    MergeType merge_type;                 // 合并类型
    float total_benefit;                  // 总收益评分
    string failure_reason;               // 失败原因（调试用）
    
    // === 输入分析 ===
    pool<SigBit> shared_inputs;          // 共享输入
    pool<SigBit> lut1_only_inputs;       // 仅LUT1的输入
    pool<SigBit> lut2_only_inputs;       // 仅LUT2的输入
    int total_inputs;                    // 总输入数
    
    // === 合并策略 ===
    SigBit split_variable;               // 分裂变量（香农展开用）
    int split_bit_position;              // split_var在input_order中的位置
    bool split_polarity;                 // 分裂极性（0或1）
    Cell *z_lut, *z5_lut;               // Z和Z5输出分配
    string merge_strategy;               // 具体合并策略描述
    
    // === 时序信息 ===
    float timing_impact;                 // 时序影响评估
    float depth1, depth2;                // 原始LUT的逻辑深度
    
    // 构造函数
    LUTMergeCandidate() : lut1(nullptr), lut2(nullptr), 
        merge_type(MergeType::INVALID), total_benefit(0.0),
        total_inputs(0), split_bit_position(5), split_polarity(false),
        z_lut(nullptr), z5_lut(nullptr),
        timing_impact(0.0), depth1(0.0), depth2(0.0) {}
};

// LUT合并优化器主类
class LUTMergeOptimizer {
public:
    enum Strategy { CONSERVATIVE, BALANCED, AGGRESSIVE };
    
    LUTMergeOptimizer();
    ~LUTMergeOptimizer();
    
    // === 配置接口 ===
    void setStrategy(Strategy s) { strategy = s; }
    void setStrategy(const string &s);
    void setBenefitThreshold(float t) { benefit_threshold = t; }
    void setMaxIterations(int n) { max_iterations = n; }
    void setDebugOutput(bool d) { enable_debug = d; }
    void setTimingAware(bool aware);
    void setBit2DepthRef(dict<SigBit, float> &depth_map) { 
        bit2depth_ref = &depth_map; 
    }
    
    // === 主优化接口 ===
    bool optimize(Module *module);
    
    // === 统计信息接口 ===
    int getSuccessfulMerges() const { return successful_merges; }
    int getInitialLUTCount() const { return initial_lut_count; }
    int getFinalLUTCount() const { return final_lut_count; }
    dict<MergeType, int> getMergeTypeBreakdown() const { 
        return merge_type_count; 
    }
    
    // === 工具方法 ===
    string getMergeTypeString(MergeType type);
    
    // 香农展开分析结构体
    struct ShannonSplitAnalysis {
        vector<SigBit> z_inputs;          // z_lut的输入向量
        vector<SigBit> z5_inputs;         // z5_lut的输入向量
        vector<SigBit> reduced_inputs;    // 去掉split_var后的输入
        int split_pos;                    // split_var在z_inputs中的位置
        
        ShannonSplitAnalysis() : split_pos(-1) {}
    };
    
private:
    // === 配置参数 ===
    Strategy strategy;
    float benefit_threshold;
    int max_iterations;
    bool enable_debug;
    
    // === 外部数据引用 ===
    dict<SigBit, float> *bit2depth_ref;  // 时序数据引用
    
    // === 运行时数据 ===
    Module *current_module;              // 当前处理的模块
    SigMap sigmap;                       // 信号映射
    
    // === 统计信息 ===
    int initial_lut_count;
    int final_lut_count;
    int successful_merges;
    dict<MergeType, int> merge_type_count; // 各类型合并统计
    
    // === 核心算法接口（在各个.cc文件中实现）===
    
    // lut_merge_analyzer.cc中实现
    bool identifyMergeCandidates(vector<LUTMergeCandidate> &candidates);
    bool analyzeMergeCandidate(Cell *lut1, Cell *lut2, 
                               LUTMergeCandidate &candidate);
    bool analyzeInputRelationships(const vector<SigBit> &lut1_inputs,
                                  const vector<SigBit> &lut2_inputs,
                                  LUTMergeCandidate &candidate);
    bool checkBasicMergeConstraints(const LUTMergeCandidate &candidate);
    bool evaluateTimingImpact(LUTMergeCandidate &candidate);
    
    // lut_merge_types.cc中实现
    MergeType determineMergeType(LUTMergeCandidate &candidate);
    bool checkBasicConstraints(const LUTMergeCandidate &candidate);
    
    // ✅ Bug 2.4修复：辅助判断函数声明
    bool checkLogicContainment(LUTMergeCandidate &candidate);
    bool checkInputSubsetRelation(LUTMergeCandidate &candidate);  
    bool checkPartialSharingFeasibility(LUTMergeCandidate &candidate);
    SigBit findOptimalSplitVariable(const LUTMergeCandidate &candidate);
    bool checkBasicMergeConstraints(LUTMergeCandidate &candidate);
    
    // lut_merge_types.cc中实现的辅助函数
    bool checkLogicalContainmentCore(const vector<bool> &contained_truth,
                                    const vector<bool> &container_truth,
                                    const vector<SigBit> &contained_inputs,
                                    const vector<SigBit> &container_inputs,
                                    bool reverse_role);
    SigBit selectBestSplitFromCandidates(const LUTMergeCandidate &candidate,
                                        const pool<SigBit> &candidates, 
                                        bool prefer_lut1);
    
    // lut_merge_shannon.cc中实现
    bool verifyShannonExpansion(const LUTMergeCandidate &candidate, 
                               SigBit split_var);
    bool verifyLogicalEquivalence(const vector<bool> &truth1,
                                 const vector<bool> &truth2,
                                 const vector<SigBit> &inputs1,
                                 const vector<SigBit> &inputs2,
                                 int split_pos);
    
    // 香农展开辅助函数
    bool verifyShannonConditions(const LUTMergeCandidate &candidate, SigBit split_var);
    bool analyzeShannonSplit(const LUTMergeCandidate &candidate, SigBit split_var, 
                            ShannonSplitAnalysis &split_analysis);
    bool extractTruthTableWithValidation(Cell *lut, vector<bool> &truth_table, int expected_size);
    void debugLogicalEquivalenceFailure(int combo, int addr1, int addr2,
                                        const vector<SigBit> &inputs1, const vector<SigBit> &inputs2,
                                        int split_pos, const dict<SigBit, int> &map1, 
                                        const dict<SigBit, int> &map2_reduced);
    void debugShannonExpansion(const LUTMergeCandidate &candidate, SigBit split_var,
                              const ShannonSplitAnalysis &split_analysis);
    
    // lut_merge_init.cc中实现
    vector<bool> computeGTP_LUT6D_INIT(const LUTMergeCandidate &candidate,
                                      const vector<SigBit> &input_order);
    vector<SigBit> arrangeInputPins(const LUTMergeCandidate &candidate);
    
    // ✅ Bug 2.3修复：各种合并类型的INIT计算方法
    vector<bool> computeINIT_Shannon(const LUTMergeCandidate &candidate, 
                                    const vector<SigBit> &input_order);
    vector<bool> computeINIT_LogicContainment(const LUTMergeCandidate &candidate,
                                             const vector<SigBit> &input_order);
    vector<bool> computeINIT_InputSubset(const LUTMergeCandidate &candidate,
                                        const vector<SigBit> &input_order);
    vector<bool> computeINIT_PartialSharing(const LUTMergeCandidate &candidate,
                                           const vector<SigBit> &input_order);
    vector<bool> computeINIT_IndependentReuse(const LUTMergeCandidate &candidate,
                                             const vector<SigBit> &input_order);
    vector<bool> computeINIT_FunctionMux(const LUTMergeCandidate &candidate,
                                        const vector<SigBit> &input_order);
    
    // INIT计算辅助函数
    vector<SigBit> arrangePinsForShannon(const LUTMergeCandidate &candidate,
                                         const pool<SigBit> &all_inputs);
    vector<SigBit> arrangePinsForLogicContainment(const LUTMergeCandidate &candidate,
                                                  const pool<SigBit> &all_inputs);
    vector<SigBit> arrangePinsForInputSubset(const LUTMergeCandidate &candidate,
                                            const pool<SigBit> &all_inputs);
    vector<SigBit> arrangePinsForPartialSharing(const LUTMergeCandidate &candidate,
                                                const pool<SigBit> &all_inputs);
    vector<SigBit> arrangePinsForGeneralCase(const LUTMergeCandidate &candidate,
                                            const pool<SigBit> &all_inputs);
    int getSignalPriority(const SigBit &signal);
    bool computeLUTOutputAtMergedAddress(Cell *lut, const vector<bool> &truth_table,
                                        const vector<SigBit> &lut_inputs,
                                        const vector<SigBit> &merged_order,
                                        const dict<SigBit, int> &merged_pos_map,
                                        int merged_addr);
    void debugINITValue(const vector<bool> &init);
    
    // lut_merge_executor.cc中实现
    bool executeSingleMerge(const LUTMergeCandidate &candidate);
    vector<LUTMergeCandidate> selectOptimalMatching(
        const vector<LUTMergeCandidate> &candidates);
    
    // executor辅助函数
    Cell* createGTP_LUT6D(const LUTMergeCandidate &candidate,
                         const vector<SigBit> &input_order,
                         const vector<bool> &init_value);
    bool updateMergedConnections(const LUTMergeCandidate &candidate,
                               Cell *merged_lut,
                               const vector<SigBit> &input_order);
    bool cleanupOriginalLUTs(const LUTMergeCandidate &candidate);
    
    // === 通用辅助函数 ===
    bool isSingleOutputLUT(Cell *cell);
    bool isGTP_LUT6D(Cell *cell);
    int getLUTInputCount(Cell *cell);
    void getCellInputsVector(Cell *cell, vector<SigBit> &inputs);
    SigBit getCellOutput(Cell *cell);
    vector<bool> extractLUTTruthTable(Cell *lut);
    float calculateMergeBenefit(const LUTMergeCandidate &candidate);
    
    // === 日志和调试辅助 ===
    int countLUTs(Module *module);
    bool validateLUTCount(Module *module);  // ✅ Bug 2.1修复：验证LUT统计一致性
    void printCandidateInfo(const LUTMergeCandidate &candidate);
    
    // === 收敛性控制 ===
    bool hasConverged(int prev_lut_count, int current_lut_count);
    void updateIterationStats(const vector<LUTMergeCandidate> &selected);
    void generateOptimizationReport();
};

// === 全局辅助函数 ===
string formatInitValue(const vector<bool> &init);
bool isValidMergeType(MergeType type);
float getMergeTypePriority(MergeType type);

// === synth_pango.cc接口函数 ===
// 这些函数在lut_merge_interface.cc中实现，供synth_pango.cc调用

// 帮助信息和参数处理
void printLUTMergeHelp();
void printLUTMergeExamples();
bool parseLUTMergeArgs(const std::vector<std::string> &args, size_t &argidx);
bool validateLUTMergeConfig();
void clearLUTMergeFlags();

// 数据同步和主优化接口
void syncBit2DepthData(const dict<SigBit, float> &bit2depth_source);
bool checkAndRunLUTMerge(const std::string &module_name, RTLIL::Module *module);

// 配置查询接口
bool isLUTMergeEnabled();
std::string getLUTMergeStrategy();
float getLUTMergeThreshold();
bool isLUTMergeDebugEnabled();
int getLUTMergeMaxIterations();
bool isLUTMergeTimingAware();

// 类型转换辅助函数
std::string getMergeTypeString(MergeType type);

YOSYS_NAMESPACE_END

#endif // LUT_MERGE_PANGO_H