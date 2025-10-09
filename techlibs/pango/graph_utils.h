/*
 * graph_utils.h
 *
 * GraphUtils - 图工具类
 * 提供统一的图数据访问、拓扑排序和图遍历功能
 *
 * 功能：
 * - 拓扑排序（Kahn算法）
 * - 图查询（driver、reader、inputs、output）
 * - 图遍历（BFS、DFS）
 *
 * 作者：根据8号文档Section 3.1实现
 * 日期：2025-10-04
 */

#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "kernel/yosys.h"
#include "kernel/sigtools.h"
#include "kernel/celltypes.h"
#include <queue>
#include <functional>

YOSYS_NAMESPACE_BEGIN

class GraphUtils {
public:
	// ===== 构造和初始化 =====

	GraphUtils(Module *module, SigMap &sigmap);

	// ===== 拓扑排序接口 =====

	/**
	 * 获取正向拓扑序（从输入到输出）
	 * @return 信号的拓扑排序向量
	 * @note 只计算一次，后续调用返回缓存结果
	 */
	const vector<SigBit>& getTopologicalOrder();

	/**
	 * 获取反向拓扑序（从输出到输入）
	 * @return 信号的反向拓扑排序向量
	 * @note 反向序 = 正向序的逆序
	 */
	const vector<SigBit>& getReverseTopologicalOrder();

	// ===== 图查询接口 =====

	/**
	 * 获取信号的驱动门
	 * @param signal 信号
	 * @return 驱动此信号的门，如果没有则返回nullptr
	 */
	Cell* getDriver(SigBit signal) const;

	/**
	 * 获取信号的所有读者（扇出门）
	 * @param signal 信号
	 * @return 读取此信号的所有门的向量
	 */
	const vector<Cell*>& getReaders(SigBit signal) const;

	/**
	 * 获取门的输入信号
	 * @param cell 门单元
	 * @return 所有输入信号的向量
	 */
	vector<SigBit> getCellInputs(Cell *cell) const;

	/**
	 * 获取门的输出信号
	 * @param cell 门单元
	 * @return 输出信号（假设单输出）
	 */
	SigBit getCellOutput(Cell *cell) const;

	// ===== 遍历辅助 =====

	/**
	 * BFS遍历（从给定信号集合开始）
	 * @param start_signals 起始信号集合
	 * @param visitor 访问函数 void(SigBit)
	 */
	void bfsTraverse(
		const pool<SigBit> &start_signals,
		std::function<void(SigBit)> visitor
	);

	/**
	 * DFS遍历
	 * @param start_signal 起始信号
	 * @param visitor 访问函数 bool(SigBit)，返回false时停止遍历
	 */
	void dfsTraverse(
		SigBit start_signal,
		std::function<bool(SigBit)> visitor
	);

	/**
	 * 重新构建图数据结构
	 * @note 在模块结构改变后（如删除门）需要调用此方法
	 */
	void rebuild();

	// ===== 调试接口 =====

	void printGraphStatistics() const;

private:
	Module *module;
	SigMap &sigmap;

	// 缓存的拓扑序（只计算一次）
	vector<SigBit> topo_order;
	vector<SigBit> reverse_topo_order;
	bool topo_computed;

	// 图数据
	dict<SigBit, Cell*> bit2driver;
	dict<SigBit, vector<Cell*>> bit2reader;

	// ⭐⭐⭐【强制性修复】⭐⭐⭐
	// ❌ 删除局部的、信息不完整的CellTypes对象
	// CellTypes ct;
	// ✅ 改为使用全局的 yosys_celltypes，它已在 synth_pango.cc 中通过 SetPangoCellTypes 完整初始化
	// ⭐⭐⭐【修复结束】⭐⭐⭐

	// 内部方法
	void buildGraphData();
	void computeTopologicalOrder();
};

YOSYS_NAMESPACE_END

#endif // GRAPH_UTILS_H
