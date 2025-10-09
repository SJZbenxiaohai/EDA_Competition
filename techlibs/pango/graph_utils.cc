/*
 * graph_utils.cc
 *
 * GraphUtils 实现
 *
 * 作者：根据8号文档Section 3.1实现
 * 日期：2025-10-04
 */

#include "graph_utils.h"

YOSYS_NAMESPACE_BEGIN

GraphUtils::GraphUtils(Module *module, SigMap &sigmap)
	: module(module), sigmap(sigmap), topo_computed(false)
{
	// ⭐⭐⭐【强制性修复】⭐⭐⭐
	// ❌ 删除所有对局部 ct 对象的配置。它们不再需要。
	// ✅ 改为使用全局的 yosys_celltypes，它已在 synth_pango.cc 中通过 SetPangoCellTypes 完整初始化
	// 这样 GraphUtils 就能识别所有的 GTP_* 原语和组合逻辑门
	// ⭐⭐⭐【修复结束】⭐⭐⭐

	// 构建图数据
	buildGraphData();
}

void GraphUtils::rebuild()
{
	// 清空所有数据结构
	bit2driver.clear();
	bit2reader.clear();
	topo_order.clear();
	reverse_topo_order.clear();
	topo_computed = false;

	// 重新构建
	buildGraphData();
}

void GraphUtils::buildGraphData()
{
	int skipped_cells = 0;
	int processed_cells = 0;

	// ⭐⭐⭐ 关键修复：像单输出映射器一样，为所有已知cell构建拓扑信息 ⭐⭐⭐
	// 策略改变：
	// 1. 记录所有已知cell的输入输出关系（包括DFF、IO等）
	// 2. 在后续处理中再区分哪些是组合门、哪些是边界
	// 3. 这样DFF.Q、OUTBUF.O等信号就能被识别为"主输入"边界

	// 遍历模块中的所有Cell
	for (auto cell : module->cells()) {
		IdString cell_type = cell->type;

		// 只跳过真正未知的单元类型
		if (!yosys_celltypes.cell_known(cell_type)) {
			log_debug("  GraphUtils: Skipping unknown cell type %s\n", log_id(cell_type));
			skipped_cells++;
			continue;
		}

		// ⭐⭐⭐ 修复核心：不再进行"是否组合"的判断，记录所有已知cell ⭐⭐⭐
		processed_cells++;

		// 获取输出端口 - 记录所有cell的输出（包括DFF、IO等）
		for (auto &conn : cell->connections()) {
			// ⭐ 使用全局 yosys_celltypes
			if (!yosys_celltypes.cell_output(cell_type, conn.first)) {
				continue;
			}

			// 对于输出端口的每个bit，记录驱动关系
			for (auto bit : sigmap(conn.second)) {
				if (bit.wire) {
					bit2driver[bit] = cell;  // ✅ 记录所有输出
				}
			}
		}

		// 获取输入端口 - 记录所有cell的输入（包括DFF、IO等）
		for (auto &conn : cell->connections()) {
			// ⭐ 使用全局 yosys_celltypes
			if (!yosys_celltypes.cell_input(cell_type, conn.first)) {
				continue;
			}

			// 对于输入端口的每个bit，记录读者关系
			for (auto bit : sigmap(conn.second)) {
				if (bit.wire) {
					bit2reader[bit].push_back(cell);  // ✅ 记录所有输入
				}
			}
		}
	}

	log("GraphUtils: Processed %d cells (including sequential/primitive), skipped %d unknown cells\n",
	    processed_cells, skipped_cells);
	log("GraphUtils: Built graph with %zu driven signals, %zu signals with readers\n",
	    bit2driver.size(), bit2reader.size());
}

const vector<SigBit>& GraphUtils::getTopologicalOrder()
{
	if (!topo_computed) {
		computeTopologicalOrder();
	}
	return topo_order;
}

const vector<SigBit>& GraphUtils::getReverseTopologicalOrder()
{
	if (!topo_computed) {
		computeTopologicalOrder();
	}

	// 如果反向序还未构建，构建它
	if (reverse_topo_order.empty() && !topo_order.empty()) {
		reverse_topo_order = topo_order;
		std::reverse(reverse_topo_order.begin(), reverse_topo_order.end());
	}

	return reverse_topo_order;
}

void GraphUtils::computeTopologicalOrder()
{
	dict<SigBit, int> in_degree;
	std::queue<SigBit> Q;

	// ⭐⭐⭐ 修复：只对组合门的输出进行拓扑排序 ⭐⭐⭐
	// 但利用完整的bit2driver识别边界（DFF、IO等的输出会被识别为零入度）

	// 初始化入度 - 只遍历组合门的输出
	for (auto &[bit, driver] : bit2driver) {
		// ⭐ 新增：只处理组合门
		if (!driver->type.begins_with("$_")) {
			continue;  // 跳过非组合门（DFF、IO等）
		}

		int degree = 0;
		for (auto input : getCellInputs(driver)) {
			// ⭐ 检查输入是否由组合门驱动
			if (bit2driver.count(input)) {
				Cell* input_driver = bit2driver.at(input);
				if (input_driver->type.begins_with("$_")) {
					degree++;  // 只计算组合门依赖
				}
				// 如果输入由DFF/IO驱动，不增加度数（视为主输入）
			}
		}
		in_degree[bit] = degree;

		if (degree == 0) {
			Q.push(bit);  // 零入度（主输入或边界）
		}
	}

	// Kahn算法主循环
	topo_order.clear();
	while (!Q.empty()) {
		SigBit bit = Q.front();
		Q.pop();
		topo_order.push_back(bit);

		// 更新后继的入度
		for (Cell *reader : getReaders(bit)) {
			// ⭐ 只处理组合门读者
			if (!reader->type.begins_with("$_")) {
				continue;
			}

			SigBit output = getCellOutput(reader);
			if (in_degree.count(output)) {
				in_degree[output]--;
				if (in_degree[output] == 0) {
					Q.push(output);
				}
			}
		}
	}

	// 检查是否有环（只检查组合门）
	int expected_comb_gates = 0;
	for (auto &[bit, driver] : bit2driver) {
		if (driver->type.begins_with("$_")) {
			expected_comb_gates++;
		}
	}

	if (topo_order.size() != (size_t)expected_comb_gates) {
		log_warning("Circuit has combinational loops! (expected %d gates, got %zu in topo order)\n",
		           expected_comb_gates, topo_order.size());
	}

	log("GraphUtils: Topological order contains %zu combinational gates\n", topo_order.size());
	topo_computed = true;
}

Cell* GraphUtils::getDriver(SigBit signal) const
{
	if (bit2driver.count(signal)) {
		return bit2driver.at(signal);
	}
	return nullptr;
}

const vector<Cell*>& GraphUtils::getReaders(SigBit signal) const
{
	static vector<Cell*> empty_vec;
	if (bit2reader.count(signal)) {
		return bit2reader.at(signal);
	}
	return empty_vec;
}

vector<SigBit> GraphUtils::getCellInputs(Cell *cell) const
{
	vector<SigBit> inputs;

	// 遍历所有连接，找到输入端口
	for (auto &conn : cell->connections()) {
		// ⭐ 使用全局 yosys_celltypes
		if (yosys_celltypes.cell_input(cell->type, conn.first)) {
			// 添加所有输入信号
			for (auto bit : sigmap(conn.second)) {
				if (bit.wire) {
					inputs.push_back(bit);
				}
			}
		}
	}

	return inputs;
}

SigBit GraphUtils::getCellOutput(Cell *cell) const
{
	// 假设单输出，遍历找到输出端口
	for (auto &conn : cell->connections()) {
		// ⭐ 使用全局 yosys_celltypes
		if (yosys_celltypes.cell_output(cell->type, conn.first)) {
			// 返回第一个输出信号（假设单输出）
			for (auto bit : sigmap(conn.second)) {
				if (bit.wire) {
					return bit;
				}
			}
		}
	}

	// 如果没有找到输出，返回无效的SigBit
	return SigBit();
}

void GraphUtils::bfsTraverse(
	const pool<SigBit> &start_signals,
	std::function<void(SigBit)> visitor)
{
	pool<SigBit> visited;
	std::queue<SigBit> Q;

	// 初始化队列
	for (auto signal : start_signals) {
		Q.push(signal);
		visited.insert(signal);
	}

	// BFS主循环
	while (!Q.empty()) {
		SigBit current = Q.front();
		Q.pop();

		// 访问当前节点
		visitor(current);

		// 获取所有读者（扇出）
		for (Cell *reader : getReaders(current)) {
			SigBit output = getCellOutput(reader);
			if (output.wire && !visited.count(output)) {
				Q.push(output);
				visited.insert(output);
			}
		}
	}
}

void GraphUtils::dfsTraverse(
	SigBit start_signal,
	std::function<bool(SigBit)> visitor)
{
	pool<SigBit> visited;
	std::function<bool(SigBit)> dfs_helper;

	// DFS递归辅助函数
	dfs_helper = [&](SigBit current) -> bool {
		if (visited.count(current)) {
			return true;  // 已访问，继续
		}

		visited.insert(current);

		// 访问当前节点，如果返回false则停止
		if (!visitor(current)) {
			return false;
		}

		// 递归访问所有后继
		for (Cell *reader : getReaders(current)) {
			SigBit output = getCellOutput(reader);
			if (output.wire) {
				if (!dfs_helper(output)) {
					return false;  // 停止遍历
				}
			}
		}

		return true;
	};

	// 启动DFS
	dfs_helper(start_signal);
}

void GraphUtils::printGraphStatistics() const
{
	log("GraphUtils Statistics:\n");
	log("  Signals with drivers: %zu\n", bit2driver.size());
	log("  Signals with readers: %zu\n", bit2reader.size());
	log("  Topological order size: %zu\n", topo_order.size());
}

YOSYS_NAMESPACE_END
