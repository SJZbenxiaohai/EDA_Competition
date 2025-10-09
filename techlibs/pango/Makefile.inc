
# Pango FPGA synthesis
OBJS += techlibs/pango/synth_pango.o
OBJS += techlibs/pango/score.o

# LUT merge optimization modules (active)
OBJS += techlibs/pango/lut_merge_optimizer.o
OBJS += techlibs/pango/lut_merge_analyzer.o
OBJS += techlibs/pango/lut_merge_utils.o
OBJS += techlibs/pango/lut_merge_types.o
OBJS += techlibs/pango/lut_merge_shannon.o
OBJS += techlibs/pango/lut_merge_init.o
OBJS += techlibs/pango/lut_merge_executor.o

# Dual-output mapper modules (new - feature/dual-output-mapper)
# Created: 2025-10-04
# Based on: 8号文档《模块详细设计与接口定义》
OBJS += techlibs/pango/graph_utils.o
OBJS += techlibs/pango/timing_analyzer.o
OBJS += techlibs/pango/mapping_context.o
OBJS += techlibs/pango/truth_table_computer.o
OBJS += techlibs/pango/heuristic_evaluator.o
OBJS += techlibs/pango/cut_manager.o
OBJS += techlibs/pango/global_merger.o
OBJS += techlibs/pango/dual_output_mapper.o

# ⚠️  DEPRECATED MODULES - 已停用，保留备用 ⚠️ 
# 停用日期: 2025-09-26
# 停用原因: 架构重构，功能已迁移到synth_pango.cc
# 如需重新启用，请取消下面的注释
# OBJS += techlibs/pango/synth_pango_extend.o
# OBJS += techlibs/pango/lut_merge_interface.o

$(eval $(call add_share_file,share/pango,techlibs/pango/pango_lib.v))
$(eval $(call add_share_file,share/pango,techlibs/pango/pango_sim.v))


