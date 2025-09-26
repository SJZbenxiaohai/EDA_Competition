
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

# ⚠️  DEPRECATED MODULES - 已停用，保留备用 ⚠️ 
# 停用日期: 2025-09-26
# 停用原因: 架构重构，功能已迁移到synth_pango.cc
# 如需重新启用，请取消下面的注释
# OBJS += techlibs/pango/synth_pango_extend.o
# OBJS += techlibs/pango/lut_merge_interface.o

$(eval $(call add_share_file,share/pango,techlibs/pango/pango_lib.v))
$(eval $(call add_share_file,share/pango,techlibs/pango/pango_sim.v))


