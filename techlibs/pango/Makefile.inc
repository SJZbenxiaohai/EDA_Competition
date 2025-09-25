
OBJS += techlibs/pango/synth_pango.o
OBJS += techlibs/pango/score.o
OBJS += techlibs/pango/lut_merge_optimizer.o
OBJS += techlibs/pango/lut_merge_interface.o
OBJS += techlibs/pango/lut_merge_utils.o
OBJS += techlibs/pango/synth_pango_extend.o
OBJS += techlibs/pango/lut_merge_analyzer.o

$(eval $(call add_share_file,share/pango,techlibs/pango/pango_lib.v))
$(eval $(call add_share_file,share/pango,techlibs/pango/pango_sim.v))


