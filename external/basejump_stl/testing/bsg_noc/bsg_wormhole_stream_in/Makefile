export BASEJUMP_STL_DIR ?= ../../../

INCDIR = +incdir+$(BASEJUMP_STL_DIR)/bsg_misc
INCDIR += +incdir+$(BASEJUMP_STL_DIR)/bsg_noc

.PHONY: sim dve all

all: sim

bsg_trace_rom.v: trace.tr
	echo "hello"

	#python $(BASEJUMP_STL_DIR)/bsg_mem/bsg_ascii_to_rom.py trace.tr bsg_trace_rom \
	#	> bsg_trace_rom.v

sim: 
	vcs -R +lint=all,noSVA-UA,noSVA-NSVU,noVCDE -sverilog -full64 -f sv.include $(INCDIR) \
		-debug_pp -assert svaext -timescale=1ps/1ps +vcs+vcdpluson

dve:
	dve -full64 -vpd vcdplus.vpd &

clean:
	rm -rf DVEfiles
	rm -rf csrc
	rm -rf simv.daidir simv.vdb
	rm -f ucli.key vcdplus.vpd simv cm.log *.tar.gz
