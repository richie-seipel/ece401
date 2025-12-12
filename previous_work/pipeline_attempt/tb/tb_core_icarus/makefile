###############################################################################
# Variables: Program ELF
###############################################################################
ELF_FILE ?= test.elf

OBJCOPY ?= riscv32-unknown-elf-objcopy
#ifeq ($(shell which $(OBJCOPY)),)
#  ${error $(OBJCOPY) missing from PATH}
#endif
ifeq ($(shell which iverilog),)
  ${error iverilog missing from PATH - Icarus Verilog required}
endif

###############################################################################
# Variables: Defaults
###############################################################################
TRACE          ?= 1

SRC_V_DIR      ?= ../../src_v .
SRC_DIR        ?= .

EXE            ?= output.out

###############################################################################
# Variables: Verilog
###############################################################################
SRC_V       ?= $(foreach src,$(SRC_V_DIR),$(wildcard $(src)/*.v))

VFLAGS      += $(patsubst %,-I%,$(SRC_V_DIR))
VFLAGS      += -DTRACE=$(TRACE)
VFLAGS      += -Dverilog_sim

###############################################################################
# Build output directory
###############################################################################
BUILD_DIR      ?= build/

###############################################################################
# Rules
###############################################################################
all: run

$(BUILD_DIR):
	@mkdir -p $@

$(BUILD_DIR)/tcm.bin: $(ELF_FILE) | $(BUILD_DIR)
	$(OBJCOPY) $< -O binary $@

tcm.hex: $(BUILD_DIR)/tcm.bin
	@echo "# Converting BIN to HEX for Verilog memory"
	hexdump -v -e '1/4 "%08x\n"' $< > tcm.hex

$(BUILD_DIR)/$(EXE): $(SRC_V) | $(BUILD_DIR)
	@echo "# Compiling verilog"
	iverilog $(VFLAGS) -o $@ $(SRC_V)

run: $(BUILD_DIR)/$(EXE) tcm.hex
	vvp $(BUILD_DIR)/$(EXE) -vcd

view:
	gtkwave wave.vcd gtksettings.sav

clean:
	rm -rf $(BUILD_DIR) *.vcd tcm.hex

