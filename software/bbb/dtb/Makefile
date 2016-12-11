# Configuration
BIN_DIR = bin
ALL_DTS = BB-SPI0-01-00A0.dts

# Source
ALL_DTBOS = $(patsubst %.dts,$(BIN_DIR)/%.dtbo,$(ALL_DTS))

# Top-level targets
.PHONY: all
all: $(ALL_DTBOS)
	@true

.PHONY: clean
clean:
	@rm -rf $(BIN_DIR)

# Translation rules
$(BIN_DIR)/%.dtbo: %.dts
	@mkdir -p $(@D)
	@echo "(DTC)     $<"
	@dtc -O dtb -o $@ -b 0 -@ $<
