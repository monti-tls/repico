# repico/software/ecu/libecu
# Copyright (C) 2016 Alexandre Monti
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Tools
CC  = arm-unknown-eabi-gcc
AS  = arm-unknown-eabi-gcc
LD  = arm-unknown-eabi-gcc -fuse-ld=bfd
AR  = arm-unknown-eabi-ar
CPP = arm-unknown-eabi-cpp
FMT = clang-format

# Directories
SRC_DIR = src
INC_DIR = inc
TMP_DIR = obj
BIN_DIR = bin

# Configuration
PRODUCT  =
DEFINES  =
CC_FLAGS =
AS_FLAGS =
LD_FLAGS =

-include conf.mk

DEP_FILES = $(addsuffix .mk,$(DEPENDS))
DEP_ALL_TARGETS = $(addsuffix -all,$(DEPENDS))
DEP_CLEAN_TARGETS = $(addsuffix -clean,$(DEPENDS))

# Mandatory CC flags
CC_FLAGS += -std=c11 -g -O0
CC_FLAGS += -Wall -Wextra -fasm -Wno-unused-function
CC_FLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CC_FLAGS += -mfloat-abi=soft
CC_FLAGS += $(DEFINES)
CC_FLAGS += -I$(INC_DIR) -I$(SRC_DIR)

# Mandatory AS flags
CC_FLAGS += $(DEFINES)
AS_FLAGS += -I$(INC_DIR) -I$(SRC_DIR)

# Mandatory LD flags
LD_FLAGS += -lm -lnosys -nostartfiles

# Format flags
FMT_FLAGS = -i -style=file

# Sources management
C_SUB  = $(shell find $(SRC_DIR) -type d)
H_SUB  = $(shell find $(INC_DIR) -type d)

C_SRC  = $(wildcard $(addsuffix /*.c,$(C_SUB)))
S_SRC  = $(wildcard $(addsuffix /*.s,$(C_SUB)))

C_OBJ  = $(patsubst $(SRC_DIR)/%.c,$(TMP_DIR)/%.o,$(C_SRC))
S_OBJ  = $(patsubst $(SRC_DIR)/%.s,$(TMP_DIR)/%.o,$(S_SRC))

C_DEP  = $(patsubst $(SRC_DIR)/%.c,$(TMP_DIR)/%.d,$(C_SRC))
S_DEP  = $(patsubst $(SRC_DIR)/%.s,$(TMP_DIR)/%.d,$(S_SRC))

C_FMT  = $(foreach d,$(C_SUB),$(patsubst $(d)/%.c,$(d)/fmt-%,$(wildcard $(d)/*.c)))
H_FMT  = $(foreach d,$(H_SUB),$(patsubst $(d)/%.h,$(d)/fmt-%,$(wildcard $(d)/*.h)))

LS_SCR = $(wildcard *.lds)
L_SCR  = $(TMP_DIR)/$(patsubst %.lds,%.ld,$(LS_SCR))

# Product files
ifeq ($(STATIC_LIBRARY),)
BINARY = $(BIN_DIR)/$(PRODUCT).elf
else
BINARY = $(BIN_DIR)/$(PRODUCT).a
endif

# Top-level
.PHONY: all
all: $(DEP_ALL_TARGETS) $(BINARY)
	@true

.PHONY: clean
clean:
	@rm -rf $(BIN_DIR) $(TMP_DIR)

.PHONY: depclean
depclean: $(DEP_CLEAN_TARGETS) clean

.PHONY: format
format: $(C_FMT) $(H_FMT)

# External dependencies definitions
include $(DEP_FILES)

# Dependencies
-include $(C_DEPS) $(S_DEPS)

# Translation
ifeq ($(STATIC_LIBRARY),)
$(BINARY): $(C_OBJ) $(S_OBJ) $(L_SCR)
	@mkdir -p $(@D)
	@echo "(LD)      $@"
	@$(LD) -o $@ $(filter-out $(L_SCR),$^) $(LD_FLAGS) -T$(L_SCR)
else
$(BINARY): $(C_OBJ) $(S_OBJ)
	@mkdir -p $(@D)
	@echo "(AR)      $@"
	@$(AR) -rcs $@ $^
endif

$(L_SCR): $(LS_SCR)
	@mkdir -p $(@D)
	@echo "(CPP)     $@"
	@$(CPP) $(DEFINES) -P -C $< -o $@

$(TMP_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(@D)
	@echo "(CC)      $<"
	@$(CC) $(CC_FLAGS) -MMD -c $< -o $@

$(TMP_DIR)/%.o: $(SRC_DIR)/%.s
	@mkdir -p $(@D)
	@echo "(AS)      $<"
	@$(AS) $(AS_FLAGS) -MMD -c $< -o $@

# Format
fmt-%: %.c
	@$(FMT) $(FMT_FLAGS) $<
	@echo "(FMT)     $<"

fmt-%: %.h
	@$(FMT) $(FMT_FLAGS) $<
	@echo "(FMT)     $<"
