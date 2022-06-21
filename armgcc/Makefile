KEYBOARD_TYPES   := central peripheral integrated peripheral-s132 integrated-s132 central-s132
SCAN_TYPES       := steno matrix
SD_VERSIONS		 := 6 7

OUTPUT_DIRECTORY_BASE := build

include ./Makefile.user

GNU_PREFIX ?= arm-none-eabi
UF2_OUTPUT_DIR ?= uf2

VERBOSE      ?= 0
PROJ_SRC_DIR := ../src
SDK_ROOT 	 ?= ../../../..
DEBUG 		 ?= 

include ./FileList.mk

LINKER_SCRIPT_BASE := ybk_usbd_ble_split_keyboard_s140
LINKER_SCRIPT_S132 := ybk_ble_split_keyboard_s132.ld
ALL_TYPES :=
SRC_FILES := 

ifeq ($(VERBOSE),1)
NO_ECHO :=
else
NO_ECHO := @
endif

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# GCC flags common to C and ASM
GCC_COMMON_FLAGS += \
	-DAPP_TIMER_V2 \
	-DAPP_TIMER_V2_RTC1_ENABLED \
	-DCONFIG_GPIO_AS_PINRESET \
	-DFLOAT_ABI_HARD \
	-DSOFTDEVICE_PRESENT \
	-D__HEAP_SIZE=8192 \
	-D__STACK_SIZE=8192 \
	-mcpu=cortex-m4 \
	-mthumb -mabi=aapcs \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \


# C flags common to all targets
C_COMMON_FLAGS += $(OPT) $(GCC_COMMON_FLAGS) -std=c99
# keep every function in a separate section, this allows linker to discard unused ones
C_COMMON_FLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
C_COMMON_FLAGS += -fno-builtin -fshort-enums
C_COMMON_FLAGS += -Wall -Werror
ifeq ($(USE_RECENT_ARMGCC),1)
C_COMMON_FLAGS += -Wno-maybe-uninitialized -Wno-array-bounds
endif
C_COMMON_FLAGS += $(if $(DEBUG),-DDEBUG)

NRF52832_FLAGS += \
	-DNRF52 \
	-DNRF52832_XXAA \
	-DNRF52_PAN_74 \
	-DS132 \
	-DNRFX_USBD_ENABLED=0 \
	-DUSBD_ENABLED=0 \
	-DAPP_USBD_ENABLED=0 \
	-DNO_NEOPIXEL \
	-DDEBUG_52832 \


# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += $(GCC_COMMON_FLAGS)

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs


CC      := $(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-gcc
LD      := $(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-ld
OBJDUMP := $(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-objdump
OBJCOPY := $(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-objcopy
SIZE    := $(GNU_INSTALL_ROOT)/$(GNU_PREFIX)-size
UF2CONV ?= python ../lib/uf2/utils/uf2conv.py 

CCACHE  := $(if $(wildcard /usr/bin/ccache),ccache)
CC      := $(CCACHE) $(CC)


define exec_command
$(NO_ECHO)$(1) $(2)
endef

# $1: file
# $2: files
# out: count of items in files after file
define progress
$(if $(filter-out $(1),$(firstword $(2))),$(call progress,$(1),$(wordlist 2,$(words $(2)),$(2))),$(words $(2)))
endef

# $1: count
# $2: char
# out: char*count
define bar
$(2)$(if $(filter 0,$(shell expr "1 <= $(shell expr "$(1) - 1")")),,$(call bar,$(shell expr $(1) - 1),$(2)))
endef

define progress_bar_body
$(call bar,$(1),#)
endef

define progress_bar_empty
$(call bar,$(1), )
endef

# $1: target file
# out: remain files count
define remain_files
$(call progress,$(1),$(TARGET_FILES))
endef

# $1:  a finished filename
# out: progressbar
#
# too slow.
define print_progress
@echo -ne "$(call remain_files,$(1))/$(words $(TARGET_FILES)) [$(call progress_bar_body,$(shell expr $(words $(TARGET_FILES)) - $(call remain_files,$(1))))$(call progress_bar_empty,$(call remain_files,$(1)))]\r"
endef

# $1:  a compiled filename
# out: (count of remain files)/(count of all the target files)
define print_progress_light
@echo -ne "$(call remain_files,$(1))/$(words $(TARGET_FILES))\r"
endef

%.hex: %.out
	@echo "Making hex"
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_DIRECTORY)/$< $(OUTPUT_DIRECTORY)/$@

%.uf2: %.hex
	$(NO_ECHO)$(UF2CONV) $(OUTPUT_DIRECTORY)/$< -c -b $(if $(filter 7,SOFTDEVICE_VERSION),0x26000,0x27000) -o $@ -f 0xADA52840

# $1: index
# $2: path to source file
define check_proj_path_match_iter
$(if $(subst 0,,$(1)),$(if $(filter-out $(word $(1),$(subst /, ,$(PROJ_SRC_DIR))),$(word $(1),$(2))),,$(call check_proj_path_match_iter,$(shell expr $(1) - 1),$(2))),1)
endef

# $1: path to source file
# out: path is under $(PROJ_SRC_DIR) ? 1 : (empty)
#
# $(PROJ_SRC_DIR)/config/sdk_config.h is using CFLAGS_PROJ and is used on files under $(SDK_ROOT), 
# so this improve compile speed plan failed.
define path_is_proj
$(call check_proj_path_match_iter,$(words $(subst /, ,$(PROJ_SRC_DIR))),$(subst /, ,$(1)))
endef

.PHONY: is_proj
is_proj:
	@echo "$(PROJ_SRC_DIR)/hoge: $(call path_is_proj,$(PROJ_SRC_DIR)/hoge)"
	@echo "$(SDK_ROOT)/hoge: $(call path_is_proj,$(SDK_ROOT)/hoge)"
	@echo "$(PROJ_SRC_DIR)/main.c.o: $(call path_is_proj,$(PROJ_SRC_DIR)/main.c.o)"

%.c.o:
	@echo "Compiling $(basename $@)"
	$(call print_progress_light,$@)
	$(NO_ECHO)$(CC) $(CFLAGS) $(CFLAGS_PROJ) $(addprefix -I,$(INC_FOLDERS)) -MP -MD -c -o $(OUTPUT_DIRECTORY)/$(notdir $@) $(basename $@)
	
%.S.o:
	@echo "Assembling $(basename $@)"
	$(call print_progress_light,$@)
	$(call exec_command,$(CC) -x assembler-with-cpp -MP -MD -c -o $(OUTPUT_DIRECTORY)/$(notdir $@),$(basename $@),Assembling)

# $(1): path
#
# mkdir -p path
define make_dir_available
$(if $(wildcard $(1)),@echo DIRECTORY $(1) exist,mkdir -p $(1))
endef

.PHONY: progress_bar
progress_bar:
	@echo "progress"
	@echo -ne "$(shell expr 0)"
	@echo -ne "[$(call progress_bar_body,3)$(call progress_bar_empty,10)]\n"

.PHONY: check_outdir
check_outdir:
	$(call make_dir_available,$(OUTPUT_DIRECTORY))
	$(call make_dir_available,$(UF2_OUTPUT_DIR))

# $1: KeyboardType_MatrixType_SoftDeviceVersion
# out: target-specific source files
define get_target_src
$(if $(filter central%,$(word 1,$(subst _, ,$(1)))),$(CENTRAL_FILES)) \
$(if $(filter peripheral%,$(word 1,$(subst _, ,$(1)))),$(PERIPH_FILES),$(BLE_HID_FILES)) \
$(if $(filter integrated%,$(word 1,$(subst _, ,$(1)))), ,$(SPLIT_FILES)) \
\
$(if $(filter steno,$(word 2,$(subst _, ,$(1)))),$(STENO_FILES),$(MATRIX_FILES)) \
$(if $(filter %-s132,$(subst _, ,$(1))),$(NRF52832_FILES),$(NRF52840_FILES) $(USB_FILES) $(NEOPIXEL_FILES)) \
\
$(if $(DEBUG),$(if $(filter integrated%,$(word 1,$(subst _, ,$(1)))),$(SPLIT_FILES),)) \

endef

# $1: KeyboardType
# $2: ScanMethod
# $3: SoftDeviceVersion
# out: additional cflags
define get_target_flags
$(if $(filter central%,$(1)), -DKEYBOARD_CENTRAL) \
$(if $(filter peripheral%,$(1)), -DKEYBOARD_PERIPH, -DBLE_RAW_HID) \
\
$(if $(filter steno,$(2)), -DSCAN_MATRIX_STENO) \
$(if $(filter %-s132,$(1)),$(NRF52832_FLAGS), -DNRF52840_XXAA -DS140) \

endef

# $1: KeyboardType 
# $2: ScanMethod
# $3: SoftDeviceVersion
# out: Makefile text for $eval
define define_co_conf
ALL_TYPES += $(1)\:$(2)\:$(3)
$(1)_$(2)_$(3).out: $(addsuffix .o,$(call get_target_src,$(1)_$(2)_$(3)) $(COMMON_SRC))
	@echo "Linking $(1)_$(2)_$(3).out"
	$(call exec_command,$(CC) $(LDFLAGS) $(if $(filter %-s132,$(1)),-T$(LINKER_SCRIPT_S132),-T$(LINKER_SCRIPT_BASE)_v$(3).ld) $(addprefix $(OUTPUT_DIRECTORY_BASE)/$(1)_$(2)_$(3)/,$(notdir $(addsuffix .o,$(call get_target_src,$(1)_$(2)_$(3)) $(COMMON_SRC)))) -W -o ,$(OUTPUT_DIRECTORY_BASE)/$(1)_$(2)_$(3)/$(1)_$(2)_$(3).out,Linking)
	$(NO_ECHO)$(SIZE) $(OUTPUT_DIRECTORY_BASE)/$(1)_$(2)_$(3)/$(1)_$(2)_$(3).out

.PHONY: $(1)\:$(2)\:$(3) check\:$(1)\:$(2)\:$(3)
check\:$(1)\:$(2)\:$(3):
	@echo "SRC: $(call get_target_src,$(1)_$(2)_$(3)) $(COMMON_SRC)"
	@echo "FLAGS: $(call get_target_flags,$(1),$(2),$(3))"
	@echo "TARGET_SPECIFIC_INC_FOLDERS: $(if $(filter %-s132,$(1)),$(INC_FOLDERS_S132),$(INC_FOLDERS_S140))"

$(1)\:$(2)\:$(3): CFLAGS := $(C_COMMON_FLAGS) -DNRF_SD_BLE_API_VERSION=$(3)
$(1)\:$(2)\:$(3): CFLAGS_PROJ := $(call get_target_flags,$(1),$(2),$(3))
$(1)\:$(2)\:$(3): ASMFLAGS += $(call get_target_flags,$(1),$(2),$(3))
$(1)\:$(2)\:$(3): TARGET_FILES := $(addsuffix .o,$(call get_target_src,$(1)_$(2)_$(3)) $(COMMON_SRC))
$(1)\:$(2)\:$(3): OUTPUT_DIRECTORY := $(OUTPUT_DIRECTORY_BASE)/$(1)_$(2)_$(3)
$(1)\:$(2)\:$(3): SOFTDEVICE_VERSION := $(3)
$(1)\:$(2)\:$(3): INC_FOLDERS += $(if $(filter %-s132,$(1)),$(INC_FOLDERS_S132),$(INC_FOLDERS_S140))

$(1)\:$(2)\:$(3): check_outdir $(1)_$(2)_$(3).uf2
	mv $(1)_$(2)_$(3).uf2 $(UF2_OUTPUT_DIR)/
	@echo done $(1):$(2):$(3)
endef

define define_sd_version
.PHONY: define_$(1)_$(2)
define_$(1)_$(2):
	$(foreach sd_version,$(SD_VERSIONS),$(eval $(call define_co_conf,$(1),$(2),$(sd_version),$(3))))
endef

define define_scan
.PHONY: define_$(1)
define_$(1):
	$(foreach scan_type,$(SCAN_TYPES),$(eval $(call define_sd_version,$(1),$(scan_type))))
endef

$(foreach keyboard,$(KEYBOARD_TYPES),$(eval $(call define_scan,$(keyboard))))

# make all not works
.PHONY: all
all: $(ALL_TYPES)

.PHONY: clean
clean:
	rm -r $(OUTPUT_DIRECTORY_BASE)
	rm -r $(UF2_OUTPUT_DIR)

