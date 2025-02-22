PROJECT_NAME := ble_app_hids_keyboard_s110_pca10028

NORDIC_SDK:=../../nRF51_SDK_10.0.0_dc26b5e
COMPONENTS_DIR:=$(NORDIC_SDK)/components



export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = $(COMPONENTS_DIR)/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

GNU_INSTALL_ROOT:=../../gcc-arm-none-eabi-4_9-2015q1


MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'
GDB              := '$(GNU_PREFIX)-gdb' #'$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gdb'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath $(COMPONENTS_DIR)/libraries/button/app_button.c) \
$(abspath $(COMPONENTS_DIR)/libraries/util/app_error.c) \
$(abspath $(COMPONENTS_DIR)/libraries/fifo/app_fifo.c) \
$(abspath $(COMPONENTS_DIR)/libraries/scheduler/app_scheduler.c) \
$(abspath $(COMPONENTS_DIR)/libraries/timer/app_timer.c) \
$(abspath $(COMPONENTS_DIR)/libraries/timer/app_timer_appsh.c) \
$(abspath $(COMPONENTS_DIR)/libraries/trace/app_trace.c) \
$(abspath $(COMPONENTS_DIR)/libraries/util/nrf_assert.c) \
$(abspath $(COMPONENTS_DIR)/libraries/uart/retarget.c) \
$(abspath $(COMPONENTS_DIR)/libraries/sensorsim/sensorsim.c) \
$(abspath $(COMPONENTS_DIR)/libraries/uart/app_uart_fifo.c) \
$(abspath $(COMPONENTS_DIR)/drivers_nrf/delay/nrf_delay.c) \
$(abspath $(COMPONENTS_DIR)/drivers_nrf/common/nrf_drv_common.c) \
$(abspath $(COMPONENTS_DIR)/drivers_nrf/hal/nrf_adc.c) \
$(abspath $(COMPONENTS_DIR)/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath $(COMPONENTS_DIR)/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath $(COMPONENTS_DIR)/drivers_nrf/pstorage/pstorage.c) \
$(abspath $(NORDIC_SDK)/examples/bsp/bsp.c) \
$(abspath $(NORDIC_SDK)/examples/bsp/bsp_btn_ble.c) \
$(abspath ./main.c) \
$(abspath $(COMPONENTS_DIR)/ble/common/ble_advdata.c) \
$(abspath $(COMPONENTS_DIR)/ble/ble_advertising/ble_advertising.c) \
$(abspath $(COMPONENTS_DIR)/ble/ble_services/ble_bas/ble_bas.c) \
$(abspath $(COMPONENTS_DIR)/ble/common/ble_conn_params.c) \
$(abspath $(COMPONENTS_DIR)/ble/ble_services/ble_dis/ble_dis.c) \
$(abspath $(COMPONENTS_DIR)/ble/ble_services/ble_hids/ble_hids.c) \
$(abspath $(COMPONENTS_DIR)/ble/common/ble_srv_common.c) \
$(abspath $(COMPONENTS_DIR)/ble/device_manager/device_manager_peripheral.c) \
$(abspath $(COMPONENTS_DIR)/toolchain/system_nrf51.c) \
$(abspath $(COMPONENTS_DIR)/softdevice/common/softdevice_handler/softdevice_handler.c) \
$(abspath $(COMPONENTS_DIR)/softdevice/common/softdevice_handler/softdevice_handler_appsh.c) \

#$(abspath $(COMPONENTS_DIR)/drivers_nrf/hal//nrf_adc.c) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(COMPONENTS_DIR)/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS  = -I$(abspath ./config/ble_app_hids_keyboard_s110_pca10028)
INC_PATHS += -I$(abspath ./config)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/scheduler)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/drivers_nrf/config)
INC_PATHS += -I$(abspath $(NORDIC_SDK)/examples/bsp)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/fifo)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/ble/ble_services/ble_hids)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/drivers_nrf/delay)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/util)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/ble/device_manager)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/drivers_nrf/uart)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/ble/common)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/sensorsim)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/drivers_nrf/pstorage)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/ble/ble_services/ble_dis)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/device)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/uart)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/button)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/timer)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/softdevice/s110/headers)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/drivers_nrf/hal)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/toolchain/gcc)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/toolchain)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/drivers_nrf/common)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/ble/ble_advertising)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/libraries/trace)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/ble/ble_services/ble_bas)
INC_PATHS += -I$(abspath $(COMPONENTS_DIR)/softdevice/common/softdevice_handler)

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DBOARD_PCA10000
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DS110
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
#CFLAGS += -DBSP_UART_SUPPORT
#CFLAGS += -DENABLE_DEBUG_LOG_SUPPORT
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror 
CFLAGS += -O3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums
CFLAGS += -g

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys
#LDFLAGS += -g

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DS110
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
//ASMFLAGS += -DBSP_UART_SUPPORT
#default target - first one defined
default: clean nrf51422_xxab_s110

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxab_s110

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxab_s110
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxab_s110: OUTPUT_FILENAME := nrf51422_xxab_s110
nrf51422_xxab_s110: LINKER_SCRIPT=config/ble_app_hids_keyboard_gcc_nrf51.ld
nrf51422_xxab_s110: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	@echo $(CC)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash:
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/nrf51422_xxab_s110.hex
	$(GDB) -batch \
	-ex "tar ext /dev/ttyACM0" \
	-ex "mon s" \
	-ex "att 1" \
	-ex run \
	-ex quit

## Flash softdevice
flash_softdevice:
	@echo Flashing: s110_nrf51_8.0.0_softdevice.hex
	$(GDB) -batch \
	-ex "tar ext /dev/ttyACM0" \
	-ex "mon s" \
	-ex "att 1" \
	-ex "load $(COMPONENTS_DIR)/softdevice/s110/hex/s110_nrf51_8.0.0_softdevice.hex" \
	-ex run \
	-ex quit

flash_all:
	@echo Flashing: s110_nrf51_8.0.0_softdevice.hex
	$(GDB) -batch \
	-ex "tar ext /dev/ttyACM0" \
	-ex "mon s" \
	-ex "att 1" \
	-ex "mon erase_mass" \
	-ex "load $(OUTPUT_BINARY_DIRECTORY)/nrf51422_xxab_s110.hex" \
	-ex "load $(COMPONENTS_DIR)/softdevice/s110/hex/s110_nrf51_8.0.0_softdevice.hex" \
	-ex run \
	-ex quit



debug:
	@echo Flashing: s110_nrf51_8.0.0_softdevice.hex
	$(GDB) -batch \
	-ex "tar ext /dev/ttyACM0" \
	-ex "mon s" \
	-ex "att 1" \
	-ex "file $(OUTPUT_BINARY_DIRECTORY)/s110_nrf51_8.0.0_softdevice.out" \
	-ex run \
	-ex quit

