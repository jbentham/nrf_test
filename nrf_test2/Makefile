# Build Nordic NRF52832 example

TARGET           := nrf_test2.elf
HEXFILE          := $(TARGET:.elf=.hex)
BUILD_DIR        := build
SDK              := ../nrf5_sdk
PROJ             := $(CURDIR)
LINKER_SCRIPT    := nrf52_test.ld
OPTIMISE         := -O3

GCC              := arm-none-eabi-gcc
OBJCOPY          := arm-none-eabi-objcopy
OBJSIZE          := arm-none-eabi-size

HEAP_SIZE        := 8192
STACK_SIZE       := 8192

SRC_FILES        += $(SDK)/modules/nrfx/mdk/gcc_startup_nrf52.S
SRC_FILES        += $(SDK)/modules/nrfx/mdk/system_nrf52.c
SRC_FILES        += $(SDK)/modules/nrfx/drivers/src/nrfx_systick.c
SRC_FILES        += $(SDK)/modules/nrfx/drivers/src/nrfx_uart.c
SRC_FILES        += $(SDK)/modules/nrfx/drivers/src/prs/nrfx_prs.c
SRC_FILES        += $(SDK)/components/libraries/util/app_util_platform.c
SRC_FILES        += $(SDK)/external/fprintf/nrf_fprintf.c
SRC_FILES        += $(SDK)/external/fprintf/nrf_fprintf_format.c
SRC_FILES        += $(PROJ)/nrf_test2.c

SRC_DIRS         := $(dir $(SRC_FILES))
SRCES            := $(notdir $(SRC_FILES))
OBJECTS          := $(SRCES:=.o)
OBJ_FILES        := $(addprefix $(BUILD_DIR)/,$(OBJECTS))

INC_FOLDERS      += $(PROJ) 
INC_FOLDERS      += $(SDK)/modules/nrfx                  $(SDK)/modules/nrfx/mdk
INC_FOLDERS      += $(SDK)/modules/nrfx/hal              $(SDK)/modules/nrfx/drivers/include
INC_FOLDERS      += $(SDK)/components/libraries/strerror $(SDK)/components/toolchain/cmsis/include
INC_FOLDERS      += $(SDK)/components/libraries/util     $(SDK)/components/libraries/balloc
INC_FOLDERS      += $(SDK)/components/libraries/ringbuf  $(SDK)/components/libraries/bsp 
INC_FOLDERS      += $(SDK)/components/libraries/log      $(SDK)/components/libraries/experimental_section_vars
INC_FOLDERS      += $(SDK)/components/libraries/delay    $(SDK)/components/drivers_nrf/nrf_soc_nosd 
INC_FOLDERS      += $(SDK)/components/libraries/atomic   $(SDK)/components/boards
INC_FOLDERS      += $(SDK)/components/libraries/memobj   $(SDK)/components/libraries/log/src
INC_FOLDERS      += $(SDK)/external/fprintf              $(SDK)/integration/nrfx
INC_FOLDERS      += $(SDK)/integration/nrfx/legacy

CFLAGS           += -std=c99 -MP -MD -c -g3 $(OPTIMISE)
CFLAGS           += -DBOARD_CUSTOM -DBSP_DEFINES_ONLY -DCONFIG_GPIO_AS_PINRESET -DFLOAT_ABI_HARD -DNRF52 -DNRF52832_XXAA -DNRF52_PAN_74
CFLAGS           += -mcpu=cortex-m4 -mthumb -mabi=aapcs -Wall -Werror -mfloat-abi=hard -mfpu=fpv4-sp-d16 -ffunction-sections -fdata-sections
CFLAGS           += -fno-strict-aliasing -fno-builtin -fshort-enums -D__HEAP_SIZE=$(HEAP_SIZE) -D__STACK_SIZE=$(STACK_SIZE)

LNFLAGS          += -O3 -g3 -mthumb -mabi=aapcs -L$(SDK)/modules/nrfx/mdk -T$(LINKER_SCRIPT) -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
LNFLAGS          += -Wl,--gc-sections --specs=nano.specs
LNFLAGS2         += -Wl,-Map=build/blinky.map -lc -lnosys -lm

VPATH            := $(SRC_DIRS)

$(BUILD_DIR)/$(TARGET): $(OBJ_FILES)
	@echo Linking $@
	@$(GCC) $(LNFLAGS) -o $@ $^ $(LNFLAGS2)
	@$(OBJCOPY) -O ihex $(BUILD_DIR)/$(TARGET) $(BUILD_DIR)/$(HEXFILE)
	@$(OBJCOPY) -O binary $(BUILD_DIR)/$(TARGET) $(BUILD_DIR)/$(TARGET:.elf=.bin)
	@$(OBJSIZE) $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/%.c.o: %.c
	@echo Compiling $<
	@$(GCC) $(CFLAGS) -o $@ $< $(addprefix -I,$(INC_FOLDERS)) 

$(BUILD_DIR)/%.S.o: %.S
	@echo Assembling $<
	@$(GCC) $(CFLAGS) -o $@ $<

jflash:	$(BUILD_DIR)/$(TARGET)
	@echo Flashing $(HEXFILE)
	nrfjprog --program $(BUILD_DIR)/$(HEXFILE) --sectorerase
	nrfjprog --reset

ocdflash: $(BUILD_DIR)/$(TARGET)
	sudo openocd -f ../openocd/rpi2.cfg -f ../openocd/nrf52_swd.cfg -c "program $(BUILD_DIR)/$(TARGET) verify reset exit"

# EOF

