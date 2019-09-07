######################################
# Makefile by CubeMX2Makefile.py
######################################

######################################
# target
######################################
TARGET = $TARGET

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0

#######################################
# pathes
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
$C_SOURCES  
$ASM_SOURCES

#######################################
# binaries
#######################################
CC = /media/sf__VIRTUAL_/buildtools/gcc-arm-none-eabi-4_8-131228/bin/arm-none-eabi-gcc
AS = /media/sf__VIRTUAL_/buildtools/gcc-arm-none-eabi-4_8-131228/bin/arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = /media/sf__VIRTUAL_/buildtools/gcc-arm-none-eabi-4_8-131228/bin/arm-none-eabi-size
HEX = $$(CP) -O ihex
BIN = $$(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
$AS_DEFS
$C_DEFS
# includes for gcc
$AS_INCLUDES
$C_INCLUDES
# compile gcc flags
ASFLAGS = $MCU $$(AS_DEFS) $$(AS_INCLUDES) $$(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $MCU $$(C_DEFS) $$(C_INCLUDES) $$(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($$(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
include mk/version.mk
# Generate dependency information
CFLAGS += -std=c99 -MD -MP -MF $$(BUILD_DIR)/.dep/$$(@F).d

#######################################
# LDFLAGS
#######################################
# link script
$LDSCRIPT
# libraries
LIBS =
LIBDIR =
LDFLAGS = $LDMCU $SPECS -T$$(LDSCRIPT) $$(LIBDIR) $$(LIBS) -Wl,-Map=$$(BUILD_DIR)/$$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $$(BUILD_DIR)/$$(TARGET).elf $$(BUILD_DIR)/$$(TARGET).hex $$(BUILD_DIR)/$$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $$(addprefix $$(BUILD_DIR)/,$$(notdir $$(C_SOURCES:.c=.o)))
vpath %.c $$(sort $$(dir $$(C_SOURCES)))
# list of ASM program objects
OBJECTS += $$(addprefix $$(BUILD_DIR)/,$$(notdir $$(ASM_SOURCES:.s=.o)))
vpath %.s $$(sort $$(dir $$(ASM_SOURCES)))

$$(BUILD_DIR)/%.o: %.c Makefile | $$(BUILD_DIR) 
	$$(CC) -c $$(CFLAGS) -Wa,-a,-ad,-alms=$$(BUILD_DIR)/$$(notdir $$(<:.c=.lst)) $$< -o $$@

$$(BUILD_DIR)/%.o: %.s Makefile | $$(BUILD_DIR)
	$$(AS) -c $$(CFLAGS) $$< -o $$@

$$(BUILD_DIR)/$$(TARGET).elf: $$(OBJECTS) Makefile
	$$(CC) $$(OBJECTS) $$(LDFLAGS) -o $$@
	$$(SZ) $$@

$$(BUILD_DIR)/%.hex: $$(BUILD_DIR)/%.elf | $$(BUILD_DIR)
	$$(HEX) $$< $$@
	
$$(BUILD_DIR)/%.bin: $$(BUILD_DIR)/%.elf | $$(BUILD_DIR)
	$$(BIN) $$< $$@	
	
$$(BUILD_DIR):
	mkdir -p $$@/.dep

#######################################
# clean up
#######################################
clean:
	find . -type f | xargs touch
	-rm -fR $$(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $$(shell mkdir -p $$(BUILD_DIR)/.dep 2>/dev/null) $$(wildcard $$(BUILD_DIR)/.dep/*)

.PHONY: clean all

# *** EOF ***
