#######################
# SDCC Makefile for making a hexfile from all .C files in this directory,
# and specified directories.
#
# Output files are created in directory './SDCC'.
# Target file for STM8 programming is ./SDCC/main.ihx
#######################

# required for Standard Peripheral Library
DEVICE           = STM8S105

# required for stm8flash
stm8flash_PATH   = ~/Öffentlich/GitHub/External/stm8flash/stm8flash
stm8flash_DEVICE = stm8s105c6
stm8flash_SWIM   = stlink
#stm8flash_SWIM   = stlinkv2

# required for stm8gal
stm8gal_PATH     = ~/Öffentlich/GitHub/stm8gal/binaries/stm8gal_linux64
stm8gal_PORT     = /dev/ttyUSB0

# define compiler path (if not in PATH), and flags
CC               = sdcc
LD               = sdcc
OPTIMIZE         = 
CFLAGS           = -mstm8 --std-sdcc99 --std-c99 $(OPTIMIZE) -D$(DEVICE)
LFLAGS           = -mstm8 -lstm8 --out-fmt-ihx

# set output folder and target name
OUTPUT_DIR       = SDCC
TARGET           = $(OUTPUT_DIR)/main.ihx

# find all -c and .h in specified directories PRJ_DIRS
PRJ_SRC_DIR      = .
PRJ_INC_DIR      = $(PRJ_SRC_DIR)
PRJ_SOURCE       = $(foreach d, $(PRJ_SRC_DIR), $(wildcard $(d)/*.c))
PRJ_HEADER       = $(foreach d, $(PRJ_INC_DIR), $(wildcard $(d)/*.h))
PRJ_OBJECTS      = $(addprefix $(OUTPUT_DIR)/, $(notdir $(PRJ_SOURCE:.c=.rel)))

# Standard Peripheral Library files to use
SPL_FILES        = stm8s_clk.c stm8s_gpio.c
SPL_ROOT         = /home/georg/Öffentlich/GitHub/STM8_SPL/STM8S_StdPeriph_Lib/STM8S_StdPeriph_Lib_V2.3.1_sdcc/Libraries/STM8S_StdPeriph_Driver
SPL_SRC_DIR      = $(SPL_ROOT)/src
SPL_INC_DIR      = $(SPL_ROOT)/inc
SPL_SOURCE       = $(addprefix $(SPL_SRC_DIR)/, $(SPL_FILES))
SPL_HEADER       = $(addprefix $(SPL_INC_DIR)/, $(SPL_FILES:.c=.h)) stm8s.h
SPL_OBJECTS      = $(addprefix $(OUTPUT_DIR)/, $(SPL_FILES:.c=.rel))

# concat all project files
SRC_DIR          = $(PRJ_SRC_DIR) $(SPL_SRC_DIR) 
INC_DIR          = $(PRJ_INC_DIR) $(SPL_INC_DIR) 
SOURCE           = $(PRJ_SOURCE) $(SPL_SOURCE) 
HEADER           = $(PRJ_HEADER) $(SPL_HEADER) 
OBJECTS          = $(PRJ_OBJECTS) $(SPL_OBJECTS) 

# set compiler include paths
INCLUDE          = $(foreach d, $(INC_DIR), $(addprefix -I, $(d)))

# set make search paths
vpath %.c $(SRC_DIR)
vpath %.h $(INC_DIR)

# debug: print variable and stop
#$(error variable is [${INC_DIR}])


########
# dependencies & make instructions
########

.PHONY: clean all default

.PRECIOUS: $(TARGET) $(OBJECTS)

default: $(OUTPUT_DIR) $(TARGET)

all: default

# create output folder
$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)
	rm -fr -- -p

# link target
$(TARGET) : $(OBJECTS)
	$(LD) $(LFLAGS) -o $@ $(OBJECTS)

# compile objects
$(OBJECTS) : $(SOURCE) $(HEADER)
$(OUTPUT_DIR)/%.rel : %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

# clean up
clean:
	rm -fr $(OUTPUT_DIR)/*
	rm -fr IAR/Debug
	rm -fr IAR/Release
	rm -fr Cosmic/Debug
	rm -fr Cosmic/Release


# upload SDCC output via SWIM ( https://github.com/vdudouyt/stm8flash )
swim:
	$(stm8flash_PATH) -c $(stm8flash_SWIM) -w $(TARGET) -p $(stm8flash_DEVICE)

# upload IAR output via SWIM ( https://github.com/vdudouyt/stm8flash )
swim_IAR:
	$(PATH_stm8flash) -c $(stm8flash_SWIM) -w ./IAR/Debug/Exe/test.s19 -p $(stm8flash_DEVICE)

# upload Cosmic output via SWIM ( https://github.com/vdudouyt/stm8flash )
swim_Cosmic:
	$(PATH_stm8flash) -c $(stm8flash_SWIM) -w ./IAR/Debug/Exe/test.s19 -p $(stm8flash_DEVICE)


# upload SDCC output via STM8 bootloader ( https://github.com/gicking/stm8gal )
serial:
	$(stm8gal_PATH) -port $(stm8gal_PORT) -write-file $(TARGET) -reset 0 -verify 0 -verbose 1

# upload IAR output via STM8 bootloader ( https://github.com/gicking/stm8gal )
serial_IAR:
	$(stm8gal_PATH) -port $(stm8gal_PORT) -write-file ./IAR/Debug/Exe/test.s19 -reset 0 -verify 0 -verbose 1

# upload Cosmic output via STM8 bootloader ( https://github.com/gicking/stm8gal )
serial_Cosmic:
	$(stm8gal_PATH) -port $(stm8gal_PORT) -write-file ./Cosmic/Debug/test.s19 -reset 0 -verify 0 -verbose 1

#EOF
