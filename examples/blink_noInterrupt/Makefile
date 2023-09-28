#######################
# SDCC Makefile for making a hexfile from all .C files in this directory,
# and specified directories.
#
# Output files are created in directory './SDCC'.
# Target file for STM8 programming is ./SDCC/main.ihx
#######################

# path to upload tools via SWIM or bootloader
STM8FLASH_PATH = ~/Öffentlich/GitHub/External/stm8flash/stm8flash
STM8GAL_PATH   = ~/Öffentlich/GitHub/stm8gal/stm8gal

# select board
#BOARD = STM8S_DISCOVERY
#BOARD = STM8L_DISCOVERY
BOARD = NUCLEO_8S207K8
#BOARD = NUCLEO_8S208RB
#BOARD = STM8_SO8_DISCO_STM8S001
#BOARD = STM8_SO8_DISCO_STM8L001
#BOARD = STM8_SO8_DISCO_STM8L050
#BOARD = SDUINO
#BOARD = MUBOARD

# set debugger & serial parameters
ifeq ($(BOARD), STM8S_DISCOVERY)
    STM8FLASH_DEVICE = stm8s105c6
	STM8FLASH_SWIM   = stlink			# built-in
	SERIAL_PORT      = /dev/ttyUSB0			# external
else ifeq ($(BOARD), STM8L_DISCOVERY)
	STM8FLASH_DEVICE = stm8l152c6
	STM8FLASH_SWIM   = stlinkv2			# built-in
	SERIAL_PORT      = /dev/ttyUSB0			# external
else ifeq ($(BOARD), NUCLEO_8S207K8)
	STM8FLASH_DEVICE = stm8s207k8
	STM8FLASH_SWIM   = stlinkv21			# built-in
	SERIAL_PORT      = /dev/ttyACM0			# built-in
else ifeq ($(BOARD), NUCLEO_8S208RB)
	STM8FLASH_DEVICE = stm8s208rb
	STM8FLASH_SWIM   = stlinkv21			# built-in
	SERIAL_PORT      = /dev/ttyACM0			# built-in
else ifeq ($(BOARD), STM8_SO8_DISCO_STM8S001)
	STM8FLASH_DEVICE = stm8s001j3
	STM8FLASH_SWIM   = stlinkv2			# built-in
	SERIAL_PORT      = /dev/ttyUSB0			# external
else ifeq ($(BOARD), STM8_SO8_DISCO_STM8L001)
	STM8FLASH_DEVICE = stm8l001j3
	STM8FLASH_SWIM   = stlinkv2			# built-in
	SERIAL_PORT      = /dev/ttyUSB0			# external
else ifeq ($(BOARD), STM8_SO8_DISCO_STM8L050)
	STM8FLASH_DEVICE = stm8l050j3
	STM8FLASH_SWIM   = stlinkv2			# built-in
	SERIAL_PORT      = /dev/ttyUSB0			# external
else ifeq ($(BOARD), SDUINO)
	STM8FLASH_DEVICE = stm8s105k6
	STM8FLASH_SWIM   = stlink			# external
	SERIAL_PORT      = /dev/ttyUSB0			# built-in
else ifeq ($(BOARD), MUBOARD)
	STM8FLASH_DEVICE = stm8s207mb
	STM8FLASH_SWIM   = stlink			# external
	SERIAL_PORT      = /dev/ttyUSB0			# built-in
endif

# define compiler path (if not in PATH), and flags
CC               = sdcc
LD               = sdcc
OPTIMIZE         = 
CFLAGS           = -mstm8 --std-sdcc99 --std-c99 $(OPTIMIZE) -D$(BOARD)
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

# concat all project files
SRC_DIR          = $(PRJ_SRC_DIR)
INC_DIR          = $(PRJ_INC_DIR)
SOURCE           = $(PRJ_SOURCE)
HEADER           = $(PRJ_HEADER)
OBJECTS          = $(PRJ_OBJECTS)

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
ifndef STM8FLASH_DEVICE
	$(error unknown board '$(BOARD)')
endif
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
	$(STM8FLASH_PATH) -c $(STM8FLASH_SWIM) -w $(TARGET) -p $(STM8FLASH_DEVICE)

# upload IAR output via SWIM ( https://github.com/vdudouyt/stm8flash )
swim_IAR:
	$(PATH_stm8flash) -c $(STM8FLASH_SWIM) -w ./IAR/Debug/Exe/test.s19 -p $(STM8FLASH_DEVICE)

# upload Cosmic output via SWIM ( https://github.com/vdudouyt/stm8flash )
swim_Cosmic:
	$(PATH_stm8flash) -c $(STM8FLASH_SWIM) -w ./IAR/Debug/Exe/test.s19 -p $(STM8FLASH_DEVICE)


# upload SDCC output via STM8 bootloader ( https://github.com/gicking/stm8gal )
serial:
	$(STM8GAL_PATH) -port $(SERIAL_PORT) -write-file $(TARGET) -reset 0 -verify 0 -verbose 1

# upload IAR output via STM8 bootloader ( https://github.com/gicking/stm8gal )
serial_IAR:
	$(STM8GAL_PATH) -port $(SERIAL_PORT) -write-file ./IAR/Debug/Exe/test.s19 -reset 0 -verify 0 -verbose 1

# upload Cosmic output via STM8 bootloader ( https://github.com/gicking/stm8gal )
serial_Cosmic:
	$(STM8GAL_PATH) -port $(SERIAL_PORT) -write-file ./Cosmic/Debug/test.s19 -reset 0 -verify 0 -verbose 1

#EOF
