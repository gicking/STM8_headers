#!/bin/bash 

# change to current working directory
cd `dirname $0`

# just for output
echo off
clear

# set path to SPL
SPL_ROOT=~/Öffentlich/GitHub/STM8_SPL/STM8S_StdPeriph_Lib/STM8S_StdPeriph_Lib_V2.3.1_sdcc/Libraries/STM8S_StdPeriph_Driver

# copy required SPL headers
cp $SPL_ROOT/inc/stm8s.h .
cp $SPL_ROOT/inc/stm8s_clk.h .
cp $SPL_ROOT/inc/stm8s_gpio.h .

# copy required SPL sources
cp $SPL_ROOT/src/stm8s_clk.c .
cp $SPL_ROOT/src/stm8s_gpio.c .
