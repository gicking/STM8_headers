#!/bin/bash 

# change to current working directory
cd `dirname $0`

# just for output
echo off
clear

# remove required SPL headers
rm stm8s.h 
rm stm8s_clk.h 
rm stm8s_gpio.h 

# remove required SPL sources
rm stm8s_clk.c 
rm stm8s_gpio.c 
