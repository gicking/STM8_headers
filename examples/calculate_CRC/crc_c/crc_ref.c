/*******************************************************************************
 *
 * crc_ref.c - Implementation of plain C code CRC library reference functions
 *
 * Copyright (c) 2020 Basil Hussain
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/

#include "config.h"
#include "crc_ref.h"

uint8_t crc8_1wire_update(uint8_t crc, uint8_t data) {
	uint8_t i;
	
	crc ^= data;

	for(i = 0; i < 8; i++) {
		if(crc & 1) {
			crc = (crc >> 1) ^ 0x8C;
		} else {
			crc = (crc >> 1);
		}
	}

	return crc;
}

uint8_t crc8_j1850_update(uint8_t crc, uint8_t data) {
	uint8_t i;
	
	crc ^= data;

	for(i = 0; i < 8; i++) {
		if(crc & 0x80) {
			crc = (crc << 1) ^ 0x1D;
		} else {
			crc = (crc << 1);
		}
	}

	return crc;
}

uint16_t crc16_ansi_update(uint16_t crc, uint8_t data) {
	uint8_t i;
	
	crc ^= data;

	for(i = 0; i < 8; i++) {
		if(crc & 1) {
			crc = (crc >> 1) ^ 0xA001;
		} else {
			crc = (crc >> 1);
		}
	}

	return crc;
}

uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data) {
	uint8_t i;
	
	crc ^= (uint16_t)data << 8;

	for(i = 0; i < 8; i++) {
		if(crc & 0x8000) {
			crc = (crc << 1) ^ 0x1021;
		} else {
			crc = (crc << 1);
		}
	}

	return crc;
}

uint32_t crc32_update(uint32_t crc, uint8_t data) {
	uint8_t i;
	
	crc ^= data;

	for(i = 0; i < 8; i++) {
		if(crc & 1) {
			crc = (crc >> 1) ^ 0xEDB88320UL;
		} else {
			crc = (crc >> 1);
		}
	}

	return crc;
}

uint32_t crc32_posix_update(uint32_t crc, uint8_t data) {
	uint8_t i;
	
	crc ^= (uint32_t)data << 24;

	for(i = 0; i < 8; i++) {
		if(crc & 0x80000000UL) {
			crc = (crc << 1) ^ 0x04C11DB7UL;
		} else {
			crc = (crc << 1);
		}
	}

	return crc;
}
