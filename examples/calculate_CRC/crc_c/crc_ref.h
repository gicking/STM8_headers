/*******************************************************************************
 *
 * crc_ref.h - Header file for plain C code CRC library reference functions
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

#ifndef CRC_REF_H_
#define CRC_REF_H_

#include <stdint.h>

// These have the same implementations, just with different initial values, so
// just alias them to the latter functions.
#define crc16_xmodem_update_ref crc16_ccitt_update_ref

extern uint8_t crc8_1wire_update(uint8_t crc, uint8_t data);
extern uint8_t crc8_j1850_update(uint8_t crc, uint8_t data);
extern uint16_t crc16_ansi_update(uint16_t crc, uint8_t data);
extern uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data);
extern uint32_t crc32_update(uint32_t crc, uint8_t data);
extern uint32_t crc32_posix_update(uint32_t crc, uint8_t data);

#endif // CRC_REF_H_
