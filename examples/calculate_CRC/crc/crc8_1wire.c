/*******************************************************************************
 *
 * crc8_1wire.c - CRC8-1WIRE implementation
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

#include <stddef.h>
#include <stdint.h>
#include "../crc.h"

#ifdef __SDCC_MODEL_LARGE
#define ASM_ARGS_SP_OFFSET 4
#define ASM_RETURN retf
#else
#define ASM_ARGS_SP_OFFSET 3
#define ASM_RETURN ret
#endif

// CRC8-1WIRE (aka Dallas, Maxim, iButton)
// Polynomial: x^8 + x^5 + x^4 + 1 (0x8C, reversed)
// Initial value: 0x00
// XOR out: 0x00

uint8_t crc8_1wire_update(uint8_t crc, uint8_t data) __naked {
	// Avoid compiler warnings for unreferenced args.
	(void)crc;
	(void)data;

	__asm
		; Load CRC variable from stack into A register for further work.
		ld a, (ASM_ARGS_SP_OFFSET+0, sp)

		; XOR the CRC with data byte.
		xor a, (ASM_ARGS_SP_OFFSET+1, sp)

	.macro crc8_1wire_update_shift_xor skip_lbl
			; Shift CRC value right by one bit.
			srl a

			; Jump if least-significant bit of CRC is now zero.
			jrnc skip_lbl

			; XOR the CRC value with the polynomial value.
			xor a, #0x8C

		skip_lbl:
	.endm

#ifdef ASM_UNROLL_LOOP

		crc8_1wire_update_shift_xor 0001$
		crc8_1wire_update_shift_xor 0002$
		crc8_1wire_update_shift_xor 0003$
		crc8_1wire_update_shift_xor 0004$
		crc8_1wire_update_shift_xor 0005$
		crc8_1wire_update_shift_xor 0006$
		crc8_1wire_update_shift_xor 0007$
		crc8_1wire_update_shift_xor 0008$

#else

		; Initialise counter to loop 8 times, once for each bit of data byte.
		ldw x, #8

	0001$:

		crc8_1wire_update_shift_xor 0002$

		; Decrement counter and loop around if it is not zero.
		decw x
		jrne 0001$

#endif

		; The A reg now contains updated CRC value, so leave it there as
		; function return value.
		ASM_RETURN
	__endasm;
}
