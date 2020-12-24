/*******************************************************************************
 *
 * crc16_ansi.c - CRC16-ANSI implementation
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

// CRC16-ANSI (aka IBM, Modbus, USB)
// Polynomial: x^16 + x^15 + x^2 + 1 (0xA001, reversed)
// Initial value: 0xFFFF
// XOR out: 0x0000

uint16_t crc16_ansi_update(uint16_t crc, uint8_t data) __naked {
	// Avoid compiler warnings for unreferenced args.
	(void)crc;
	(void)data;

	// For return value/arg: 0xAABB
	// x = 0xAABB (xh = 0xAA, xl = 0xBB)

	__asm
		; XOR the LSB of the CRC with data byte, and put it back in the CRC.
		ld a, (ASM_ARGS_SP_OFFSET+2, sp)
		xor a, (ASM_ARGS_SP_OFFSET+1, sp)
		ld (ASM_ARGS_SP_OFFSET+1, sp), a

		; Load CRC variable from stack into X register for further work.
		ldw x, (ASM_ARGS_SP_OFFSET+0, sp)

	.macro crc16_ansi_update_shift_xor skip_lbl
			; Shift CRC value right by one bit.
			srlw x

			; Jump if least-significant bit of CRC is now zero.
			jrnc skip_lbl

			; XOR the CRC value with the polynomial value.
			rrwa x                       ; put LSB of crc into a
			xor a, #0x01                 ; xor it with 0x01
			rrwa x                       ; put MSB of crc into a
			xor a, #0xA0                 ; xor it with 0xA0
			rrwa x                       ; put counter back into a

		skip_lbl:
	.endm

#ifdef ASM_UNROLL_LOOP

		crc16_ansi_update_shift_xor 0001$
		crc16_ansi_update_shift_xor 0002$
		crc16_ansi_update_shift_xor 0003$
		crc16_ansi_update_shift_xor 0004$
		crc16_ansi_update_shift_xor 0005$
		crc16_ansi_update_shift_xor 0006$
		crc16_ansi_update_shift_xor 0007$
		crc16_ansi_update_shift_xor 0008$

#else

		; Initialise counter to loop 8 times, once for each bit of data byte.
		ld a, #8

	0001$:

		crc16_ansi_update_shift_xor 0002$

		; Decrement counter and loop around if it is not zero.
		dec a
		jrne 0001$

#endif

		; The X reg now contains updated CRC value, so leave it there as
		; function return value.
		ASM_RETURN
	__endasm;
}
