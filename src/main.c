/* Copyright 2016, Eric Pernia.

 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Date: 2016-04-26
 */

/*==================[inclusions]=============================================*/

#include "main.h"         /* <= own header */
#include "math.h"
#include "sAPI.h"         /* <= sAPI header */
#include "hc06_driver.h"         /* <= sAPI header */
#include "ledsManager.h"
#include "stdlib.h"

#define dosFIX_MPY(DEST,A,B)       {       \
        _DX = (B);                      \
        _AX = (A);                      \
        asm imul dx;                    \
        asm add ax,ax;                  \
        asm adc dx,dx;                  \
        DEST = _DX;             }

#define FIX_MPY(DEST,A,B)       DEST = ((long)(A) * (long)(B))>>15

#define N_onda_senos          1024    /* dimension del vector de los senos */
#define LOG2_N_WAVE     10      /* log2(N_WAVE) */
#define N_volumen          100     /* dimension del vector de volumenes */
#ifndef fixed
#define fixed short
#endif

extern fixed Seno[N_onda_senos];
extern fixed Loudampl[N_volumen];
int db_from_ampl(fixed re, fixed im);
uint8_t asdf[8] = { 4, 4, 4, 4, 4, 4, 4, 4 };
fixed fix_mpy(fixed a, fixed b);

/*
 fix_fft() -.

 si n>0 la FFT está hecha, sino si n<0 la inversa de la FFT esta hecha
 fr[n],fi[n] es la parte real y la imaginaria respectivamente, ENTRADA Y SALIDA
 tamaño  de los datos = 2^m
 setear 0=dft, 1=idft
 */
int fix_fft(int fr[], int fi[], int m, int inverse) {
	int mr, nn, i, j, l, k, istep, n, scale, shift;
	fixed qr, qi, tr, ti, wr, wi;

	n = 1 << m;

	if (n > N_onda_senos)
		return -1;

	mr = 0;
	nn = n - 1;
	scale = 0;

	/* diezmado en tiempo, hay que reordenar los datos */
	for (m = 1; m <= nn; ++m) {
		l = n;
		do {
			l >>= 1;
		} while (mr + l > nn);
		mr = (mr & (l - 1)) + l;

		if (mr <= m)
			continue;
		tr = fr[m];
		fr[m] = fr[mr];
		fr[mr] = tr;
		ti = fi[m];
		fi[m] = fi[mr];
		fi[mr] = ti;
	}

	l = 1;
	k = LOG2_N_WAVE - 1;
	while (l < n) {
		if (inverse) {
			/* escalamiento variable segun como vengan los datos */
			shift = 0;
			for (i = 0; i < n; ++i) {
				j = fr[i];
				if (j < 0)
					j = -j;
				m = fi[i];
				if (m < 0)
					m = -m;
				if (j > 16383 || m > 16383) {
					shift = 1;
					break;
				}
			}
			if (shift)
				++scale;
		} else {
			/* escalamiento fijo para que normalice bien, va a haber log2(n) pasadas
			 *
			 * */
			shift = 1;
		}
		/*puede que no sea tan obvio pero el shift se va a hacer una sola vez por cada dato
		 *   */
		istep = l << 1;
		for (m = 0; m < l; ++m) {
			j = m << k;
			/* 0 <= j < N_WAVE/2 */
			wr = Seno[j + N_onda_senos / 4];
			wi = -Seno[j];
			if (inverse)
				wi = -wi;
			if (shift) {
				wr >>= 1;
				wi >>= 1;
			}
			for (i = m; i < n; i += istep) {
				j = i + l;
				tr = fix_mpy(wr, fr[j]) - fix_mpy(wi, fi[j]);
				ti = fix_mpy(wr, fi[j]) + fix_mpy(wi, fr[j]);
				qr = fr[i];
				qi = fi[i];
				if (shift) {
					qr >>= 1;
					qi >>= 1;
				}
				fr[j] = qr - tr;
				fi[j] = qi - ti;
				fr[i] = qr + tr;
				fi[i] = qi + ti;
			}
		}
		--k;
		l = istep;
	}

	return scale;
}

/*      window() - ventana de HANNING, con hamming no funcionaba tan bien       */
void window(fixed fr[], int n) {
	int i, j, k;

	j = N_onda_senos / n;
	n >>= 1;
	for (i = 0, k = N_onda_senos / 4; i < n; ++i, k += j)
		FIX_MPY(fr[i], fr[i], 16384 - (Seno[k] >> 1));
	n <<= 1;
	for (k -= j; i < n; ++i, k -= j)
		FIX_MPY(fr[i], fr[i], 16384 - (Seno[k] >> 1));
}

/*      fix_loud() -
 *
 * computa el volumen en el dominio de la frecuencia.
 * n debería ser ntot/2, donde ntot se le pasa a fix_fft();
 * se le suman 6dB para tomar en cuenta los componentes del aliasing
 * scale_shift debería ser el resultado de fix_fft() si los datos temporales fueron
 * obtenidos de la IFFT, o 0 sino.
 * loud[] es el volumen, en dB, 32767 es +10 y al reves para los negativos
 *
 *  */
void fix_loud(fixed loud[], fixed fr[], fixed fi[], int n, int scale_shift) {
	int i, max;

	max = 0;
	if (scale_shift > 0)
		max = 10;
	scale_shift = (scale_shift + 1) * 6;

	for (i = 0; i < n; ++i) {
		loud[i] = db_from_ampl(fr[i], fi[i]) + scale_shift;
		if (loud[i] > max)
			loud[i] = max;
	}
}

/*      db_from_ampl() -
 * tira el volumen en dB a partir de la amplitud en complejo

 */
int db_from_ampl(fixed re, fixed im) {
	static long loud2[N_volumen] = { 0 };
	long v;
	int i;

	if (loud2[0] == 0) {
		loud2[0] = (long) Loudampl[0] * (long) Loudampl[0];
		for (i = 1; i < N_volumen; ++i) {
			v = (long) Loudampl[i] * (long) Loudampl[i];
			loud2[i] = v;
			loud2[i - 1] = (loud2[i - 1] + v) / 2;
		}
	}

	v = (long) re * (long) re + (long) im * (long) im;

	for (i = 0; i < N_volumen; ++i)
		if (loud2[i] <= v)
			break;

	return (-i);
}

/*
 fix_mpy() - multiplicación en punto fijo
 */
fixed fix_mpy(fixed a, fixed b) {
	FIX_MPY(a, a, b);
	return a;
}

/*
 iscale() - escala un numero por (numer/denom)
 */
int iscale(int value, int numer, int denom) {
#ifdef  DOS
	asm mov ax,value
	asm imul WORD PTR numer
	asm idiv WORD PTR denom

	return _AX;
#else
	return (long) value * (long) numer / (long) denom;
#endif
}

/*
 fix_dot() - producto punto de dos vectores en punto fijo
 */
fixed fix_dot(fixed *hpa, fixed *pb, int n) {
	fixed *pa;
	long sum;
	register fixed a, b;

	/*      seg = FP_SEG(hpa);
	 off = FP_OFF(hpa);
	 seg += off>>4;
	 off &= 0x000F;				no
	 pa = MK_FP(seg,off);
	 */
	sum = 0L;
	while (n--) {
		a = *pa++;
		b = *pb++;
		FIX_MPY(a, a, b);
		sum += a;
	}

	if (sum > 0x7FFF)
		sum = 0x7FFF;
	else if (sum < -0x7FFF)
		sum = -0x7FFF;

	return (fixed) sum;
#ifdef  DOS
	/* ASSUMES hpa is already normalized so FP_OFF(hpa) < 16 */
	asm push ds
	asm lds si,hpa
	asm les di,pb
	asm xor bx,bx

	asm xor cx,cx

	loop: /* intermediate values can overflow by a factor of 2 without
	 causing an error; the final value must not overflow! */
	asm lodsw
	.
	asm imul word ptr es:[di]
	asm add bx,ax
	asm adc cx,dx
	asm jo overflow
	asm add di,2
	asm dec word ptr n
	asm jg loop

	asm add bx,bx
	asm adc cx,cx
	asm jo overflow

	asm pop ds
	return _CX;

	overflow:
	asm mov cx,7FFFH
	asm adc cx,0

	asm pop ds
	return _CX;
#endif

}


fixed Seno[1024] = { 0, 201, 402, 603, 804, 1005, 1206, 1406, 1607, 1808,
		2009, 2209, 2410, 2610, 2811, 3011, 3211, 3411, 3611, 3811, 4011, 4210,
		4409, 4608, 4807, 5006, 5205, 5403, 5601, 5799, 5997, 6195, 6392, 6589,
		6786, 6982, 7179, 7375, 7571, 7766, 7961, 8156, 8351, 8545, 8739, 8932,
		9126, 9319, 9511, 9703, 9895, 10087, 10278, 10469, 10659, 10849, 11038,
		11227, 11416, 11604, 11792, 11980, 12166, 12353, 12539, 12724, 12909,
		13094, 13278, 13462, 13645, 13827, 14009, 14191, 14372, 14552, 14732,
		14911, 15090, 15268, 15446, 15623, 15799, 15975, 16150, 16325, 16499,
		16672, 16845, 17017, 17189, 17360, 17530, 17699, 17868, 18036, 18204,
		18371, 18537, 18702, 18867, 19031, 19194, 19357, 19519, 19680, 19840,
		20000, 20159, 20317, 20474, 20631, 20787, 20942, 21096, 21249, 21402,
		21554, 21705, 21855, 22004, 22153, 22301, 22448, 22594, 22739, 22883,
		23027, 23169, 23311, 23452, 23592, 23731, 23869, 24006, 24143, 24278,
		24413, 24546, 24679, 24811, 24942, 25072, 25201, 25329, 25456, 25582,
		25707, 25831, 25954, 26077, 26198, 26318, 26437, 26556, 26673, 26789,
		26905, 27019, 27132, 27244, 27355, 27466, 27575, 27683, 27790, 27896,
		28001, 28105, 28208, 28309, 28410, 28510, 28608, 28706, 28802, 28897,
		28992, 29085, 29177, 29268, 29358, 29446, 29534, 29621, 29706, 29790,
		29873, 29955, 30036, 30116, 30195, 30272, 30349, 30424, 30498, 30571,
		30643, 30713, 30783, 30851, 30918, 30984, 31049, 31113, 31175, 31236,
		31297, 31356, 31413, 31470, 31525, 31580, 31633, 31684, 31735, 31785,
		31833, 31880, 31926, 31970, 32014, 32056, 32097, 32137, 32176, 32213,
		32249, 32284, 32318, 32350, 32382, 32412, 32441, 32468, 32495, 32520,
		32544, 32567, 32588, 32609, 32628, 32646, 32662, 32678, 32692, 32705,
		32717, 32727, 32736, 32744, 32751, 32757, 32761, 32764, 32766, 32767,
		32766, 32764, 32761, 32757, 32751, 32744, 32736, 32727, 32717, 32705,
		32692, 32678, 32662, 32646, 32628, 32609, 32588, 32567, 32544, 32520,
		32495, 32468, 32441, 32412, 32382, 32350, 32318, 32284, 32249, 32213,
		32176, 32137, 32097, 32056, 32014, 31970, 31926, 31880, 31833, 31785,
		31735, 31684, 31633, 31580, 31525, 31470, 31413, 31356, 31297, 31236,
		31175, 31113, 31049, 30984, 30918, 30851, 30783, 30713, 30643, 30571,
		30498, 30424, 30349, 30272, 30195, 30116, 30036, 29955, 29873, 29790,
		29706, 29621, 29534, 29446, 29358, 29268, 29177, 29085, 28992, 28897,
		28802, 28706, 28608, 28510, 28410, 28309, 28208, 28105, 28001, 27896,
		27790, 27683, 27575, 27466, 27355, 27244, 27132, 27019, 26905, 26789,
		26673, 26556, 26437, 26318, 26198, 26077, 25954, 25831, 25707, 25582,
		25456, 25329, 25201, 25072, 24942, 24811, 24679, 24546, 24413, 24278,
		24143, 24006, 23869, 23731, 23592, 23452, 23311, 23169, 23027, 22883,
		22739, 22594, 22448, 22301, 22153, 22004, 21855, 21705, 21554, 21402,
		21249, 21096, 20942, 20787, 20631, 20474, 20317, 20159, 20000, 19840,
		19680, 19519, 19357, 19194, 19031, 18867, 18702, 18537, 18371, 18204,
		18036, 17868, 17699, 17530, 17360, 17189, 17017, 16845, 16672, 16499,
		16325, 16150, 15975, 15799, 15623, 15446, 15268, 15090, 14911, 14732,
		14552, 14372, 14191, 14009, 13827, 13645, 13462, 13278, 13094, 12909,
		12724, 12539, 12353, 12166, 11980, 11792, 11604, 11416, 11227, 11038,
		10849, 10659, 10469, 10278, 10087, 9895, 9703, 9511, 9319, 9126, 8932,
		8739, 8545, 8351, 8156, 7961, 7766, 7571, 7375, 7179, 6982, 6786, 6589,
		6392, 6195, 5997, 5799, 5601, 5403, 5205, 5006, 4807, 4608, 4409, 4210,
		4011, 3811, 3611, 3411, 3211, 3011, 2811, 2610, 2410, 2209, 2009, 1808,
		1607, 1406, 1206, 1005, 804, 603, 402, 201, 0, -201, -402, -603, -804,
		-1005, -1206, -1406, -1607, -1808, -2009, -2209, -2410, -2610, -2811,
		-3011, -3211, -3411, -3611, -3811, -4011, -4210, -4409, -4608, -4807,
		-5006, -5205, -5403, -5601, -5799, -5997, -6195, -6392, -6589, -6786,
		-6982, -7179, -7375, -7571, -7766, -7961, -8156, -8351, -8545, -8739,
		-8932, -9126, -9319, -9511, -9703, -9895, -10087, -10278, -10469,
		-10659, -10849, -11038, -11227, -11416, -11604, -11792, -11980, -12166,
		-12353, -12539, -12724, -12909, -13094, -13278, -13462, -13645, -13827,
		-14009, -14191, -14372, -14552, -14732, -14911, -15090, -15268, -15446,
		-15623, -15799, -15975, -16150, -16325, -16499, -16672, -16845, -17017,
		-17189, -17360, -17530, -17699, -17868, -18036, -18204, -18371, -18537,
		-18702, -18867, -19031, -19194, -19357, -19519, -19680, -19840, -20000,
		-20159, -20317, -20474, -20631, -20787, -20942, -21096, -21249, -21402,
		-21554, -21705, -21855, -22004, -22153, -22301, -22448, -22594, -22739,
		-22883, -23027, -23169, -23311, -23452, -23592, -23731, -23869, -24006,
		-24143, -24278, -24413, -24546, -24679, -24811, -24942, -25072, -25201,
		-25329, -25456, -25582, -25707, -25831, -25954, -26077, -26198, -26318,
		-26437, -26556, -26673, -26789, -26905, -27019, -27132, -27244, -27355,
		-27466, -27575, -27683, -27790, -27896, -28001, -28105, -28208, -28309,
		-28410, -28510, -28608, -28706, -28802, -28897, -28992, -29085, -29177,
		-29268, -29358, -29446, -29534, -29621, -29706, -29790, -29873, -29955,
		-30036, -30116, -30195, -30272, -30349, -30424, -30498, -30571, -30643,
		-30713, -30783, -30851, -30918, -30984, -31049, -31113, -31175, -31236,
		-31297, -31356, -31413, -31470, -31525, -31580, -31633, -31684, -31735,
		-31785, -31833, -31880, -31926, -31970, -32014, -32056, -32097, -32137,
		-32176, -32213, -32249, -32284, -32318, -32350, -32382, -32412, -32441,
		-32468, -32495, -32520, -32544, -32567, -32588, -32609, -32628, -32646,
		-32662, -32678, -32692, -32705, -32717, -32727, -32736, -32744, -32751,
		-32757, -32761, -32764, -32766, -32767, -32766, -32764, -32761, -32757,
		-32751, -32744, -32736, -32727, -32717, -32705, -32692, -32678, -32662,
		-32646, -32628, -32609, -32588, -32567, -32544, -32520, -32495, -32468,
		-32441, -32412, -32382, -32350, -32318, -32284, -32249, -32213, -32176,
		-32137, -32097, -32056, -32014, -31970, -31926, -31880, -31833, -31785,
		-31735, -31684, -31633, -31580, -31525, -31470, -31413, -31356, -31297,
		-31236, -31175, -31113, -31049, -30984, -30918, -30851, -30783, -30713,
		-30643, -30571, -30498, -30424, -30349, -30272, -30195, -30116, -30036,
		-29955, -29873, -29790, -29706, -29621, -29534, -29446, -29358, -29268,
		-29177, -29085, -28992, -28897, -28802, -28706, -28608, -28510, -28410,
		-28309, -28208, -28105, -28001, -27896, -27790, -27683, -27575, -27466,
		-27355, -27244, -27132, -27019, -26905, -26789, -26673, -26556, -26437,
		-26318, -26198, -26077, -25954, -25831, -25707, -25582, -25456, -25329,
		-25201, -25072, -24942, -24811, -24679, -24546, -24413, -24278, -24143,
		-24006, -23869, -23731, -23592, -23452, -23311, -23169, -23027, -22883,
		-22739, -22594, -22448, -22301, -22153, -22004, -21855, -21705, -21554,
		-21402, -21249, -21096, -20942, -20787, -20631, -20474, -20317, -20159,
		-20000, -19840, -19680, -19519, -19357, -19194, -19031, -18867, -18702,
		-18537, -18371, -18204, -18036, -17868, -17699, -17530, -17360, -17189,
		-17017, -16845, -16672, -16499, -16325, -16150, -15975, -15799, -15623,
		-15446, -15268, -15090, -14911, -14732, -14552, -14372, -14191, -14009,
		-13827, -13645, -13462, -13278, -13094, -12909, -12724, -12539, -12353,
		-12166, -11980, -11792, -11604, -11416, -11227, -11038, -10849, -10659,
		-10469, -10278, -10087, -9895, -9703, -9511, -9319, -9126, -8932, -8739,
		-8545, -8351, -8156, -7961, -7766, -7571, -7375, -7179, -6982, -6786,
		-6589, -6392, -6195, -5997, -5799, -5601, -5403, -5205, -5006, -4807,
		-4608, -4409, -4210, -4011, -3811, -3611, -3411, -3211, -3011, -2811,
		-2610, -2410, -2209, -2009, -1808, -1607, -1406, -1206, -1005, -804,
		-603, -402, -201, };


fixed Loudampl[100] = { 32767, 29203, 26027, 23197, 20674, 18426, 16422, 14636,
		13044, 11626, 10361, 9234, 8230, 7335, 6537, 5826, 5193, 4628, 4125,
		3676, 3276, 2920, 2602, 2319, 2067, 1842, 1642, 1463, 1304, 1162, 1036,
		923, 823, 733, 653, 582, 519, 462, 412, 367, 327, 292, 260, 231, 206,
		184, 164, 146, 130, 116, 103, 92, 82, 73, 65, 58, 51, 46, 41, 36, 32,
		29, 26, 23, 20, 18, 16, 14, 13, 11, 10, 9, 8, 7, 6, 5, 5, 4, 4, 3, 3, 2,
		2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, };

#define SATURACION 460
#define CantMuestras 1024
#define MAXCYCLE 700 //TODO: MAXCYCLE
#define M       10
#define N       (1024)
#define CANTSENOS N
#define BANDAS 16
#define fixed short

int senial[1024];
int ultimaMuestra;
unsigned int refreshCounter;
unsigned int nrInterrupts;
unsigned char calculationFlag;
int real[N], imag[N];
unsigned int energia[N];
unsigned char bandas[BANDAS / 2];
char mostrar;

int kb_abs(int num) { //calcula el valor absoluto rapidito ANDA BIEN
	int mascara = num >> 31;
	return (mascara + num) ^ mascara;
}

void ADC_IRQ(void) { //TODO: ADC_IRQ

	nrInterrupts++;
	ultimaMuestra = (uint16_t) adcRead(ADC0) - 511;
	senial[nrInterrupts % CantMuestras] = ultimaMuestra;

	if ((nrInterrupts % MAXCYCLE) == 0) {
		calculationFlag = 1;
	}
	ultimaMuestra = kb_abs(ultimaMuestra);

	mostrar=(ultimaMuestra > 10)?1:0;
//		gpioWrite(LEDB, ON);
//		if (ultimaMuestra > 128) {
//			gpioWrite(LED1, ON);
//			if (ultimaMuestra > 256) {
//				gpioWrite(LED2, ON);
//				if (ultimaMuestra > 512) {
//					gpioWrite(LED3, ON);
//				} else
//					gpioWrite(LED3, OFF);
//			} else
//				gpioWrite(LED2, OFF);
//		} else
//			gpioWrite(LED1, OFF);
//	}
//
//	else {
//		gpioWrite(LEDB, OFF);
//	}

}
void LED_REF(void)
{
	ledsControl(bandas);
}

void setup() {

	int k;
	shiftBoardInit();
	gpioConfig(LEDR, OUTPUT);
	gpioConfig(LEDG, OUTPUT);
	gpioConfig(LEDB, OUTPUT);
	gpioConfig(LED1, OUTPUT);
	gpioConfig(LED2, OUTPUT);
	gpioConfig(LED3, OUTPUT);
	gpioWrite(LED3, OFF);
	gpioWrite(LED2, OFF);
	gpioWrite(LED1, OFF);
	refreshCounter = 0;
	nrInterrupts = 0;
	for (k = 0; k < BANDAS; k++)
		bandas[k] = 0;
	HC06_init(9600);
	for (k = 0; k < CantMuestras; k++) {
		senial[k] = 0;
	}
	calculationFlag = 1;

	// Configuraciï¿½n de Timers

	//uint32_t TICKS_MEF = Timer_microsecondsToTicks(1000000);

}

void calcularBandas() {
	int i,j,med;

//	for (i = 0; i < 1024; i++) {
//		real[i] = senial[i];
//	}
	memcpy(real, senial,1024 * sizeof(int));
	for(i=0;i<8;i++)
		bandas[i]=0;
	fix_fft(real, imag, 10, 0);

	for ( j=0; j<1; j++)
	{
		med=0;
		for (i = 5; i < 64; i++) {
			med+=(lround(hypot(real[(j<<6)+i],imag[(j<<6)+i]))<<1);

			//med+=(lround(sqrt(pow(real[(j<<6)+i], 2) + pow(imag[(j<<6)+i], 2)))<<1);
		}
		bandas[j]=med>>7;
	}
	for ( j=1; j<8; j++)
	{
		med=0;
		for (i = 0; i < 64; i++) {
			med+=(lround(hypot(real[(j<<6)+i],imag[(j<<6)+i]))<<1);
		//	med+=(lround(sqrt(pow(real[(j<<6)+i], 2) + pow(imag[(j<<6)+i], 2)))<<1);
		}
		bandas[j]=med>>6;
	}

	for ( j=6; j<8; j++)
		{
			med=0;
			for (i = 0; i < 64; i++) {
				med+=(lround(hypot(real[(j<<6)+i],imag[(j<<6)+i]))<<1);
			//	med+=(lround(sqrt(pow(real[(j<<6)+i], 2) + pow(imag[(j<<6)+i], 2)))<<1);
			}
			bandas[j]=med>>5;
		}
}

int main(void) {
	//long i = 0;

	boardConfig();

	setup(); //arranca la ciaa para muestrear y demas

	uint32_t TICKS_ADC = Timer_microsecondsToTicks(40);	//25 KHz
	//uint32_t TICKS_LED = Timer_microsecondsToTicks(200);	//25 KHz
	Timer_Init(TIMER0, TICKS_ADC, ADC_IRQ);
	//Timer_Init(TIMER1, TICKS_LED, LED_REF);
	adcConfig(BASS_SPECTRUM);
//	dacConfig(DAC_DISABLE);

	while (1) {

//		if (kb_abs(ultimaMuestra) >= 470) { //se fija que no sature, si satura prende los leds.
//			gpioWrite(LED1, ON);
//			gpioWrite(LED2, ON);
//			gpioWrite(LED3, ON);
//		} else {
//			gpioWrite(LED1, OFF);
//			gpioWrite(LED2, OFF);
//			gpioWrite(LED3, OFF);
//		}

		if (calculationFlag == 1) {
			calcularBandas();
			calculationFlag = 0;
		}
		if(mostrar)
		ledsControl(bandas);
	}

	return 0;
}
