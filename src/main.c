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
#include "mef.h"
#include "hc06_driver.h"         /* <= sAPI header */
#include "ledsManager.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]      ========================*/

/*==================[internal functions declaration] ========================*/

/*==================[internal data definition]       ========================*/

/*==================[external data definition]       ========================*/

/*==================[internal functions definition]  ========================*/

/*==================[external functions definition]  ========================*/

#define u32 long
#define CantMuestras (1024)
#define LOWFREQDIV 8
//////////////////// valores para calcular seno y coseno en punto fijo/////////////////////////
#define PRECISION 10
#define qN 11
#define qA PRECISION
#define qP 15
#define qR (2*qN-qP)
#define qS (qN+qP+1-qA)
#define MAXTOTALSAMPLES 2048
#define FSAMPLE 25000
#define ESCALA (1<<PRECISION) 			//  valores mayores a 10 da overflow al multiplicar#define int2PI (1<<13)					// un circulo completo tiene 8192 unidades de largo, el seno usa esta propiedad#define ALPHA ((7<<PRECISION)/13)		// 0.53836*(1<<PRECISION)#define BETA ((6<<PRECISION)/13)		// 1-0.53836*(1<<PRECISION)		para la hamming#define FminBorde 20#define FmaxBorde 12500#define Fs 2*FmaxBorde 				//nyquist ok#define refreshRate 60					//test#define SATURACION 465#define FREQS 8#define MAXCYCLE (FSAMPLE/FREQS/refreshRate)#define nrOfBands 8int ultimaMuestra = 0;						// ultima muestraint senial[CantMuestras];int16_t senial_bajas[CantMuestras];	// señal de las bajas frecuenciasunsigned int freqs[FREQS];						// frecuencias de cada banda/filtrou32 indice_bajas = 0; // indice que indica que filtros de frecuencia usan una frtecuencia de muestreo mas bajau32 amplitud;unsigned char calculationFlag;unsigned int oldMinF, oldMaxF, F0, F16;unsigned int nrInterrupts, nrInterruptsSinceEffectChange,nrInterruptsSinceSignal;	// contadores de interrupcion
unsigned int refreshCounter;
unsigned int Freq[FREQS + 1];// frecuencias superior e inferior por cada filtro/banda

unsigned int Div[FREQS];// divisor de frecuencias de muestras para cada filtro

unsigned int NFreq[FREQS];	// numero de muestras necesarios para cada filtro

unsigned int dosPiQ;	// 2*pi*Q value for the CQT (Constant Q Transform)

int band_it;
unsigned int DivIndex;
unsigned int leds[FREQS];

int kb_abs(int num) { //calcula el valor absoluto rapidito ANDA BIEN
	int mascara = num >> 31;
	return (mascara + num) ^ mascara;
}

int applyDecay(int new, int old) {
	if (new >= old) {
		return (2 * new + old) / 3;
	} else {
		return (old + new * (1024)) / 1024;
	}
}

void ADC_IRQ(void) { //TODO: ADC_IRQ
	nrInterrupts++;
	ultimaMuestra = (uint16_t) adcRead(ADC0) - 511;
	senial[nrInterrupts % CantMuestras] = ultimaMuestra;
	senial_bajas[(nrInterrupts / LOWFREQDIV) % CantMuestras] = ultimaMuestra;
	actualizarEntradas();

	ledsControl(leds);

	if (!(refreshCounter % MAXCYCLE))
		calculationFlag = 1;
}

void setup() { //TODO: setup
	int k;
	oldMinF = 20;
	oldMaxF = 12500;
	amplitud = 256;
	nrInterrupts = 0;
	for (k = 0; k < FREQS; ++k) {
		Div[k] = 1;	//
	}
	for (k = 0; k < FREQS; k++) {
		leds[k] = 0;
	}
	HC06_init(9600);

	// Configuraciï¿½n de Timers
	uint32_t TICKS_ADC = Timer_microsecondsToTicks(40);	//25 KHz
	//uint32_t TICKS_MEF = Timer_microsecondsToTicks(1000000);
	Timer_Init(TIMER0, TICKS_ADC, ADC_IRQ);
	uartConfig(UART_USB, 115200);
	adcConfig(BASS_SPECTRUM); /* Configuraciï¿½n personalizada para BASS */
	dacConfig(DAC_DISABLE); /* DAC DESACTIVADO */
	boardGpiosInit();
	preprocesar_filtros();

}

void MEF_IRQ(void) {
	uint8_t tecla = HC06_ReadByte();
	UpdateMEF(tecla);
}

void actualizarEntradas() {
	char k;
	unsigned int valor;
	unsigned int minF = 20;
	unsigned int maxF = 12500;	// TODO: ACTUALIZAR ENTRADAS
	float base = 1.0091;
	F0 = powf(base, minF);
	F16 = powf(base, maxF);

	for (k = 0; k < FREQS; k++) {
		valor = freqs[k] << 10;
		leds[k] = applyDecay(valor, leds[k]);
		if (leds[k] < 1)
			leds[k] = 1;
		if (leds[k] > 16)
			leds[k] = 16;
	}
}

void preprocesar_filtros() { //TODO: preprocesar filtros
// Calcula el tamaï¿½o de los filtros, como asï¿½ tambien otras constantes necesarias para la cqt. Tiene que ser llamada cada vez que se cambia la Fs, la Fmax o Fmin.

	float nn = powf(2, log(F16 / (double) F0) / log(2) / 16.0); //cambio de base
	dosPiQ = int2PI * (nn + 1) / (nn - 1) / 2;

	int i;
	for (i = 0; i < FREQS + 1; ++i) {
		Freq[i] = (F0 * powf(nn, i) + F16 / powf(nn, FREQS - i)) / 2;
	}

	indice_bajas = 0;
	while (Fs / (Freq[indice_bajas + 1] - Freq[indice_bajas]) >= CantMuestras
			&& indice_bajas < FREQS) {
		++indice_bajas;
	}

	int samplesLeft = MAXTOTALSAMPLES;

	for (i = FREQS; i > indice_bajas; --i) {
		Div[i - 1] = 1 + Fs / ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
		NFreq[i - 1] = Fs / (Freq[i] - Freq[i - 1]) / Div[i - 1];
		samplesLeft -= NFreq[i - 1];
	}
	for (; i > 0; --i) {
		Div[i - 1] = 1
				+ Fs / LOWFREQDIV / ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
		NFreq[i - 1] = Fs / LOWFREQDIV / (Freq[i] - Freq[i - 1]) / Div[i - 1];
		samplesLeft -= NFreq[i - 1];
	}
}

//Funcion trigonometrica que tira valores entre -1024 y 1024, siendo PI = 8192. Es recontra rï¿½pido.
//Usa taylor de grado 4, re afanado de un blog
int SinAprox(int x) {
// S(x) = x * ( (3<<p) - (x*x>>r) ) >> s
// n : Q-pos for quarter circle             11, so full circle is 2^13 long
// A : Q-pos for output                     10
// p : Q-pos for parentheses intermediate   15
// r = 2n-p                                  7
// s = A-1-p-n                              17

	x = x << (30 - qN);		// resize to pi range
							// shift to full s32 range (Q13->Q30)

	if ((x ^ (x << 1)) < 0)	// test for quadrant 1 or 2
		x = (1 << 31) - x;

	x = x >> (30 - qN);
	return (x * ((3 << qP) - (x * x >> qR)) >> qS);
}

int CosAprox(int in) {
	return SinAprox((int2PI >> 2) - in);
}

int hamming(int m, int k) {
	return ALPHA - (BETA * CosAprox(int2PI * m / NFreq[k]) >> PRECISION);
}

void cqt() {
	unsigned int k, i, indice;
	int ventana, fase;
	float real_f, imag_f;
	int real, imag;
	for (k = 0; k < indice_bajas; ++k) {
		indice = nrInterrupts % CantMuestras - 1 + 8 * CantMuestras;
		real = ALPHA
				- (BETA * senial_bajas[indice % CantMuestras] >> PRECISION);
		imag = 0;
		for (i = 1; i < NFreq[k]; ++i) {
			ventana = hamming(i, k)
					* senial_bajas[(indice - i * Div[k]) % CantMuestras];
			fase = dosPiQ * i / NFreq[k];
			real += ventana * CosAprox(fase) >> PRECISION;
			imag += ventana * SinAprox(fase) >> PRECISION;
		}

		real_f = real / (float) ESCALA;
		imag_f = imag / (float) ESCALA;
		freqs[k] = logf(
				powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1)
				* amplitud / 32;
	}
	for (; k < FREQS; ++k) {
		indice = nrInterrupts % CantMuestras - 1 + 8 * CantMuestras;
		real = ALPHA - (BETA * senial[indice % CantMuestras] >> PRECISION);
		imag = 0;
		for (i = 1; i < NFreq[k]; ++i) {
			ventana = hamming(i, k)
					* senial[(indice - i * Div[k]) % CantMuestras];
			fase = dosPiQ * i / NFreq[k];
			real += ventana * CosAprox(fase) >> PRECISION;
			imag += ventana * SinAprox(fase) >> PRECISION;
		}
		real_f = real / (float) ESCALA;
		imag_f = imag / (float) ESCALA;

		freqs[k] = logf(
				powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1)
				* amplitud / 32;
	}
}

/*
 * Funcion robada de por ahï¿½, transforma un valor int en cualquier base a ascii
 */
char* itoa(int value, char* result, int base) {
// check that the base if valid
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ =
				"zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35
						+ (tmp_value - value * base)];
	} while (value);

// Apply negative sign
	if (tmp_value < 0)
		*ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void) {

	/* Inicializar la placa */
	boardConfig();
//Timer_Init(TIMER1,TICKS_MEF,MEF_IRQ);
	setup(); //arranca la ciaa para muestrear y demas

	/* ------------- REPETIR POR SIEMPRE ------------- */
	while (1) {
		if (kb_abs(ultimaMuestra) >= SATURACION) { //se fija que no sature, si satura prende los leds.
			gpioWrite(LED1, ON);
			gpioWrite(LED2, ON);
			gpioWrite(LED3, ON);
		} else {
			gpioWrite(LED1, OFF);
			gpioWrite(LED2, OFF);
			gpioWrite(LED3, OFF);
		}
		actualizarEntradas();

		if (calculationFlag == 1) {
			calculationFlag = 0;
			refreshCounter = MAXCYCLE;
			cqt();
		}

	}

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
	 por ningun S.O. */
	return 0;
}

/*==================[end of file]============================================*/
