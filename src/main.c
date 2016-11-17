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
#include <math.h>
#include "sAPI.h"         /* <= sAPI header */
#include <mef.h>
#include <fft_kb.h>

/*==================[macros and definitions]=================================*/

#define forsn(i,s,n) for(i=(s);i<(n);i++){}
#define forn (i,n) forsn(i,0,n)

/*==================[internal data declaration]      ========================*/

/*==================[internal functions declaration] ========================*/

/*==================[internal data definition]       ========================*/

/*==================[external data definition]       ========================*/

/*==================[internal functions definition]  ========================*/

/*==================[external functions definition]  ========================*/

#define u32 long
#define NRSAMPLES (1024)
#define LOWFREQDIV 8
//////////////////// valores para calcular seno y coseno en punto fijo/////////////////////////
#define PRECISION 10
#define qN 11
#define qA PRECISION
#define qP 15
#define qR (2*qN-qP)
#define qS (qN+qP+1-qA)
#define MAXTOTALSAMPLES 2048
////////////////////////////////////////////////////////////////////////////////////////////////

#define SCALE (1<<PRECISION) 			// NOTE: Higher than 10 might give overflow for 32-bit numbers when multiplying...#define int2PI (1<<13)					// So in our book, a circle is a full 8192 units long. The sinus functions is based on this property!#define ALPHA ((7<<PRECISION)/13)		// 0.53836*(1<<PRECISION)#define BETA ((6<<PRECISION)/13)		// 1-0.53836*(1<<PRECISION)#define MEM 3							// the number of memorized old sampling values, should always be odd#define FREQSBands 8

#define FminBorde 64
#define FmaxBorde 16384
#define Fs 2*FmaxBorde 				//nyquist ok
#define refreshRate 80					//test
#define MAXCYCLE (Fs/16/refreshRate)
#define F0 64
#define F16 16384
#define DECAYPRECISION 10					// so with a decay of 1, every 1<<DECAYPRECISION th frame the height is lowered by 1#define REDUNDANCY 11
#define FREQS 8

int latestSample = 0;						// most recent sample value
int signal[NRSAMPLES];
int test[NRSAMPLES];						// current sample signal
uint16_t signal_lowfreq[NRSAMPLES];	// current sample signal, with a lower sampling frequency (see LOWFREQDIV)
int freqs[FREQSBands];						// frequencies for each band/filter
u32 indice_bajas = 0;// index stating which frequency filters use low frequency sampling
u32 amplitud;



unsigned int nrInterrupts, nrInterruptsSinceEffectChange,
		nrInterruptsSinceSignal;	// interrupt counters

unsigned int Freq[FREQSBands + 1];// lower and upper frequency for each filter/band

unsigned int Div[FREQSBands];	// sample frequency divider for each filter

unsigned int NFreq[FREQSBands];	// number of samples needed for each filter

unsigned int dosPiQ;	// 2*pi*Q value for the CQT (Constant Q Transform)

int band_it;
unsigned int DivIndex;

int kb_abs(int num) { //calcula el valor absoluto rapidito ANDA BIEN
	int mascara = num >> 31;
	return (mascara + num) ^ mascara;
}

void preprocesar_filtros() {
	// Calcula el tama�o de los filtros, como as� tambien otras constantes necesarias para la cqt. Tiene que ser llamada cada vez que se cambia la Fs, la Fmax o Fmin.

	float nn = powf(2, log(F16 / (double) F0) / log(2) / 16.0); //cambio de base
	dosPiQ = int2PI * (nn + 1) / (nn - 1) / 2;

	int i;
	for (i = 0; i < FREQS + 1; ++i) {
		Freq[i] = (F0 * powf(nn, i) + F16 / powf(nn, FREQS - i)) / 2;
	}

	indice_bajas = 0;
	while (Fs / (Freq[indice_bajas + 1] - Freq[indice_bajas]) >= NRSAMPLES && indice_bajas < FREQS) {
		++indice_bajas;
	}

	int samplesLeft = MAXTOTALSAMPLES;

	for (i = 16; i > indice_bajas; --i) {
		Div[i - 1] = 1 + Fs / ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
		NFreq[i - 1] = Fs / (Freq[i] - Freq[i - 1]) / Div[i - 1];
		samplesLeft -= NFreq[i - 1];
	}
	for (; i > 0; --i) {
		Div[i - 1] = 1
				+ Fs / LOWFREQDIV
						/ ((Freq[i] - Freq[i - 1]) * samplesLeft / i);
		NFreq[i - 1] = Fs / LOWFREQDIV / (Freq[i] - Freq[i - 1])
				/ Div[i - 1];
		samplesLeft -= NFreq[i - 1];
	}
}

//Funcion trigonometrica que tira valores entre -1024 y 1024, siendo PI = 8192. Es recontra r�pido.
//Usa taylor de no se que grado, re afanado de un blog

int SinApprox(int x) {
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
	return SinApprox((int2PI >> 2) - in);
}

int hamming(int m, int k) {
	return ALPHA - (BETA * CosAprox(int2PI * m / NFreq[k]) >> PRECISION);
}

void cqt() {
	unsigned int k, i, indx;
	int windowed, angle;
	float real_f, imag_f;
	int real, imag;
	for (k = 0; k < indice_bajas; ++k) {
		indx = nrInterrupts % NRSAMPLES - 1 + 8 * NRSAMPLES;
		real = ALPHA - (BETA * signal_lowfreq[indx % NRSAMPLES] >> PRECISION);
		imag = 0;
		for (i = 1; i < NFreq[k]; ++i) {
			windowed = hamming(i, k)
					* signal_lowfreq[(indx - i * Div[k]) % NRSAMPLES];
			angle = dosPiQ * i / NFreq[k];
			real += windowed * CosAprox(angle) >> PRECISION;
			imag += windowed * SinApprox(angle) >> PRECISION;
		}

		real_f = real / (float) SCALE;
		imag_f = imag / (float) SCALE;
		freqs[k] = logf(
				powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1)
				* amplitud / 32;
	}
	for (; k < FREQSBands; ++k) {
		indx = nrInterrupts % NRSAMPLES - 1 + 8 * NRSAMPLES;
		real = ALPHA - (BETA * signal[indx % NRSAMPLES] >> PRECISION);
		imag = 0;
		for (i = 1; i < NFreq[k]; ++i) {
			windowed = hamming(i, k) * signal[(indx - i * Div[k]) % NRSAMPLES];
			angle = dosPiQ * i / NFreq[k];
			real += windowed * CosAprox(angle) >> PRECISION;
			imag += windowed * SinApprox(angle) >> PRECISION;
		}
		real_f = real / (float) SCALE;
		imag_f = imag / (float) SCALE;

		freqs[k] = logf(
				powf(real_f * real_f + imag_f * imag_f, 0.5) / NFreq[k] + 0.1)
				* amplitud / 32;
	}
}

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

	/* ------------- INICIALIZACIONES ------------- */

	/* Inicializar la placa */
	boardConfig();

	/* Inicializar el conteo de Ticks con resolución de 1ms, sin tickHook */
	tickConfig(1, 0);

	/* Inicializar DigitalIO */
	gpioConfig(0, GPIO_ENABLE);

	/* Configuración de pines de entrada para Teclas de la CIAA-NXP */
	gpioConfig(TEC1, INPUT);
	gpioConfig(TEC2, INPUT);
	gpioConfig(TEC3, INPUT);
	gpioConfig(TEC4, INPUT);
	/* Configuración de pines de salida para Leds de la CIAA-NXP */
	gpioConfig(LEDR, OUTPUT);
	gpioConfig(LEDG, OUTPUT);
	gpioConfig(LEDB, OUTPUT);
	gpioConfig(LED1, OUTPUT);
	gpioConfig(LED2, OUTPUT);
	gpioConfig(LED3, OUTPUT);

	/* Inicializar UART_USB a 115200 baudios */
	uartConfig(UART_USB, 115200);

	uint16_t muestras[NRSAMPLES];
	/* Inicializar AnalogIO */
	/* Posibles configuraciones:
	 *    ENABLE_ANALOG_INPUTS,  DISABLE_ANALOG_INPUTS,
	 *    ENABLE_ANALOG_OUTPUTS, DISABLE_ANALOG_OUTPUTS
	 */
	adcConfig(ADC_ENABLE); /* ADC */
	dacConfig(DAC_ENABLE); /* DAC */

	/*
	 * Configurar frecuencia de muestreo ( f sample ) del ADC
	 * al maximo (400.000 muestras
	 *
	 */
	ADC_CLOCK_SETUP_T ADCSetup;
	Chip_ADC_SetSampleRate(LPC_ADC0, &ADCSetup, ADC_MAX_SAMPLE_RATE);

	/* Configuración de estado inicial del Led */
	bool_t ledState1 = OFF;

	/* Contador */
	uint16_t i = 0;

	/* Buffer */
	static uint8_t uartBuff[10];

	/* Variable para almacenar el valor leido del ADC CH1 */
	uint16_t muestra = 0;
	uint16_t caso;

	/* Variables de delays no bloqueantes */
	delay_t delay1;
	delay_t delay2;
	delay_t delay3;

	/* Inicializar Retardo no bloqueante con tiempo en ms */
	delayConfig(&delay1, 60);
	delayConfig(&delay2, 200);
	delayConfig(&delay3, 5);

	uint8_t* num = 0;

	/* ------------- REPETIR POR SIEMPRE ------------- */
	while (1) {
		if (delayRead(&delay3)) {
			uint16_t z = 0;
			for (; z < 1023; z++) {

				signal[z] = (uint16_t) adcRead(AI0);

			}
			num++;
			uartWriteString(UART_USB, (uint8_t*) "Fin de muestreo:   ");
			itoa(num, uartBuff, 10); /* 10 significa decimal */

			uartWriteString(UART_USB, uartBuff);
			uartWriteString(UART_USB, (uint8_t*) " \r\n ");

		}

		/* delayRead retorna TRUE cuando se cumple el tiempo de retardo */

		if (delayRead(&delay1)) {

			/* Leo la Entrada Analogica AI0 - ADC0 CH 1 */
			muestra = adcRead(AI0);

			/* Envío la primer parte del mnesaje a la Uart */
			//uartWriteString(UART_USB, (uint8_t*) "AI0 value: ");
			caso = (muestra < 256) ? 0 : (muestra >= 256 && muestra < 512) ? 1 :
					(muestra >= 512 && muestra < 768) ? 2 : 3;

			switch (caso) {
			case 0: {
				gpioWrite(LEDR, ON);
				gpioWrite(LEDG, ON);
				gpioWrite(LEDB, ON);
				gpioWrite(LED1, OFF);
				gpioWrite(LED2, OFF);
				gpioWrite(LED3, OFF);

				break;
			}

			case 1:
				gpioWrite(LEDR, OFF);
				gpioWrite(LEDG, OFF);
				gpioWrite(LEDB, OFF);
				gpioWrite(LED1, ON);
				gpioWrite(LED2, OFF);
				gpioWrite(LED3, OFF);

				break;

			case 2:
				gpioWrite(LEDR, OFF);
				gpioWrite(LEDG, OFF);
				gpioWrite(LEDB, OFF);
				gpioWrite(LED1, OFF);
				gpioWrite(LED2, ON);
				gpioWrite(LED3, OFF);

				break;

			case 3:
				gpioWrite(LEDR, OFF);
				gpioWrite(LEDG, OFF);
				gpioWrite(LEDB, OFF);
				gpioWrite(LED1, OFF);
				gpioWrite(LED2, OFF);
				gpioWrite(LED3, ON);

				break;
			}

			/* Conversión de muestra entera a ascii con base decimal */
			//itoa(muestra, uartBuff, 10); /* 10 significa decimal */
			/* Enviar muestra y Enter */
			//uartWriteString(UART_USB, uartBuff);
			//uartWriteString(UART_USB, (uint8_t*) ";\r\n");
			/* Escribo la muestra en la Salida AnalogicaAO - DAC */
			//analogWrite(AO, muestra);
		}

		/* delayRead retorna TRUE cuando se cumple el tiempo de retardo */
//		if (delayRead(&delay2)) {
//			if (ledState1)
//				ledState1 = OFF;
//			else
//				ledState1 = ON;
//			gpioWrite(LED1, ledState1);
//
//			/* Si pasaron 20 delays le aumento el tiempo */
//			i++;
//			if (i == 20)
//				delayWrite(&delay2, 1000);
//		}
	}

	/* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
	 por ningun S.O. */
	return 0;
}

/*==================[end of file]============================================*/
