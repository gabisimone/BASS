/*
 * fft_kb.h
 *
 *  Created on: 29/10/2016
 *      Author: arcant
 */

#ifndef EXAMPLES_EJ07_SAPI_BM_ADC_DAC_INC_FFT_KB_H_
#define EXAMPLES_EJ07_SAPI_BM_ADC_DAC_INC_FFT_KB_H_
#include <complex.h>
typedef double complex cpx;
void fft(double complex x[], double complex y[], int dx, int N, int dir);

#endif /* EXAMPLES_EJ07_SAPI_BM_ADC_DAC_INC_FFT_KB_H_ */
