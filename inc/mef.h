/*
 * mef.h

 *
 *  Created on: 29/10/2016
 *      Author: arcant
 */

#ifndef EXAMPLES_EJ07_SAPI_BM_ADC_DAC_INC_MEF_H_
#define EXAMPLES_EJ07_SAPI_BM_ADC_DAC_INC_MEF_H_

void f_norm(void);
void f_spec_config(void);
void f_fir_config(void);
void f_spec (void);
void f_fir(void);
void MEF_reinicio (void);
void (*MEF[])(void);




void Init_MEF(void);
void UpdateMEF(unsigned char tecla);

#endif /* EXAMPLES_EJ07_SAPI_BM_ADC_DAC_INC_MEF_H_ */

