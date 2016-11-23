/*
 * backup_marco.c
 *
 *  Created on: 23/11/2016
 *      Author: arcant
 */


/*
 * /* Copyright 2014, 2015 Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 * Copyright 2015, Eric Pernia
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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

/** \brief Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials    Name
 * ---------------------------
 * ENP          Eric Pernia
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20151104   v0.0.1   ENP   First version
 */

/*==================[inclusions]=============================================*/
#include "sAPI.h"
/*==================[macros and definitions]=================================*/




/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

volatile uint32_t msTicks = 0;
uint32_t ledStatus = OFF;
uint32_t delay_ms = 0;
uint32_t tiempoCumplido = 0;
volatile uint32_t sendClk = 0;

/*==================[internal functions definition]==========================*/





static void boardGpiosInit(void) {
	gpioConfig(GPIO0,OUTPUT);
	gpioConfig(GPIO1,OUTPUT);
	gpioConfig(GPIO2,OUTPUT);
	gpioConfig(GPIO5,OUTPUT);
	gpioConfig(GPIO6,OUTPUT);
	gpioConfig(GPIO8,OUTPUT);
}

static void shiftInit(void) {
	uint8_t i;
	gpioWrite(GPIO0,OFF);
	for (i = 0; i <= 10; i++)
		;
	gpioWrite(GPIO0,ON);;
}

static void shiftInitT(void) {
	uint8_t i;
	gpioWrite(GPIO8,OFF);;
	for (i = 0; i <= 10; i++)
		;
	gpioWrite(GPIO8,ON);;
}

static void shiftClock(void) {
	volatile uint8_t i;
	gpioWrite(GPIO1,ON);
	for (i = 0; i <= 10; i++);
	gpioWrite(GPIO1,OFF);
	for (i = 0; i <= 10; i++);
}

static void shiftClockT(void) {
	volatile uint8_t i;
	gpioWrite(GPIO2,ON);;
	for (i = 0; i <= 10; i++)
		;
	gpioWrite(GPIO2,OFF);;
	for (i = 0; i <= 10; i++)
		;
}

static void shiftData(uint8_t dataToSend) {
	uint8_t i;
	shiftInit();
	gpioWrite(GPIO5,ON);;
	for (i = 0; i < dataToSend; i++) {
		shiftClock();
	}


}



void ledsControl(uint8_t* data)
{
	uint8_t i,d;
	gpioWrite(GPIO6,ON);
	shiftClockT();
	gpioWrite(GPIO6,OFF);
	shiftClockT();
	for(i=0;i<8;i++)
	{
		d=*(data+i);
		shiftData(d);
		shiftClockT();
	}
}





/*==================[external functions definition]==========================*/



/* Set up and initialize board hardware */
void boardInit(void) {

	boardGpiosInit();
	shiftInit();
	shiftInitT();
}


/*==================[end of file]============================================*/


