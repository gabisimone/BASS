#include "sAPI.h"
#include "stdio.h"

/*==================[macros and definitions]=================================*/

#define TICKRATE_HZ (1000) /* 1000 ticks per second --> 1ms Tick */
#define RETRASOCLOCKS 0

volatile uint32_t msTicks = 0;
uint32_t ledStatus = OFF;
uint32_t delay_ms = 0;
uint32_t tiempoCumplido = 0;
volatile uint32_t sendClk = 0;
/*==================[internal functions definition]==========================*/


static void boardGpiosInit(void) {
	gpioConfig(CLKLEDS,OUTPUT);
	gpioConfig(CLKGND,OUTPUT);
	gpioConfig(DATAGND,OUTPUT);
	gpioConfig(DATALEDS,OUTPUT);
	gpioConfig(CLRLEDS,OUTPUT);
	gpioConfig(CLRGND,OUTPUT);

}

static void shiftInit(void) {
	uint8_t i;
	gpioWrite(CLRLEDS,OFF);
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLRLEDS,ON);
}

static void shiftInitT(void) {
	uint8_t i;
	gpioWrite(CLRGND,OFF);
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLRGND,ON);
}

static void shiftClock(void) {
	volatile uint8_t i;
	gpioWrite(CLKLEDS,ON);
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLKLEDS,OFF);
//	for (i = 0; i <= RETRASOCLOCKS; i++);
}

static void shiftClockT(void) {
	volatile uint8_t i;
	gpioWrite(CLKGND,ON);
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLKGND,OFF);
//	for (i = 0; i <= RETRASOCLOCKS; i++);
}

static void shiftData(uint8_t dataToSend) {
	uint8_t i;
	shiftInit();
	gpioWrite(DATALEDS,ON);
	for (i = 0; i < dataToSend; i++) {
		shiftClock();
	}
}
void ledsControl(uint8_t* data)
{
	uint8_t i,d;
	gpioWrite(DATAGND,ON);
	shiftClockT(); // tiempo para pasar el bit que prende las diferentes columnas
	gpioWrite(DATAGND,OFF);
	for(i=0;i<8;i++)
	{
		d=*(data+i);
		d=(d>8)?d=8:(d<0)?d=0:d;
		shiftData(d);
		shiftInit();
		shiftClockT();
	}
}



/*==================[external functions definition]==========================*/



/* Set up and initialize board hardware */
void shiftBoardInit(void) {

	boardGpiosInit();
	shiftInit();
	shiftInitT();
}

/*==================[external functions definition]==========================*/




