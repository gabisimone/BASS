#include "sAPI.h"

#define TICKRATE_HZ (1000) /* 1000 ticks per second --> 1ms Tick */

volatile uint32_t msTicks = 0;
uint32_t delay_ms = 0;
uint32_t tiempoCumplido = 0;
volatile uint32_t sendClk = 0;
/*==================[internal functions definition]==========================*/

 void boardButtonsInit(void) {
	/* Config EDU-CIAA-NXP Button Pins as GPIOs */
	gpioConfig(TEC1,INPUT);
	gpioConfig(TEC2,INPUT);
	gpioConfig(TEC3,INPUT);
	gpioConfig(TEC4,INPUT);
}

 void boardLedsInit(void) {

	/* Config EDU-CIAA-NXP Led Pins as GPIOs */
	gpioConfig(LEDR,OUTPUT);
	gpioConfig(LEDG,OUTPUT);
	gpioConfig(LEDB,OUTPUT);
	gpioConfig(LED1,OUTPUT);
	gpioConfig(LED2,OUTPUT);
	gpioConfig(LED3,OUTPUT);

}

 void boardGpiosInit(void) {
	gpioConfig(CLKLEDS,OUTPUT);
	gpioConfig(CLKGND,OUTPUT);
	gpioConfig(DATAGND,OUTPUT);
	gpioConfig(DATALEDS,OUTPUT);
	gpioConfig(CLRLEDS,OUTPUT);
	gpioConfig(CLRGND,OUTPUT);

}

 void shiftInit(void) {
	uint8_t i;
	gpioWrite(CLRLEDS,OFF);
	for (i = 0; i <= 10; i++);
	gpioWrite(CLRLEDS,ON);
}

 void shiftInitT(void) {
	uint8_t i;
	gpioWrite(CLRGND,OFF);
	for (i = 0; i <= 10; i++);
	gpioWrite(CLRGND,ON);
}

 void shiftClock(void) {
	volatile uint8_t i;
	gpioWrite(CLKLEDS,ON);
	for (i = 0; i <= 10; i++);
	gpioWrite(CLKLEDS,OFF);
	for (i = 0; i <= 10; i++);
}

 void shiftClockT(void) {
	volatile uint8_t i;
	gpioWrite(CLKGND,ON);
	for (i = 0; i <= 10; i++);
	gpioWrite(CLKGND,OFF);
	for (i = 0; i <= 10; i++);
}

void shiftData(uint8_t dataToSend) {
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
	//shiftClockT();
	for(i=0;i<8;i++)
	{
		d=*(data+i);
		shiftData(d);
		shiftClockT();
	}
}


/*==================[external functions definition]==========================*/




void boardInit(void) {


	/* Initializes GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/* Config EDU-CIAA-NXP Button Pins */
	boardButtonsInit();

	/* Config EDU-CIAA-NXP Led Pins */
	boardLedsInit();

	boardGpiosInit();

	shiftInit();
	shiftInitT();
}
