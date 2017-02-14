/*==================[inclusions]=============================================*/

#include "sAPI.h"
#include "stdio.h"

/*==================[macros and definitions]=================================*/

#define TICKRATE_HZ (1000) /* 1000 ticks per second --> 1ms Tick */
#define RETRASOCLOCKS 6

volatile uint32_t msTicks = 0;
uint32_t ledStatus = OFF;
uint32_t delay_ms = 0;
uint32_t tiempoCumplido = 0;
//uint8_t dataToSend[8]={0,1,2,3,4,5,6,7};
uint8_t dataToSend[8]={8,8,8,8,8,8,8,8};
volatile uint32_t sendClk = 0;
/*==================[internal functions definition]==========================*/





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
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLRLEDS,ON);
}

 void shiftInitT(void) {
	uint8_t i;
	gpioWrite(CLRGND,OFF);
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLRGND,ON);
}

void shiftClock(void) {
	volatile uint8_t i;
	gpioWrite(CLKLEDS,ON);
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLKLEDS,OFF);
	for (i = 0; i <= RETRASOCLOCKS; i++);
}

void shiftClockT(void) {
	volatile uint8_t i;
	gpioWrite(CLKGND,ON);
	for (i = 0; i <= RETRASOCLOCKS; i++);
	gpioWrite(CLKGND,OFF);
	for (i = 0; i <= RETRASOCLOCKS; i++);
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
	for(i=0;i<8;i++)
	{
		d=*(data+i);
		shiftData(d);
		shiftInit();
		shiftClockT();
	}
}



/*==================[external functions definition]==========================*/



/* Set up and initialize board hardware */




/*int main(void) {
	/* perform the needed initialization here
	int i,k;
	long j;
//	uint8_t dataToSend[8]={0,0,0,0,0,0,0,0};
//	uint8_t dataToSend[8]={1,1,1,1,1,1,1,1};
//	uint8_t dataToSend[8]={2,2,2,2,2,2,2,2};
//	uint8_t dataToSend[8]={3,3,3,3,3,3,3,3};
//	uint8_t dataToSend[8]={4,4,4,4,4,4,4,4};
//	uint8_t dataToSend[8]={5,5,5,5,5,5,5,5};
//	uint8_t dataToSend[8]={6,6,6,6,6,6,6,6};
//	uint8_t dataToSend[8]={7,7,7,7,7,7,7,7};

	int waitTime = 100;

	boardInit();
	uint32_t TICKS_ADC = Timer_microsecondsToTicks(40);//25 KHz

	Timer_Init(TIMER0, TICKS_ADC, ADC_IRQ);
	delay_ms = 100;
//prueba
//	gpioWrite(DATAGND,ON);
//	shiftClockT(); // tiempo para pasar el bit que prende las diferentes columnas
//	gpioWrite(DATAGND,OFF);
//	gpioWrite(DATALEDS,ON);
//	shiftClock(); //1
//	shiftClock(); //2
//	shiftClock(); //3
//	shiftClock(); //4
//	shiftClock(); //5
//	shiftClock(); //6
//	shiftClock(); //7
//	shiftClock(); //8
//	shiftClock(); //9
//	gpioWrite(DATALEDS,OFF);
//prueba
	while (1) {
		j++;
		if(j==20000)
		{
			for(i=0;i<8;i++)
			{
				if (dataToSend[1]==8)
				{
					dataToSend[i]=0;//(dataToSend[i]+1)%9; //= valores de la cqt
				}
				else
				{
					dataToSend[i]=8;
				}

			}
			j=0;
		}
	//	ledsControl(dataToSend);
//prueba
//		gpioWrite(DATALEDS,ON);
//		shiftClock();
//		gpioWrite(DATALEDS,OFF);
//		for (i=0;i<9;i++)
//		{
//			shiftClock();
//		}
//prueba
		// resultado: incluso prendiendo los leds problematicos individualmente siguen atenuandose
		// conclusion: falta de corriente/tension
		// nueva pregunta: en que se diferencia el circuito que enciende el led problematico del normal?
		// solucion parcial: 2/3 leds problematicos arreglados (diagnostico: mal contacto en soldadura)
	}

	return 0;
}
*/
