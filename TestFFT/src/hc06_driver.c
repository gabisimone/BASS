#include "chip.h"
#include "string.h"
#include "hc06_driver.h"


/*==================[macros]=================================================*/

#define UART_232_LPC LPC_USART3  /* UART3 (RS232) */

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

void HC06_init( uint32_t baudRate ){
   
      Chip_UART_Init(UART_232_LPC);
      Chip_UART_SetBaud(UART_232_LPC, baudRate);  /* Set Baud rate */
      Chip_UART_SetupFIFOS(UART_232_LPC, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0); /* Modify FCR (FIFO Control Register)*/
      Chip_UART_TXEnable(UART_232_LPC); /* Enable UART Transmission */
      Chip_SCU_PinMux(2, 3, MD_PDN, FUNC2);              /* P2_3,FUNC2: UART3_TXD */
      Chip_SCU_PinMux(2, 4, MD_PLN|MD_EZI|MD_ZI, FUNC2); /* P2_4,FUNC2: UART3_RXD */
 
}

uint8_t HC06_ReadByte(){

   uint8_t receivedByte = 0;


      if (Chip_UART_ReadLineStatus(UART_232_LPC) & UART_LSR_RDR) {
         receivedByte = Chip_UART_ReadByte(UART_232_LPC);
      }

   return receivedByte;
}

void HC06_WriteByte( uint8_t byte ){

      while ((Chip_UART_ReadLineStatus(UART_232_LPC) & UART_LSR_THRE) == 0) {}   // Wait for space in FIFO
      Chip_UART_SendByte(UART_232_LPC, byte);
  
}

void HC06_WriteString( uint8_t * str ){

   while(*str != 0){
	  HC06_WriteByte( *str );
	  str++;
   }
}

