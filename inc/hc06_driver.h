#ifndef _HC06_DRIVER_
#define _HC06_DRIVER_

#ifdef __cplusplus
extern "C" {
#endif
/*==================[inclusions]=============================================*/


/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[ISR external functions definition]======================*/



/*==================[external functions definition]==========================*/
void HC06_init(uint32_t baudRate );
uint8_t HC06_ReadByte( );
void HC06_WriteByte( uint8_t byte );
void HC06_WriteString( uint8_t * str );



/*==================[end of file]============================================*/
#ifdef __cplusplus
}
#endif

#endif /* #ifndef _HC06_DRIVER_ */