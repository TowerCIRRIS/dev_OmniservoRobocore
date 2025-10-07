/**
 * @file    uartApp.h
 * @author  Simon Latour
 *
 * created on: Jan 6, 2022
 *
 */

#ifndef TEAMATAPPS_UARTAPP_H_
#define TEAMATAPPS_UARTAPP_H_
#include "usart.h"
#include "uartManager_V1_0.h"
#include "string.h"

#ifdef __cplusplus
 extern "C" {
#endif


//#define SERIALDEBUG_BUFFER_LINE_SIZE	 128  				//Nb caractères maximale dans la ligne
//extern char debugSerialInBuffer[SERIALDEBUG_BUFFER_LINE_SIZE];		// Buffer pour la réception de données du port série
//extern char debugSerialReceivedLine[SERIALDEBUG_BUFFER_LINE_SIZE]; // Buffer de travail, lorsqu'une ligne complète est reçue, elle est copié dans ce buffer
//extern uint8_t debugSerialReceivedLineLenght;					// Information sur la longueur de la dernière ligne reçue.


//#define UART1_TX_BUFFER_SIZE 300
#define UART1_RX_BUFFER_SIZE (300+1)
#define UART1_TX_BUFFER_SIZE (300+1)

//extern char debugSerialOutBuffer[UART1_TX_BUFFER_SIZE];	// Port série débug
//extern uint8_t debugSerialBufferIndex;


//#define 	bufferSize 128
//extern char inputBuffer[bufferSize];
//extern int 	bufferPointer;
//extern char uart1_receivedChar;
//extern bool uart1_dataReadyFlag;

#define ATUART_TIMEOUT_TRY_ONCE	0
#define ATUART_TIMEOUT_INFINITE	0xFFFFFFFF


 ///// Uart manager /////
 extern uartManager serial1;


void uartAppInit();
UART_HandleTypeDef * getUartHandle(uint32_t uartId);


void usbSerialOut(const char* outData);
//
//static inline void usbSerialOut(const char* outData);
//static inline void usbSerialOut(const char* outData)
//{
//	//uint32_t error =
//	size_t bufsize = strlen(outData);
//	HAL_UART_Transmit_IT(&hlpuart1, (uint8_t*)outData, bufsize);
//};


//static inline void serialRxCompleteCallback()
//{
//	if (bufferPointer < bufferSize - 1) {
//		inputBuffer[bufferPointer++] = uart1_receivedChar;
//	}
//
//	if(uart1_receivedChar == '\n' || uart1_receivedChar == '\r' )
//	{
//		inputBuffer[bufferPointer++] = '\0';
//		uart1_dataReadyFlag = true;
//	}
//
//	HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) &uart1_receivedChar, 1);
//
//
//};




#ifdef __cplusplus
 }
#endif
#endif /* TEAMATAPPS_UARTAPP_H_ */
