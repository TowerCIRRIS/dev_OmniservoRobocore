/**
 * @file    uartApp.cpp
 * @author  Simon Latour
 *
 * created on: Jan 6, 2022
 *
 */

#include "uartApp.h"

//#include "usart.h"
#include <string>
using namespace std;


uint8_t serialdebugonoff = 0;

// Variables pour gestion du port série de debug
	string ReadSerialPortVal; // Initialize variable to read on serial port
    string serialPortLineBuffer = "";

	bool debug_newLineReceived = false; // Drapeau indiquant qu'une ligne de commande a été reçu
	char debug_receivedChar;	// Dernier caractère reçu sur le port série

//	//#define SERIALDEBUG_BUFFER_LINE_SIZE	 128  				//Nb caractères maximale dans la ligne
//	char debugSerialInBuffer[1];		// Buffer pour la réception de données du port série
//	char debugSerialReceivedLine[SERIALDEBUG_BUFFER_LINE_SIZE]; // Buffer de travail, lorsqu'une ligne complète est reçue, elle est copié dans ce buffer
//	uint8_t debugSerialReceivedLineLenght = 0;					// Information sur la longueur de la dernière ligne reçue.
//	char debugSerialOutBuffer[UART1_TX_BUFFER_SIZE];	// Port série débug
//	uint8_t debugSerialBufferIndex = 0;
//
////	#define  IbufferSize = 128;
//	char inputBuffer[bufferSize];



	unsigned char uart1TxBuffer[UART1_TX_BUFFER_SIZE];
	unsigned char uart1RxBuffer[UART1_RX_BUFFER_SIZE];

	uint8_t uart1TxPortBuffer[UART1_TX_BUFFER_SIZE];
	uint8_t uart1RxPortBuffer[1];


	int bufferPointer = 0;

	serialPortHandle_t hSerial1;
	uartManager serial1(&hSerial1, uart1TxBuffer,UART1_TX_BUFFER_SIZE, uart1RxBuffer,UART1_RX_BUFFER_SIZE);


//	bool usb_newLineReceived = false; // Drapeau indiquant qu'une ligne de commande a été reçu
//	char usb_receivedChar;	// Dernier caractère reçu sur le port série
//
//
//	char usbSerialInBuffer[SERIALDEBUG_BUFFER_LINE_SIZE];		// Buffer pour la réception de données du port série
//	char usbSerialReceivedLine[SERIALDEBUG_BUFFER_LINE_SIZE]; // Buffer de travail, lorsqu'une ligne complète est reçue, elle est copié dans ce buffer
//	uint8_t usbSerialReceivedLineLenght = 0;					// Information sur la longueur de la dernière ligne reçue.
//	char usbSerialOutBuffer[300];	// Port série débug
//	uint8_t usbSerialBufferIndex = 0;


	void debugSerialInputManagement();			// Fonction de gestion du texte reçu sur le port série

	bool gSerial_1_TxBusy = false;


	// Variables pour le port série de communication
		bool gCommInProgressFlag = false;
		int gSerialTxBusyTimeoutCounter = 0;
		char uart1_receivedChar;
		bool uart1_dataReadyFlag = false;


void uartAppInit()
{

	bufferPointer = 0;


	hSerial1.uartId = UART1_ID;
	hSerial1.halfDuplexEnable = HALF_DUPLEX_DISABLED;
	hSerial1.rxBuff = uart1RxPortBuffer;
	hSerial1.txBuff = uart1TxPortBuffer;
	hSerial1.txInProgress = false;

	serial1.init(460800, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE);
	serial1.enableRxMode();
	serial1.startRxInterrupt(1);




//	// Starting the Serial Comm with the computer
//	HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) &uart1_receivedChar, 1);

}


void uart_Transmit(serialPortHandle_t *hSerial, uint32_t dataSize)
{
	UART_HandleTypeDef *huart;

	huart = getUartHandle(hSerial->uartId);

	if(huart->Instance == LPUART1)
	{
		HAL_UART_Transmit_IT(huart, hSerial->txBuff, dataSize);
	}

}


//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == LPUART1)
//	{
//		serial1.txCompleteCallback();
//	}
//}

UART_HandleTypeDef * getUartHandle(uint32_t uartId)
{
	switch(uartId)
		{
			case UART1_ID:
				return &hlpuart1;
			default:
				return NULL;
		}
}


void uart_portConfig(uint32_t uartId, uint32_t baud, uint32_t databits, uint32_t parity, uint32_t stopbits, HalfDuplexMode_t halfDuplex)
{
	UART_HandleTypeDef *huart;

	huart = getUartHandle(uartId);

	huart->Init.BaudRate = baud;
	switch(databits)
	{

		case 7:
			huart->Init.WordLength = UART_WORDLENGTH_7B;
			break;

		case 8:
			huart->Init.WordLength = UART_WORDLENGTH_8B;
			break;

		case 9:
			huart->Init.WordLength = UART_WORDLENGTH_9B;
			break;

		default:
			huart->Init.WordLength = UART_WORDLENGTH_8B;
		break;
	}

	switch(stopbits)
	{

		case 1:
			huart->Init.StopBits = UART_STOPBITS_1;
			break;

		case 2:
			huart->Init.StopBits = UART_STOPBITS_2;
			break;

		default:
			huart->Init.StopBits = UART_STOPBITS_1;
		break;
	}

	switch(parity)
	{

		case PARITY_NONE:
			huart->Init.Parity = UART_PARITY_NONE;
			break;

		case PARITY_EVEN:
			huart->Init.Parity = UART_PARITY_EVEN;
			break;

		case PARITY_ODD:
			huart->Init.Parity = UART_PARITY_ODD;
			break;

		default:
			huart->Init.StopBits = UART_STOPBITS_1;
		break;
	}


	if(halfDuplex == HalfDuplexMode_t::HALF_DUPLEX_ENABLED)
	  {
		if (HAL_HalfDuplex_Init(huart) != HAL_OK)
		{
		  Error_Handler();
		}
	  }
	  else
	  {
		if (HAL_UART_Init(huart) != HAL_OK)
		{
		  Error_Handler();
		}
	  }

}

void uart_enableReceiver(serialPortHandle_t *hSerial, int dataLen)
{
	if(hSerial->halfDuplexEnable)
	{
		uart_enableHalfDuplexReceiver(hSerial);
	}
}

void uart_enableTransmitter(serialPortHandle_t *hSerial)
{
	if(hSerial->halfDuplexEnable)
	{
		uart_enableHalfDuplexTransmitter(hSerial);
	}
}

void uart_enableHalfDuplexReceiver(serialPortHandle_t *hSerial)
{
	switch(hSerial->uartId)
	{
		case UART1_ID:
			HAL_HalfDuplex_EnableReceiver(&hlpuart1);
			break;

		default:
			break;
	}
}

void uart_startRxInterrupt(serialPortHandle_t *hSerial, int dataLen)
{
	HAL_StatusTypeDef status;

	switch(hSerial->uartId)
	{
		case UART1_ID:
			HAL_UART_AbortReceive_IT(&hlpuart1);
			HAL_UART_Receive_IT(&hlpuart1, hSerial->rxBuff, dataLen);
			break;

		default:
			break;
	}
}

void uart_stopRxInterrupt(serialPortHandle_t *hSerial)
{
	switch(hSerial->uartId)
	{
		case UART1_ID:
			HAL_UART_AbortReceive_IT(&hlpuart1);
			break;
		default:
				break;
	}
}

void uart_enableHalfDuplexTransmitter(serialPortHandle_t *hSerial)
{
	switch(hSerial->uartId)
		{
			case UART1_ID:
				HAL_HalfDuplex_EnableTransmitter(&hlpuart1);
				break;
			case UART2_ID:
//				HAL_HalfDuplex_EnableTransmitter(&huart2);
					break;
			case UART3_ID:
//				HAL_HalfDuplex_EnableTransmitter(&huart3);
						break;
			case UART4_ID:
	//			HAL_HalfDuplex_EnableTransmitter(&huart4);
						break;
			case UART5_ID:
	//			HAL_HalfDuplex_EnableTransmitter(&huart4);
						break;
		}
}

void usbSerialOut(const char* outData)
{
	size_t bufsize = strlen(outData);
	serial1.write(outData, bufsize);
};




/**
 * @fn void HAL_UART_RxCpltCallback(UART_HandleTypeDef*)
 * @brief Cette fonction est appelé lorsque du data a été reçu sur le port série.
 *
 * @details On accumule le data dans un buffer et lorsque l'utilisateur appui sur "enter" ou qu'on atteint le nombre de caractère maximale, on copie le contenu
 * 			du buffer dans un buffer de travail
 *
 * @param huart
 */
//void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
//{
////	if(huart->Instance == huart1.Instance){
////		debugSerialInBuffer[debugSerialBufferIndex] = debug_receivedChar;
////
////		debugSerialBufferIndex++;
////
////		if(debug_receivedChar == '\r' || (debugSerialBufferIndex == SERIALDEBUG_BUFFER_LINE_SIZE))
////		{
////
////			for(int i = 0; i < debugSerialBufferIndex; i++)	// Copy buffer de réception dans buffer de "traitement"
////			{
////				debugSerialReceivedLine[i] = debugSerialInBuffer[i];
////			}
////
////			debugSerialReceivedLineLenght = debugSerialBufferIndex;
////			debugSerialBufferIndex = 0;	// Remetsl'index a zéro pour prochain messages
////
////			debug_newLineReceived= true; // Signalement de nouvelle ligne reçue
////		}
////
////		HAL_UART_Receive_IT (huart, (uint8_t*) &debug_receivedChar, 1); // redémarre l'interruption de réception
////	}
//
////	if(huart->Instance == huart2.Instance)
////	{
////#ifdef ATCOMMEXAMPLE_UART
////		atCommApp_ReceiveBytes((uint8_t*) &uart2_receivedChar, 1);
////#endif
////		HAL_UART_Receive_IT (&huart2, (uint8_t*) &uart2_receivedChar, 1);
////
////	}
//
//}




