#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "stm32l4xx_it.h"
#include "mainApp.h"
#include "stdio.h"
#include <cstring>
#include <ctype.h>
#include <math.h>

#include "OmniServoSequencer.h"

/**
 * This example lets the user send different commands to 2 servos via serial commands
 * from a computer. They can be sent by typing the letter or numerical value followed by the ENTER key.
 * The commands are sent to a single motor at a time and the worked on servo can be switched with the (w) command.
 * If you only want to control a single servo, change both motor IDs to the used servo's ID.
 *
 * The motor can be controlled by sending float command values. The interpretation of the command is based on the currently used mode.
 * For example, sending 90 while working with degrees will have the following effects depending on the mode.
 * 		Disabled : Will do nothing.
 * 		Absolute Position : Will turn to 90 degrees.
 * 		Incremental Position : Will increment the turn position by 90 degrees.
 * 		Speed : Will make the motor spin at 90 deg/s

 * The possible commands are as follows :
 *
 * (p) : Changes to disabled mode
 * (a) : Changes to absolute position mode
 * (i) : Changes to incremental position mode
 * (s) : Changes to speed mode
 *
 * (r) : Resets the angles to [0,360[ deg or equivalent in rads
 * (o) : Sets the zero reference @ current location
 * (l) : Sets the zero reference @ given angle from hardware default
 * (j) : Changes the origin so that the current angle matches a given one
 * (k) : Switches the reference direction
 * (t) : Sets the center of the initial read angle range. After enabling this mode, the next numerical value
 * 		 will be the sent angle.
 * (y) : Sets the torque constant of the DC motor (in mNm/A). After enabling this mode, the next numerical value
 * 		 will be the sent constant.
 * (c) : Sets the number of turns. After enabling this mode, the next numerical value
 * 		 will be the sent number of turns.
 *
 * (u) : Switches between deg and rad units
 * (n) : Changes the PID values for the current mode (position or speed). After enabling this mode, the next 3 numerical values
 * 	     will be tied to kp, kd & ki in this order.
 *
 * (w) : Switches the currently worked on servo
 * (m) : Toggles the printing of current servo values
 *
 * (e) : Resets the servos
 * (q) : Changes the motor ID of every connected motor. After enabling this mode, the next numerical value will be the new motor ID.
 *
 * (float value) : Motor movement command
 */

#define PRINT_TIMEOUT false // Controls the printing of timeout



/* ------------------ Functions & Variables for the Example ------------------ */
void serialReading();
void switchServo();



const int bufferSize = 128;
char uart_receivedChar;
char uart2_receivedChar;
char inputBuffer[bufferSize];
const int bufferN = sizeof(inputBuffer) / sizeof(inputBuffer[0]);
int bufferPointer;
float bufferData;
char serialOutBuffer[300];

bool getZeroAngleFlag = false;
bool getCenterAngleFlag = false;
bool getTorqueConstant = false;
bool getNumberTurns = false;
bool getCurrentAngleFlag = false;

bool getIDflag = false;

bool getPIDflag = false;
float PIDvals[3];
int PIDinc = 0;

bool printValues = false;

long previousMillis5hz = 0;
long previousMillis20hz = 0;

float currentADD = 0;
uint8_t currentCounter = 0;

int8_t commStatus;

/* ------------------ Servo Communication Declaration ------------------ */
uint8_t m_uart2TxBuffer[OMNISERVO_COMM_BUFFER_SIZE];
uint8_t servoUart2TxCallback(int dataLen);

/* ------------------ Servo Objects Declaration ------------------ */
//OmniServo servo1(SLAVE_1_ID);
//OmniServo servo2(SLAVE_2_ID);
OmniServoSequencer omniSequencer(OMNISERVO_COMM_BUFFER_SIZE,m_uart2TxBuffer,&servoUart2TxCallback);

OmniServo* currentServo;
/* ------------------ Servo Objects Declaration ------------------ */

/* ------------------ Functions & Variables for the Example ------------------ */

void setup() {

	omniSequencer.addServo(SLAVE_1_ID);
	//omniSequencer.addServo(SLAVE_2_ID);

	currentServo = omniSequencer.getServo(0);

	// Power up device on port 1
	HAL_GPIO_WritePin(OUT_PORT1_POWER_GPIO_Port, OUT_PORT1_POWER_Pin, GPIO_PIN_SET);

	// Starting the Serial Comm with the computer
	HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) &uart_receivedChar, 1);
	HAL_Delay(1000);

	// Initialization of the Serial Comm for the servos
	//OmniServo::servoSerialInit(&huart2, RS485_DIR_GPIO_Port, RS485_DIR_Pin);
	HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, RS485_RX_ENABLE);
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &uart2_receivedChar, 1);
	omniSequencer.resetSlaves();

	sprintf(serialOutBuffer, "OmniServo Test Program Started !\r\n");
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), 1000);
}

void loop() {
	// Update for the communication of the servos
	//commStatus = OmniServo::updateServoCom();

	commStatus = omniSequencer.updateServoCom();
	// Dealing with errors with the communication if there are any
	if (commStatus < 0 && PRINT_TIMEOUT) {
		sprintf(serialOutBuffer, "TIMEOUT #%d\r\n", -commStatus);
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), 1000);
	}

	// Reading the serial buffer to check for any sent commands
	serialReading();

	// Printing current servo values if printing is enabled
	if (printValues) {
		if (HAL_GetTick() - previousMillis5hz >= 200) {
			previousMillis5hz = HAL_GetTick();
			if (printValues) {
				if (currentServo->getCurrentUnits() == DEGREES) {
					sprintf(serialOutBuffer,
							"ID: %d Angle : %.2f °  Vitesse : %.2f °/s  Courant : %.3f A  Couple : %.2f Nm  Temp : %.2f °C\r\n",
							currentServo->m_motorID,
							currentServo->getCurrentAngle(),
							currentServo->getCurrentSpeed(),
							currentServo->getCurrentCurrent(),
							currentServo->getCurrentTorque(),
							currentServo->getCurrentTemp());
				}
				else {
					sprintf(serialOutBuffer,
							"ID: %d Angle : %.2f rad  Vitesse : %.2f rad/s  Courant : %.3f A  Couple : %.2f Nm  Temp : %.2f °C\r\n",
							currentServo->m_motorID,
							currentServo->getCurrentAngle(),
							currentServo->getCurrentSpeed(),
							currentServo->getCurrentCurrent(),
							currentServo->getCurrentTorque(),
							currentServo->getCurrentTemp());
				}
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), 1000);
			}
		}
	}
}

void serialReading() {
	if (inputBuffer[bufferPointer-1] == '\n' || inputBuffer[bufferPointer-1] == '\r' ) {
		inputBuffer[bufferPointer++] = '\0';
		bufferData = atof(inputBuffer);

		// If command is not a number, look for characters
		if (bufferData == 0 && !isdigit(inputBuffer[0])) {
			switch (inputBuffer[0])
			{
			case 115: // (s) Check for speed mode
				sprintf(serialOutBuffer, "Speed mode\r\n");
				currentServo->changeMode(SPEED);
				break;
			case 97: // (a) Check for abs position mode
				sprintf(serialOutBuffer, "ABS position mode\r\n");
				currentServo->changeMode(ABS_POSITION);
				break;
			case 105: // (i) Check for incremental position mode
				sprintf(serialOutBuffer, "Incremental position mode\r\n");
				currentServo->changeMode(INC_POSITION);
				break;
			case 112: // (p) Check power off
				sprintf(serialOutBuffer, "Off mode\r\n");
				currentServo->changeMode(DISABLED);
				break;
			case 114: // (r) Check for reset of angles
				sprintf(serialOutBuffer, "Reset angles\r\n");
				currentServo->resetTo360();
				break;
			case 111: // (o) Check for zero setting
				sprintf(serialOutBuffer, "Setting zero reference\r\n");
				currentServo->setOrigin();
				break;
			case 108: // (l) Check for zero setting @ given angle
				getZeroAngleFlag = true;
				sprintf(serialOutBuffer, "Changing zero reference angle\r\nEnter angle : \r\n");
				break;
			case 106: // (j) Check for zero setting with current angle
				getCurrentAngleFlag = true;
				sprintf(serialOutBuffer, "Changing current angle\r\nEnter angle : \r\n");
				break;
			case 107: // (k) Check for reference direction switching
				sprintf(serialOutBuffer, "Switching the reference direction\r\n");
				currentServo->switchRef();
				break;
			case 116: // (t) Check for center angle
				getCenterAngleFlag = true;
				sprintf(serialOutBuffer, "Changing center angle\r\nEnter angle : \r\n");
				break;
			case 121: // (y) Check for torque constant
				getTorqueConstant = true;
				sprintf(serialOutBuffer, "Changing torque constant\r\nEnter torque constant : \r\n");
				break;
			case 99: // (c) Check for number of turns
				getNumberTurns = true;
				sprintf(serialOutBuffer, "Changing number of turns\r\nEnter nb turns : \r\n");
				break;
			case 101: // (e) Check to reset slaves
				//OmniServo::resetSlaves();
				omniSequencer.resetSlaves();
				sprintf(serialOutBuffer, "Resetting servos\r\n");
				break;
			case 110: // (n) Entering PID
				getPIDflag = true;
				sprintf(serialOutBuffer, "Changing PID Values\r\nEnter kp : \r\n");
				break;
			case 117: // (u) Switching units
				if (currentServo->getCurrentUnits() == DEGREES) {
					currentServo->changeWorkingUnits(RADIANS);
					sprintf(serialOutBuffer, "Switching to Radians\r\n");
				}
				else {
					currentServo->changeWorkingUnits(DEGREES);
					sprintf(serialOutBuffer, "Switching to Degrees\r\n");
				}
				break;
			case 109: // (m) Toggle print
				if (printValues) {printValues = false;}
				else {printValues = true;}
				sprintf(serialOutBuffer, "Toggle printing\r\n");
				break;
			case 119: // (w) Switch servo
				switchServo();
				break;
			case 113: // (q) Change ID
				getIDflag = true;
				sprintf(serialOutBuffer, "Enter new motor ID : \r\n");
				break;
			}
		}
		// Else, read the numerical command
		else {
			if (getZeroAngleFlag) {
				sprintf(serialOutBuffer, "Sending zero reference angle : %.2f\r\n", bufferData);
				currentServo->setOrigin(bufferData);
				getZeroAngleFlag = false;
			}
			else if (getCurrentAngleFlag) {
				sprintf(serialOutBuffer, "Sending current angle : %.2f\r\n", bufferData);
				currentServo->setCurrentAngle(bufferData);
				getCurrentAngleFlag = false;
			}
			else if (getCenterAngleFlag) {
				sprintf(serialOutBuffer, "Sending center angle : %.2f\r\n", bufferData);
				currentServo->setInitialCenterAngle(bufferData);
				getCenterAngleFlag = false;
			}
			else if (getTorqueConstant) {
				sprintf(serialOutBuffer, "Sending torque constant : %.2f\r\n", bufferData);
				currentServo->sendTorqueConstant(bufferData);
				getTorqueConstant = false;
			}
			else if (getNumberTurns) {
				sprintf(serialOutBuffer, "Sending number of turns : %d\r\n", (int16_t)bufferData);
				currentServo->setTurns((int16_t)bufferData);
				getNumberTurns = false;
			}
			else if (getIDflag) { // If we are getting new motor ID
				sprintf(serialOutBuffer, "Sending new motor ID : %d\r\n", (uint8_t)bufferData);
				//OmniServo::changeMotorID((uint8_t)bufferData);
				currentServo->changeID((uint8_t)bufferData);
				getIDflag = false;
			}
			else if (getPIDflag) { // If we are getting PID values
				PIDvals[PIDinc] = bufferData;
				PIDinc += 1;
				if (PIDinc == 1) {sprintf(serialOutBuffer, "Enter kd : \r\n");}
				else if (PIDinc == 2) {sprintf(serialOutBuffer, "Enter ki : \r\n");}
				if (PIDinc >= 3) {
					sprintf(serialOutBuffer, "Sending PID : P:%.2f D:%.2f I:%.2f\r\n",PIDvals[0],PIDvals[1],PIDvals[2]);
					PIDinc = 0;
					getPIDflag = false;
					currentServo->setPIDValues(currentServo->getCurrentMode(), PIDvals[0], PIDvals[1], PIDvals[2]);
				}
			}
			else { // Else it's a motor movement command
				sprintf(serialOutBuffer, "Command : %.2f\r\n",bufferData);
				currentServo->sendCommand(bufferData);
			}
		}
		// Empty the buffer and return the pointer to 0
		for (int i=0; i<bufferPointer; i++) {
			inputBuffer[i] = 0;
		}
		bufferPointer = 0;
		// Print the command
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), 1000);
	}
}

void switchServo() {
//    if (currentServo == &servo1) {currentServo = &servo2;}
//    else {currentServo = &servo1;}
	//if (currentServo == omniSequencer.getServo(0)) {currentServo = omniSequencer.getServo(1);}
	//else {currentServo = omniSequencer.getServo(0);}

    sprintf(serialOutBuffer, "On Servo #%d\r\n", currentServo->getMotorID());
}

/**
 * Cette fonction est un callback qui sera appelé par le gestionnaire de omniServo lorsque
 * le data a été copié dans le buffer et est prêt à être envoyer.
 */
uint8_t servoUart2TxCallback(int dataLen)
{
	if(dataLen > 0 && dataLen <= sizeof(m_uart2TxBuffer))
	{
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, RS485_TX_ENABLE);
		HAL_UART_Transmit_DMA(&huart2, m_uart2TxBuffer, dataLen);
		return 0; // TODO defines d'error code
	}

	return 1;
}
volatile HAL_StatusTypeDef uart2Rxstatus;

// Gestionnaire des interruptions de réception du port série
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if(huart->Instance == huart2.Instance) {
		omniSequencer.rxUartInterruptHandler((uint8_t*)&uart2_receivedChar, 1);
		uart2Rxstatus = HAL_UART_Receive_IT(&huart2, (uint8_t*) &uart2_receivedChar, 1);
	}


	//OmniServo::rxUartInterruptHandler(huart);
	if(huart->Instance == hlpuart1.Instance) {
		if (bufferPointer < bufferSize - 1) {
			inputBuffer[bufferPointer++] = uart_receivedChar;
		}
		HAL_UART_Receive_IT(&hlpuart1, (uint8_t*) &uart_receivedChar, 1);
	}
}

// Gestion du changement de mode du RS-485 après envoi du message
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	//OmniServo::txUartInterruptHandler(huart);
	if(huart->Instance == huart2.Instance) {
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, RS485_RX_ENABLE);
	}

}
