#include "main.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "stm32g4xx_it.h"
#include "mainApp.h"
#include "stdio.h"

//TODO Gestion et partage des erreurs
//TODO Optimisation de la comm au besoin

char serialOutBuffer[300];	// Port série débug

uint32_t timeoutCounter = 0; // reset after 0.5s (50 counts @ 100Hz)
bool successfulCommFlag = false;

bool sendCurrentAngleFlag = false;
bool sendCurrentSpeedFlag = false;
bool sendCurrentModeFlag = false;
bool sendCurrentUnitsFlag = false;
bool sendCurrentCurrentFlag = false;
bool sendCurrentTempFlag = false;
bool sendTorqueConstantFlag = false;
bool sendCurrentIdFlag = false;
bool sendMotorConfigInfoFlag = false;


uint8_t speedFreqCount = 0;
bool commandUpdateFlag = false;

OmniServo servo(0.001);

atComm txComm(ATCOMM_BUFFER_SIZE);
atComm rxComm(ATCOMM_BUFFER_SIZE);

PIDvalues rxPIDvalues;
UnitsCommand rxUnitsCommand;
ReferenceCommand rxReferenceCommand;
MotorCommand rxMotorCommand;


newIdCommand motorIdChangeBuffer;

uint8_t serialDataBuffer[SERIAL_DATA_BUFFER_SIZE];

// Variables de gestion de communication
char uart3_receivedChar;

#define RS485_TX_ENABLE GPIO_PIN_SET
#define RS485_RX_ENABLE GPIO_PIN_RESET

void manageReceivedData();
bool dataToSend();
void checkToSend();
void updateServoCommand();

void setup()
{
	HAL_Delay(50);

	//Idle au départ, config pour choisir idle ou brake
	TIM2->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	TIM4->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);

	servo.init();
	txComm.resetBuffer();
	rxComm.resetBuffer();
	// On débute le RS-485 en RX
	HAL_GPIO_WritePin(OUT_RS485_TXEN_GPIO_Port, OUT_RS485_TXEN_Pin, RS485_RX_ENABLE);
	// Activation de l'interruption de réception du port série
	HAL_UART_Receive_DMA(&huart3, (uint8_t*) &uart3_receivedChar, 1);
}

volatile uint32_t tick_10khz = 0;
volatile uint32_t tickCounter = 0;
volatile uint32_t counter_10ms = 0;
volatile uint32_t tickCounter1 = 0;
volatile uint32_t tickCounter2 = 0;
volatile uint32_t tickCounter3 = 0;
volatile uint32_t tickCounter4 = 0;
volatile uint32_t tickCounter4Last = 0;
volatile uint32_t servoCommandTime = 0;
volatile uint32_t tickCounter5 = 0;
volatile uint32_t tickCounter6 = 0;
volatile uint32_t tickCounterLast = 0;
volatile uint32_t maxLoopTime =0;
volatile uint32_t loopTime = 0;
volatile float filteredloopTime = 0;
volatile uint32_t commTime = 0;
volatile uint32_t commTimeMax = 0;
volatile uint32_t ledtimer = 0;

void loop()
{

	tickCounter1 = tick_10khz;
	manageReceivedData();
	tickCounter2 = tick_10khz;
	checkToSend();
	tickCounter3 = tick_10khz;

	commTime = tickCounter3 - tickCounter1;
	if(commTime > commTimeMax)
	{
		commTimeMax = commTime;
	}

	if (commandUpdateFlag) {
		commandUpdateFlag = false;

		ledtimer++;
		if(ledtimer >= 1000)
		{
			ledtimer = 0;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}

		tickCounter4 = tick_10khz;
		servoCommandTime = tickCounter4 - tickCounter4Last;
		tickCounter4Last = tickCounter4;
		updateServoCommand();
		tickCounter5= tick_10khz - tickCounter4; // temps de calcul de la commande
	}
	tickCounterLast = tickCounter6;
	tickCounter6= tick_10khz;
	loopTime = (tickCounter6 - tickCounterLast);
	if(loopTime > 5)
	{
		filteredloopTime++; // = (float)loopTime *0.001 + filteredloopTime*0.999;
	}


	if( loopTime > maxLoopTime)
	{
		maxLoopTime = loopTime ;
	}
}

// Gestionnaire de la réception de message
void manageReceivedData() {
	bool boolDataRx = false;
	uint8_t uint8DataEx = 0;
	int dataStatus = rxComm.validateData();
	if (dataStatus == ATCOMM_SUCCESS) {
		successfulCommFlag = true; // If any Servo message is successfully received, we acknowledge the master's activity
		if (rxComm.getDestinationId() == servo.reqMotorID() || rxComm.getDestinationId() == BROADCAST) {
			rxComm.lockBuffer();
			int dataCount = rxComm.getDataCount();
			if(dataCount > 0) {
				for (int i = 0; i < dataCount; i++) {
					dataInfo_t dInfo;
					uint8_t uint8DataRx;
					if(ATCOMM_SUCCESS == rxComm.getDataInfo(i,&dInfo)) {
						switch (dInfo.dataType)
						{
						case dataType_CurrentAngleRadReq:
							sendCurrentAngleFlag = true;
							break;
						case dataType_CurrentSpeedReq:
							sendCurrentSpeedFlag = true;
							break;
						case dataType_CurrentModeReq:
							sendCurrentModeFlag = true;
							break;
						case dataType_CurrentUnitsReq:
							sendCurrentUnitsFlag = true;
							break;
						case dataType_CurrentCurrentReq:
							sendCurrentCurrentFlag = true;
							break;
						case dataType_CurrentTempReq:
							sendCurrentTempFlag = true;
							break;
						case dataType_RequestID:
							sendCurrentIdFlag = true;
							break;
						case dataType_RequestMotorConfigInfo:
							sendMotorConfigInfoFlag = true;
							break;
						case dataType_RestoreFactorySettings:
							servo.restoreFactorySettings(KEEP_ID);
							break;

						case dataType_DriveMode:
							rxComm.getData(dInfo, &uint8DataEx, dInfo.dataLen);
							servo.setDriveMode(uint8DataEx);
							break;

						case dataType_SaveConfigToFlash:
							servo.saveConfigToFlash();
							break;

						case dataType_PositionPIDvalues:
							rxComm.getData(dInfo, &rxPIDvalues, dInfo.dataLen);
							if (rxPIDvalues.MotorID == servo.reqMotorID()) {
								servo.asgPIDpositionValues(rxPIDvalues.kp,rxPIDvalues.kd,rxPIDvalues.ki, rxPIDvalues.filter);
							}
							break;
						case dataType_VelocityPIDvalues:
							rxComm.getData(dInfo, &rxPIDvalues, dInfo.dataLen);
							if (rxPIDvalues.MotorID == servo.reqMotorID()) {
								servo.asgPIDspeedValues(rxPIDvalues.kp,rxPIDvalues.kd,rxPIDvalues.ki);
							}
							break;
						case dataType_UnitsCommand:
							rxComm.getData(dInfo, &rxUnitsCommand, dInfo.dataLen);
							if (rxUnitsCommand.MotorID == servo.reqMotorID()) {
								servo.changeWorkingUnits((WorkingUnits)rxUnitsCommand.unitType);
							}
							break;
						case dataType_ReferenceCommand:
							rxComm.getData(dInfo, &rxReferenceCommand, dInfo.dataLen);
							if (rxReferenceCommand.MotorID == servo.reqMotorID()) {
								switch (rxReferenceCommand.refMode) {
									case 0:
										servo.setTurns(rxReferenceCommand.nbTurns);
										break;
									case 1:
										servo.resetTo360();
										break;
									case 2:
										servo.setOrigin();
										break;
									case 3:
										servo.setOrigin(rxReferenceCommand.originAngle);
										break;
									case 4:
										servo.setCurrentAngle(rxReferenceCommand.originAngle);
										break;
									default:
										break;
								}
								if (rxReferenceCommand.centerAngleFlag) {
									servo.setCenterReadAngle(rxReferenceCommand.centerAngle);
								}
								if (rxReferenceCommand.torqueConstantFlag) {
									servo.setTorqueConstant(rxReferenceCommand.torqueConstant);
								}
								if (rxReferenceCommand.switchRefFlag) {
									servo.switchRef();
								}
							}
							break;

						case dataType_MotorCommand:

							rxComm.getData(dInfo, (MotorCommand*)&rxMotorCommand, dInfo.dataLen);
							if (rxMotorCommand.MotorID == servo.reqMotorID()) {
								if ((ServoModes)rxMotorCommand.controlMode != servo.reqCurrentMode()) {
									servo.changeMode((ServoModes)rxMotorCommand.controlMode);
								}
								else {
									servo.sendComand(rxMotorCommand.command);
								}
							}
							break;

						case dataType_ControlEnableRequest:
							rxComm.getData(dInfo, (bool*)&boolDataRx, dInfo.dataLen);
							servo.changeControlEnable(boolDataRx);
							break;

						case dataType_ChangeID:
							rxComm.getData(dInfo, &motorIdChangeBuffer, dInfo.dataLen);
							if(motorIdChangeBuffer.MotorID == servo.reqMotorID())
							{
								servo.changeMotorID(motorIdChangeBuffer.newID);
							}

							break;

						case dataType_ChangeModeRequest:
							rxComm.getData(dInfo, &uint8DataRx, dInfo.dataLen);
							servo.changeMode((ServoModes)uint8DataRx);
							break;


						case dataType_Reset:
							servo.reset();
							break;

						case dataType_TorqueConstantReq:
							sendTorqueConstantFlag = true;
							break;
						// Implement other cases
						default:
							// Unknown data type, ignore or handle error
						break;
						}
					}
				}
			}
			rxComm.unlockBuffer();
		}
		rxComm.resetBuffer();
	}
}

bool dataToSend() {
	return (sendCurrentAngleFlag || sendCurrentSpeedFlag || sendCurrentModeFlag || sendCurrentUnitsFlag
			|| sendCurrentCurrentFlag || sendCurrentTempFlag || sendTorqueConstantFlag || sendCurrentIdFlag
			|| sendMotorConfigInfoFlag);
}
// Gestionnaire de l'envoi de message
void checkToSend() {

	if (dataToSend()) {
		// Envoi d'un nouveau message
		txComm.startNewMessage(servo.reqMotorID(), MASTER_ID);
		if (sendCurrentAngleFlag) {
			sendCurrentAngleFlag = false;
			float angleBuffer = servo.reqCurrentAngle();
			txComm.addData(dataType_CurrentAngleRadReq, sizeof(angleBuffer), &angleBuffer);
		}
		if (sendCurrentSpeedFlag) {
			sendCurrentSpeedFlag = false;
			float speedBuffer = servo.reqCurrentSpeed();
			txComm.addData(dataType_CurrentSpeedReq, sizeof(speedBuffer), &speedBuffer);
		}
		if (sendCurrentModeFlag) {
			sendCurrentModeFlag = false;
			uint8_t modeBuffer = servo.reqCurrentMode();
			txComm.addData(dataType_CurrentModeReq, sizeof(modeBuffer), &modeBuffer);
		}
		if (sendCurrentUnitsFlag) {
			sendCurrentUnitsFlag = false;
			uint8_t unitsBuffer = servo.reqWorkingUnits();
			txComm.addData(dataType_CurrentUnitsReq, sizeof(unitsBuffer), &unitsBuffer);
		}
		if (sendCurrentCurrentFlag) {
			sendCurrentCurrentFlag = false;
			float currentBuffer = servo.reqCurrentCurrent();
			txComm.addData(dataType_CurrentCurrentReq, sizeof(currentBuffer), &currentBuffer);
		}
		if (sendCurrentTempFlag) {
			sendCurrentTempFlag = false;
			float tempBuffer = servo.reqCurrentTemp();
			txComm.addData(dataType_CurrentTempReq, sizeof(tempBuffer), &tempBuffer);
		}
		if (sendTorqueConstantFlag) {
			sendTorqueConstantFlag = false;

			float torqueConstantBuffer = servo.reqTorqueConstant();
			txComm.addData(dataType_TorqueConstantReq, sizeof(torqueConstantBuffer), &torqueConstantBuffer);
		}
		if(sendCurrentIdFlag) {
			sendCurrentIdFlag = false;
			uint8_t tempMotorID = servo.reqMotorID();
			txComm.addData(dataType_RequestID, sizeof(tempMotorID), &tempMotorID);
		}
		if(sendMotorConfigInfoFlag) {
			sendMotorConfigInfoFlag = false;
			ServoConfigInfo configData = servo.getConfigInfo();
			txComm.addData(dataType_motorConfigInfo, sizeof(configData), &configData);

		}

		txComm.setLastPacketStatus();
		txComm.completeMessage();
		int bytesRead = txComm.getSendPacket(serialDataBuffer, SERIAL_DATA_BUFFER_SIZE);
		HAL_GPIO_WritePin(OUT_RS485_TXEN_GPIO_Port, OUT_RS485_TXEN_Pin, RS485_TX_ENABLE);
		HAL_UART_Transmit_DMA(&huart3, serialDataBuffer, bytesRead);
	}
}

void updateServoCommand() {

	if (servo.reqCurrentMode() != SPEED)
	{
		servo.updateCommand();
	}
	speedFreqCount += 1;
	if (speedFreqCount >= 10)
	{
		speedFreqCount = 0;
		servo.computeSpeed();
		if (servo.reqCurrentMode() == SPEED)
		{
			servo.updateCommand();
		}
		servo.updateCurrentAndTemp();
	}
}

// Gestionnaire des interruptions de réception du port série
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == huart3.Instance)
	{
		rxComm.addReceivedBytes((uint8_t*) &uart3_receivedChar, 1);
		HAL_UART_Receive_DMA(&huart3, (uint8_t*) &uart3_receivedChar, 1);
	}
}

// Gestion du changement de mode du RS-485 après envoi du message
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart3.Instance)
	{
		// On remet le RS 485 en mode RX
		HAL_GPIO_WritePin(OUT_RS485_TXEN_GPIO_Port, OUT_RS485_TXEN_Pin, RS485_RX_ENABLE);
	}
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {

		commandUpdateFlag = true;
		tickCounter++;
		counter_10ms++;
		if(timeoutCounter>= 10)
		{
			counter_10ms = 0;
			timeoutCounter++;
			if (successfulCommFlag || !servo.getControlEnable())
			{
				timeoutCounter = 0;
				successfulCommFlag = false;
			}
			else
			{
				//TODO remettre, TEMPORAIRE
				//			timeoutCounter += 1;
				//			if (timeoutCounter >= 50) { // 0.5s for a timeout
				//				timeoutCounter = 0;
				//				rxComm.resetBuffer();
				//				servo.reset();
				//			}
			}

		}
	}

	if (htim == &htim6) {
		tick_10khz++;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == huart3.Instance)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_DMA(&huart3, (uint8_t*) &uart3_receivedChar, 1);
	}
}
