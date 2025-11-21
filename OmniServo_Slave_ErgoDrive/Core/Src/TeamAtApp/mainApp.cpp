#include "main.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "stm32g4xx_it.h"
#include "mainApp.h"
#include "stdio.h"

#include "atCommOmniServoDefines.h"
#include "atComm.h"
#include "backupData_OmniServo.h"
#include "OmniServo_STM_Slave.h"

//TODO Gestion et partage des erreurs
//TODO Optimiser la communication si necessaire

char serialOutBuffer[300];	// Buffer du port serie pour debug

#define SERIAL_DATA_BUFFER_SIZE			1024
#define ATCOMM_BUFFER_SIZE				1042

extern atComm txComm;
extern atComm rxComm;
extern uint8_t serialDataBuffer[SERIAL_DATA_BUFFER_SIZE];

uint32_t timeoutCounter = 0; // remise a zero apres 0.5s (50 cycles @ 100Hz)
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
bool sendTemperatureLimitsFlag = false;
bool sendCurrentLimitsFlag = false;


uint8_t speedFreqCount = 0;
bool commandUpdateFlag = false;
bool calibrateInertiaFlag = false;


OmniServo servo(0.001);

atComm txComm(ATCOMM_BUFFER_SIZE);
atComm rxComm(ATCOMM_BUFFER_SIZE);

PIDvalues rxPIDvalues;
UnitsCommand rxUnitsCommand;
ReferenceCommand rxReferenceCommand;
MotorCommand rxMotorCommand;


newIdCommand motorIdChangeBuffer;

uint8_t serialDataBuffer[SERIAL_DATA_BUFFER_SIZE];

// Variables d'etat de communication
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

	// Demarre a l'arret (PWM 0) pour choisir ensuite frein ou idle
	TIM2->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	TIM4->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);

	servo.init();
	txComm.resetBuffer();
	rxComm.resetBuffer();
	// Demarre le RS-485 en mode RX
	HAL_GPIO_WritePin(OUT_RS485_TXEN_GPIO_Port, OUT_RS485_TXEN_Pin, RS485_RX_ENABLE);
	// Active l'interruption de reception serie
	HAL_UART_Receive_DMA(&huart3, (uint8_t*) &uart3_receivedChar, 1);
}

volatile uint32_t tick_10khz = 0;
volatile uint32_t counter_10ms = 0;
volatile uint32_t commTime = 0;
volatile uint32_t commTimeMax = 0;
volatile uint32_t ledtimer = 0;

void loop()
{

	manageReceivedData();
	checkToSend();

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

		updateServoCommand();
	}



}

// Reception: parse les trames valides et met a jour flags/commandes
void manageReceivedData()
{
	bool boolDataRx = false;
	uint8_t uint8DataRx = 0;
	float floatDataRx = 0;
	TemperatureLimits rxTemperatureLimits;
	CurrentLimits rxCurrentLimits;
	int dataStatus = rxComm.validateData();
	if (dataStatus == ATCOMM_SUCCESS)
	{
		successfulCommFlag = true; // Si un message Servo est recu, on confirme l'activite du maitre
		if (rxComm.getDestinationId()
				== servo.reqMotorID() || rxComm.getDestinationId() == BROADCAST)
		{
			rxComm.lockBuffer();
			int dataCount = rxComm.getDataCount();
			if (dataCount > 0)
			{
				for (int i = 0; i < dataCount; i++)
				{
					dataInfo_t dInfo;
					uint8_t uint8DataRx;
					if (ATCOMM_SUCCESS == rxComm.getDataInfo(i, &dInfo))
					{
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
						case dataType_TemperatureLimitsReq:
							sendTemperatureLimitsFlag = true;
							break;
						case dataType_CurrentLimitsReq:
							sendCurrentLimitsFlag = true;
							break;
						case dataType_RestoreFactorySettings:
							servo.restoreFactorySettings(KEEP_ID);
							break;

						case dataType_DriveMode:
							rxComm.getData(dInfo, &uint8DataRx, dInfo.dataLen);
							servo.setDriveMode(uint8DataRx);
							break;

						case dataType_SaveConfigToFlash:
							servo.saveConfigToFlash();
							break;

						case dataType_PositionPIDvalues:
							rxComm.getData(dInfo, &rxPIDvalues, dInfo.dataLen);
							if (rxPIDvalues.MotorID == servo.reqMotorID())
							{
								servo.setPositionPID(rxPIDvalues.kp,
										rxPIDvalues.kd, rxPIDvalues.ki,
										rxPIDvalues.filter);
							}
							break;

						case dataType_setMaxVelocity:
							rxComm.getData(dInfo, &floatDataRx, dInfo.dataLen);
							servo.setMaxVelocity(floatDataRx);
							break;
						case dataType_setMaxAcceleration:
							rxComm.getData(dInfo, &floatDataRx, dInfo.dataLen);
							servo.setMaxAcceleration(floatDataRx);
							break;
						case dataType_TemperatureLimits:
							rxComm.getData(dInfo, &rxTemperatureLimits, dInfo.dataLen);
							if (rxTemperatureLimits.MotorID == servo.reqMotorID())
							{
								servo.setTemperatureLimits(rxTemperatureLimits);
							}
							break;
						case dataType_CurrentLimits:
							rxComm.getData(dInfo, &rxCurrentLimits, dInfo.dataLen);
							if (rxCurrentLimits.MotorID == servo.reqMotorID())
							{
								servo.setCurrentLimits(rxCurrentLimits);
							}
							break;
						case dataType_VelocityPIDvalues:
							rxComm.getData(dInfo, &rxPIDvalues, dInfo.dataLen);
							if (rxPIDvalues.MotorID == servo.reqMotorID())
							{
								servo.setVelocityPID(rxPIDvalues.kp,
										rxPIDvalues.kd, rxPIDvalues.ki);
							}
							break;
						case dataType_UnitsCommand:
							rxComm.getData(dInfo, &rxUnitsCommand,
								dInfo.dataLen);
							if (rxUnitsCommand.MotorID == servo.reqMotorID())
							{
								servo.changeWorkingUnits(
										(WorkingUnits) rxUnitsCommand.unitType);
							}
							break;
						case dataType_ReferenceCommand:
							rxComm.getData(dInfo, &rxReferenceCommand,
								dInfo.dataLen);
							if (rxReferenceCommand.MotorID
									== servo.reqMotorID())
							{
								switch (rxReferenceCommand.refMode)
								{
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
									servo.setOrigin(
										rxReferenceCommand.originAngle);
									break;
								case 4:
									servo.setCurrentAngle(
										rxReferenceCommand.originAngle);
									break;
								default:
									break;
								}
								if (rxReferenceCommand.centerAngleFlag)
								{
									servo.setCenterReadAngle(
										rxReferenceCommand.centerAngle);
								}
								if (rxReferenceCommand.torqueConstantFlag)
								{
									servo.setTorqueConstant(
										rxReferenceCommand.torqueConstant);
								}
								if (rxReferenceCommand.switchRefFlag)
								{
									servo.switchRef();
								}
							}
							break;

						case dataType_MotorCommand:

						rxComm.getData(dInfo,
							(MotorCommand*) &rxMotorCommand,
							dInfo.dataLen);
						if (rxMotorCommand.MotorID == servo.reqMotorID())
						{
							if ((ServoModes) rxMotorCommand.controlMode
									!= servo.reqCurrentMode())
							{
								servo.changeMode(
										(ServoModes) rxMotorCommand.controlMode);
							}
							else
							{
								servo.sendComand(rxMotorCommand.command);
							}
						}
						break;

						case dataType_ControlEnableRequest:
							rxComm.getData(dInfo, (bool*) &boolDataRx,
								dInfo.dataLen);
							servo.changeControlEnable(boolDataRx);
							break;

						case dataType_ChangeID:
							rxComm.getData(dInfo, &motorIdChangeBuffer,
								dInfo.dataLen);
							if (motorIdChangeBuffer.MotorID
									== servo.reqMotorID())
							{
								servo.changeMotorID(motorIdChangeBuffer.newID);
							}

							break;

						case dataType_ChangeModeRequest:
							rxComm.getData(dInfo, &uint8DataRx, dInfo.dataLen);
							servo.changeMode((ServoModes) uint8DataRx);
							break;

						case dataType_Reset:
							servo.reset();
							break;

						case dataType_TorqueConstantReq:
							sendTorqueConstantFlag = true;
							break;
							// Ajouter autres cas si necessaire
						default:
							// Type de donnees inconnu, ignorer ou gerer l'erreur
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

bool dataToSend()
{
	return (sendCurrentAngleFlag || sendCurrentSpeedFlag || sendCurrentModeFlag
			|| sendCurrentUnitsFlag || sendCurrentCurrentFlag
			|| sendCurrentTempFlag || sendTorqueConstantFlag
			|| sendCurrentIdFlag || sendMotorConfigInfoFlag
			|| sendTemperatureLimitsFlag || sendCurrentLimitsFlag);
}

// Emission: regroupe la telemetrie en un seul message
void checkToSend() {

	if (dataToSend()) {
		// Envoie un nouveau message
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
		if(sendTemperatureLimitsFlag) {
			sendTemperatureLimitsFlag = false;
			TemperatureLimits tempLimits = servo.reqTemperatureLimits();
			txComm.addData(dataType_TemperatureLimitsReq, sizeof(tempLimits), &tempLimits);
		}
		if(sendCurrentLimitsFlag) {
			sendCurrentLimitsFlag = false;
			CurrentLimits currentLimits = servo.reqCurrentLimits();
			txComm.addData(dataType_CurrentLimitsReq, sizeof(currentLimits), &currentLimits);
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



// IRQ UART RX: injecte les octets recus dans le decodeur
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == huart3.Instance)
	{
		rxComm.addReceivedBytes((uint8_t*) &uart3_receivedChar, 1);
		HAL_UART_Receive_DMA(&huart3, (uint8_t*) &uart3_receivedChar, 1);
	}
}

// Apres TX, repasse le RS-485 en mode RX
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart3.Instance)
	{
		// Remet le RS-485 en RX
		HAL_GPIO_WritePin(OUT_RS485_TXEN_GPIO_Port, OUT_RS485_TXEN_Pin, RS485_RX_ENABLE);
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == hi2c1.Instance) {
		if (servo.m_encoder.handleTxComplete() != HAL_OK) {
			while (1) {
			}
		}
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == hi2c1.Instance)
	{
		if(servo.m_encoder.handleRxComplete() < 0)
		{
			while(1);
		}
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		tick_10khz++;
	}
	
	if (htim == &htim3) {

		if(servo.isInitDone())
		{
			int stat = servo.m_encoder.refreshRawAngle_DMA();
	//			if(stat != HAL_OK)
//			{
//				while(1); //TODO gerer les erreurs encodeur
//			}

		}      
		commandUpdateFlag = true;
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
				//TODO Reactiver le reset de timeout une fois le watchdog comm valide
				//			timeoutCounter += 1;
				//			if (timeoutCounter >= 50) { // 0.5s pour un timeout
				//				timeoutCounter = 0;
				//				rxComm.resetBuffer();
				//				servo.reset();
				//			}
			}

		}
	}


}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == huart3.Instance)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_DMA(&huart3, (uint8_t*) &uart3_receivedChar, 1);
	}
}
