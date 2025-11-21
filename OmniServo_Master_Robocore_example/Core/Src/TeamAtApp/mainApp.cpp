#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "stm32l4xx_it.h"
#include "mainApp.h"
#include "stdio.h"
#include <cstring>
#include <ctype.h>
#include <math.h>


#include "at_platformAbstraction_V1_1.h"
#include "teamATbasic_V1_1.h"
#include "OmniServoRCSequencer.h"
#include "uartApp.h"
#include "cliApp.h"

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

//void switchServo(int servoId = -1);




//char uart_receivedChar;
char uart2_receivedChar;
//char inputBuffer[bufferSize];
//const int bufferN = sizeof(inputBuffer) / sizeof(inputBuffer[0]);

float bufferData;
//char serialOutBuffer[300];

bool getZeroAngleFlag = false;
bool getCenterAngleFlag = false;
bool getTorqueConstant = false;
bool getNumberTurns = false;
bool getCurrentAngleFlag = false;

bool getIDflag = false;

bool getPIDflag = false;
bool getVelocityPIDflag = false;
float PIDvals[3];
int PIDinc = 0;

bool printValues = false;

long previousMillis5hz = 0;
long previousMillis20hz = 0;

float currentADD = 0;
uint8_t currentCounter = 0;

int8_t commStatus;

/* ------------------ Servo Communication Declaration ------------------ */
#define RS485_TX_ENABLE GPIO_PIN_SET
#define RS485_RX_ENABLE GPIO_PIN_RESET

uint8_t m_uart2TxBuffer[OMNISERVO_COMM_BUFFER_SIZE];
uint8_t servoUart2TxCallback(int dataLen);

char main_debugSerialOutBuffer[UART1_TX_BUFFER_SIZE];

/* ------------------ Servo Objects Declaration ------------------ */
OmniServoRCSequencer omniSequencer(OMNISERVO_COMM_BUFFER_SIZE,m_uart2TxBuffer,
			&servoUart2TxCallback,
			&HAL_GetTick
			);

//actuator_Omniservo* currentRCServo;
/* ------------------ Servo Objects Declaration ------------------ */

/* ------------------ Functions & Variables for the Example ------------------ */

void printServoConfig(actuator_Omniservo* servo);
/* ------------------- CLI & UART Declarations ------------------- */

	// Declaration of the CLI callable functions

int cmd_getMotorInfo(const char* argString);
int cmd_findServos(const char* argString);
int cmd_changeMode(const char* argString);
int cmd_disableControl(const char* argString);
int cmd_startControl(const char* argString);
int cmd_reset360(const char* argString);
int cmd_setZero(const char* argString);
int cmd_setZeroReference(const char* argString);
int cmd_setCurrentAngle(const char* argString);
int cmd_switchDirection(const char* argString);
int cmd_setCenterAngle(const char* argString);
int cmd_setTorqueConstant(const char* argString);
int cmd_setTurns(const char* argString);
int cmd_resetSlaves(const char* argString);
int cmd_setPID(const char* argString);
int cmd_switchUnits(const char* argString);
int cmd_togglePrint(const char* argString);
int cmd_switchServo(const char* argString);
int cmd_changeServoId(const char* argString);
int cmd_listServos(const char* argString);
int cmd_command(const char* argString);
int cmd_restoreFactory(const char* argString);
int cmd_syncServoConfigs(const char* argString);
int cmd_saveServoConfigs(const char* argString);
int cmd_changeDriveMode(const char* argString);
int cmd_setMaxVelocity(const char* argString);
int cmd_setMaxAcceleration(const char* argString);


    //const int numCliCommands = 27;
	CLI_FUNC_PTR commands_func[]{
    		 &cmd_getMotorInfo,
			 &cmd_findServos,
			 &cmd_changeMode,
			 &cmd_disableControl,
			 &cmd_startControl,
			 &cmd_reset360,
			 &cmd_setZero,
			 &cmd_setZeroReference,
			 &cmd_setCurrentAngle,
			 &cmd_switchDirection,
			 &cmd_setCenterAngle,
			 &cmd_setTorqueConstant,
			 &cmd_setTurns,
			 &cmd_resetSlaves,
			 &cmd_setPID,
			 &cmd_switchUnits,
			 &cmd_togglePrint,
			 &cmd_switchServo,
			 &cmd_changeServoId,
			 &cmd_listServos,
			 &cmd_command,
			 &cmd_restoreFactory,
			 &cmd_syncServoConfigs,
			 &cmd_saveServoConfigs,
			 &cmd_changeDriveMode,
			 &cmd_setMaxVelocity,
			 &cmd_setMaxAcceleration
         };

	const int numCliCommands = sizeof(commands_func)/ sizeof(CLI_FUNC_PTR);

    const char *commands_str[numCliCommands] = {
    		 "printconfig",
			 "findservos",
			 "changemode",
			 "stop",
			 "start",
			 "reset360",
			 "setzero",
			 "setzeroref",
			 "setangle",
			 "switchdir",
			 "setcenterangle",
			 "settorqueconstant",
			 "setturns",
			 "reset",
			 "setpid",
			 "switchunits",
			 "p",
			 "switchservo",
			 "setservoid",
			 "listservos",
			 "c",
			 "restorefactory",
			 "sync",
			 "save",
			 "setdrivemode",
			 "setmaxvel",
			 "setmaxaccel"
         };




void setup() {

	cli_init(&usbSerialOut);
	HAL_Delay(1000);

	// Power up device on port 1
	HAL_GPIO_WritePin(OUT_PORT1_POWER_GPIO_Port, OUT_PORT1_POWER_Pin, GPIO_PIN_SET);

	// Initialization of the Serial Comm for the servos
	HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, RS485_RX_ENABLE);

	//Wait for user to start system.
	myCLI.print("\n\n\r\t-->OmniServo Test Program Started, press button to continue\r\n");
	while(!HAL_GPIO_ReadPin(IN_BUTTON_1_GPIO_Port, IN_BUTTON_1_Pin))
	{

	}

	HAL_UART_Receive_IT(&huart2, (uint8_t*) &uart2_receivedChar, 1);

	myCLI.print("\r\nSearching for servos...\r\n");
	int nbServosFound = omniSequencer.pingAndAddAllServos(1,20);
	sprintf(main_debugSerialOutBuffer, "\r\n-->Number of servos found: %d\r\n", nbServosFound);
	myCLI.print(main_debugSerialOutBuffer);

	if(nbServosFound == 0)
	{
		myCLI.print("\r\n[ERROR]: No servos found !\r\n");
				while(1);
	}


	actuator_Omniservo* currentServo = omniSequencer.setActiveServoByIdndex(0);

	if(currentServo == NULL)
	{
		myCLI.print("\r\n[ERROR]: Servo not added !\r\n");
		while(1);
	}
	else
	{
		// Print list of all detected servos and show their current parameters
		cmd_listServos(NULL);
	}

	omniSequencer.resetSlaves();


}

void loop() {

	// Update for the communication of the servos
	commStatus = omniSequencer.updateServoCom();

	// Dealing with errors with the communication if there are any
	if (commStatus < 0 && PRINT_TIMEOUT) {
		sprintf(main_debugSerialOutBuffer, "TIMEOUT #%d\r\n", -commStatus);
		myCLI.forcePrint(main_debugSerialOutBuffer);
	}

	// Reading the serial buffer to check for any sent commands
	cliAppRun();

	// Printing current servo values if printing is enabled
	if (printValues) {
		if (HAL_GetTick() - previousMillis5hz >= 10) {
			previousMillis5hz = HAL_GetTick();
			if (printValues) {
				actuator_Omniservo* currentServo = omniSequencer.activeServo();
				if (currentServo->getCurrentUnits() == DEGREES) {
					sprintf(main_debugSerialOutBuffer,
						"ID:%d Commande:%.2f ° Angle:%.2f ° Vitesse:%.2f °/s\r\n",
						currentServo->getMotorID(),
						currentServo->getAngleCommand().deg(),
						currentServo->getAngle().deg(),
						currentServo->getCurrentSpeed()
						);
//					sprintf(main_debugSerialOutBuffer,
//							"ID: %d Commande : %.2f ° Angle : %.2f °  Vitesse : %.2f °/s  Courant : %.3f A  Couple : %.2f Nm  Temp : %.2f °C\r\n",
//							currentServo->getMotorID(),
//							currentServo->getAngleCommand().deg(),
//							currentServo->getAngle().deg(),
//							currentServo->getCurrentSpeed(),
//							currentServo->getCurrentCurrent(),
//							currentServo->getCurrentTorque(),
//							currentServo->getCurrentTemp());
				}
				else {
					sprintf(main_debugSerialOutBuffer,
							"ID: %d Angle : %.2f rad  Vitesse : %.2f rad/s  Courant : %.3f A  Couple : %.2f Nm  Temp : %.2f °C\r\n",
							currentServo->getMotorID(),
							currentServo->getAngle().deg(),
							currentServo->getCurrentSpeed(),
							currentServo->getCurrentCurrent(),
							currentServo->getCurrentTorque(),
							currentServo->getCurrentTemp());
				}

				myCLI.forcePrint(main_debugSerialOutBuffer);
			}
		}
	}
}

//TODO change active servo vs all servos
int cmd_changeMode(const char* argString)
{

  	myCLI.print("\n\n\rChanging mode to : ");
	 if(strcmp(argString, "v") == 0)
	 {
		myCLI.print("\n\r-->Velocity mode\r\n");
		omniSequencer.activeServo()->setOperatingMode(operatingMode_t::CONTROL_MODE_VELOCITY_ANGULAR);
	 }
	 else if(strcmp(argString, "p") == 0)
	 {
		myCLI.print( "\n\r-->ABS position mode\r\n");
		omniSequencer.activeServo()->setOperatingMode(operatingMode_t::CONTROL_MODE_POSITION_ANGULAR);
	 }
	 else if(strcmp(argString, "i") == 0)
	 {
		 myCLI.print("\n\r-->Incremental position mode\r\n");
		 omniSequencer.activeServo()->setOperatingMode(operatingMode_t::CONTROL_MODE_POSITION_INCREMENT_ANGULAR);
	 }
	 else
	 {
		 myCLI.print("\n\r-->Argument not recognized\r\n");
		 myCLI.print("\t v: Velocity mode\r\n");
		 myCLI.print("\t p: Absolute position mode\r\n");
		 myCLI.print("\t i: Incremental position mode\r\n");
	 }

	return 0;
}

int cmd_disableControl(const char* argString)
{
	actuator_Omniservo* activeServo = omniSequencer.activeServo();
	if(activeServo == nullptr)
	{
		myCLI.print("\n\r[ERROR]: Active Servo error\r\n");
		return 1;
	}
	activeServo->disableControl();

	myCLI.print("\n\r-->Control disabled\r\n");

	return 0;
}

int cmd_startControl(const char* argString)
{
	actuator_Omniservo* activeServo = omniSequencer.activeServo();
	if(activeServo == nullptr)
	{
		myCLI.print("\n\r[ERROR]: Active Servo error\r\n");
		return 1;
	}
	activeServo->enableControl();

	myCLI.print("\n\r-->Control started\r\n");

	return 0;
}

int cmd_reset360(const char* argString)
{
	omniSequencer.activeServo()->resetTo360();

	myCLI.print("\n\r-->Angle reset 0-360\r\n");

	return 0;
}

int cmd_setZero(const char* argString)
{
	omniSequencer.activeServo()->setZeroPosition();
	myCLI.print("\n\r-->Zero position set to current angle\r\n");

	return 0;
}

int cmd_setZeroReference(const char* argString)
{
	float bufferData = atof(argString);
	omniSequencer.activeServo()->setZeroPosition(bufferData);
	char output[100];

	sprintf(output, "\n\r-->Zero position set to: %.2f\r\n", bufferData);

	myCLI.print(output);

	return 0;
}

int cmd_setCurrentAngle(const char* argString)
{
	float bufferData = atof(argString);
	//omniSequencer.activeServo()->setCurrentAngle(bufferData);
	omniSequencer.activeServo()->setRxAngle(bufferData);
	//TODO messager qui envoit au servo de changer l'angle courant
	char output[100];

	sprintf(output, "\n\r-->Setting current angle to: %.2f\r\n", bufferData);
	myCLI.print(output);

	return 0;
}

int cmd_switchDirection(const char* argString)
{
	omniSequencer.activeServo()->switchRef();
	myCLI.print("\n\r-->Switching the reference direction\r\n");

	return 0;
}

int cmd_setCenterAngle(const char* argString)
{
	float bufferData = atof(argString);
	omniSequencer.activeServo()->setInitialCenterAngle(bufferData);
	char output[100];

	sprintf(output, "\n\r-->Setting center angle to: %.2f\r\n", bufferData);
	myCLI.print(output);

	return 0;
}

int cmd_setTorqueConstant(const char* argString)
{
	float bufferData = atof(argString);
	omniSequencer.activeServo()->sendTorqueConstant(bufferData);
	char output[100];

	sprintf(output, "\n\r-->Sending torque constant: %.2f\r\n", bufferData);
	myCLI.print(output);

	return 0;
}

int cmd_setTurns(const char* argString)
{
	int16_t bufferData = (int16_t)atoi(argString);
	omniSequencer.activeServo()->setTurns(bufferData);
	char output[100];

	sprintf(output, "\n\r-->Setting number of turns to: %d\r\n", (int)bufferData);
	myCLI.print(output);

	return 0;
}

int cmd_resetSlaves(const char* argString)
{
	if(strcmp(argString, "all") == 0)
	{
		omniSequencer.resetSlaves();
		myCLI.print("\n\r-->Resetting all servos\r\n");
	}
	else
	{
//TODO
//		int8_t bufferData = (int8_t)atoi(argString);
//		actuator_Omniservo* servo = omniRCSeq.getServoByID(bufferData);
//		if(servo == NULL)
//		{
//			myCLI.print("\n\r-->No servo with this ID\r\n");
//			return -1;
//		}}
//		servo->resetServo();
	}

	return 0;
}

int cmd_setPID(const char* argString)
{
	char input[50];
	float floatGain;

	float kp, ki, kd, filter;
	int state = 0;

	if(argString[0] == 'v')
	{
		myCLI.print("\n\n\rChanging Velocity PID Values\n");
	}
	else if(argString[0] == 'p')
	{
		myCLI.print("\n\n\rChanging Position PID Values\n");
	}
	else
	{
		myCLI.print("\n\r--> Argument not recognized\r\n");
		myCLI.print("\t p: Position PID\r\n");
		myCLI.print("\t v: Velocity PID\r\n");
		return ERROR_ARGUMENT_ERROR;
	}

	uint32_t startTime = atGetSysTick_ms();
	myCLI.print("\n\r<-- Enter kp:");
	while(atGetSysTick_ms() - startTime < 10000)
	{

		if(serial1.readToChar(input,'\r') )
		{
			if(isdigit(input[0]))
			{
				floatGain = atof(input);
				if(state == 0)
				{
					myCLI.print(input);
					kp = floatGain;
					state = 1;
					startTime = atGetSysTick_ms();
					myCLI.print("\n\r<-- Enter ki:");

				}
				else if(state == 1)
				{
					myCLI.print(input);
					ki = floatGain;
					state = 2;
					startTime = atGetSysTick_ms();
					myCLI.print("\n\r<-- Enter kd:");

				}
//				else if(state == 2)
//				{
//					myCLI.print(input);
//					kd = floatGain;
//					state = 3;
//					startTime = atGetSysTick_ms();
//					myCLI.print("\n\r<-- Enter Filter (0.0-1.0):");
//				}
				else if(state == 2)
				{

					myCLI.print(input);
					kd = floatGain;
					state = 3;
					char output[100];

					if(argString[0] == 'v')
					{
						sprintf(output, "\n\r--> Sending Vel. PID : P:%.2f I:%.2f D:%.2f (filter not used)\r\n",kp,ki,kd);
						omniSequencer.activeServo()->setVelocityKpGain(kp);
						omniSequencer.activeServo()->setVelocityKiGain(ki);
						omniSequencer.activeServo()->setVelocityKdGain(kd);
					}
					else	 // Position
					{
						sprintf(output, "\n\r--> Sending Pos. PID : P:%.2f I:%.2f D:%.2f\r\n",kp,ki,kd);

						omniSequencer.activeServo()->setKpGain(kp);
						omniSequencer.activeServo()->setKiGain(ki);
						omniSequencer.activeServo()->setKdGain(kd);
						//omniSequencer.activeServo()->setPositionFilter(filter);
					}
					myCLI.print(output);


					return 0;

				}
			}
			else
			{
				myCLI.print("\n\n\r--> [ERROR]: Argument not a number, aborting PID settings\r\n\n");
				return ERROR_ARGUMENT_ERROR;
			}

		}
	}

	myCLI.print("\n\n\r--> [ERROR]: Time out, aborting PID settings\r\n\n");
	return ERROR_ARGUMENT_ERROR;


}


int cmd_switchUnits(const char* argString)
{
	if(strcmp(argString, "degrees") == 0 || strcmp(argString, "deg") == 0 || strcmp(argString, "d") == 0)
	{
		omniSequencer.activeServo()->sendWorkingUnits(DEGREES);
		myCLI.print("\n\r-->Switching to degrees.\r\n");
	}
	else if(strcmp(argString, "radians") == 0 || strcmp(argString, "rad") == 0 || strcmp(argString, "r") == 0)
	{
		omniSequencer.activeServo()->sendWorkingUnits(RADIANS);
		myCLI.print("\n\r-->Switching to radians.\r\n");
	}
	else
	{
		myCLI.print("\n\r-->Missing parameter\r\n");
		myCLI.print("\n\r\t [Degrees]: use degrees, deg or d");
		myCLI.print("\n\r\t [Radians]: use radians, rad or r\n\r");
	}

	return 0;
}


int cmd_togglePrint(const char* argString)
{
	if(strcmp(argString, "on") == 0 )
	{
		printValues = true;
		myCLI.print("\n\r-->Print Data: On\r\n");
	}
	else if(strcmp(argString, "off") == 0 )
	{
		printValues = false;
		myCLI.print("\n\r-->Print Data: Off\r\n");
	}
	else
	{
		if (printValues)
		{
			printValues = false;
			myCLI.print("\n\r-->Print Data: Off\r\n");
		}
		else {
			printValues = true;
			myCLI.print("\n\r-->Print Data: On\r\n");
		}
	}

	return 0;
}

int cmd_switchServo(const char* argString)
{
	char output[100];

	if(isdigit(argString[0]))
	{
		int servoId = atoi(argString);
		sprintf(output, "\n\r<-- Trying switch to ID#%d", servoId);
		myCLI.print(output);
		omniSequencer.setActiveServo(servoId);
		sprintf(output, "\n\r-->Active Servo ID#%d", omniSequencer.activeServo()->getMotorID());
		myCLI.print(output);
	}
	else if(argString[0] == '\0')
	{
		sprintf(output, "\n\r<-- Trying switch to next available servo");
		myCLI.print(output);
		omniSequencer.setNextActiveServo();
		sprintf(output, "\n\r-->Active Servo ID#%d", omniSequencer.activeServo()->getMotorID());
		myCLI.print(output);
	}
	else
	{
		myCLI.print("\n\n\r--> [ERROR]: Argument not a number");
		myCLI.print("\n\n\r\tOption 1: Enter servo ID");
		myCLI.print("\n\r\tOption 2: Enter nothing to switch to the next available servo\r\n\n");
		return ERROR_ARGUMENT_ERROR;
	}

	return 0;
}

int cmd_changeServoId(const char* argString)
{
	char output[100];

	if(isdigit(argString[0]))
	{
		int servoId = atoi(argString);
		sprintf(output, "\n\r<-- Trying switch to ID of motor #%d to #%d", omniSequencer.activeServo()->getMotorID(), servoId);
		myCLI.print(output);
		omniSequencer.activeServo()->changeIDrequest(servoId);
	}
	else
	{
		myCLI.print("\n\n\r--> [ERROR]: Argument not a number");
		myCLI.print("\n\n\r\tOption 1: Enter desired servo ID\r\n\n");
		return ERROR_ARGUMENT_ERROR;
	}

	return 0;
}


void printServoConfig(actuator_Omniservo* servo)
{
	ServoConfigInfo configInfo = servo->getConfig();
	char output[130];

	if(configInfo.workingUnits == DEGREES)
	{
		sprintf(output, "\n\r\tWorking Units: Degrees");
	}
	else
	{
		sprintf(output, "\n\r\tWorking Units: Radians");
	}
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tOrigin reference: %ld",configInfo.originRef);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tDirection: %ld",configInfo.refDirection);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tCenter Angle: %.2f deg.",configInfo.centerReadAngle);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tTorque Constant: %.2f mNm/A",configInfo.torqueConstant);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tPosition PID: Kp: %.2f \tKi: %.2f \tKd: %.2f \tFilter(tau): %.4f",configInfo.kpp, configInfo.kip, configInfo.kdp, configInfo.positionFilter);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tVelocity PID: Kp: %.2f \tKi: %.2f \tKd: %.2f",configInfo.kpv, configInfo.kiv, configInfo.kdv);
	myCLI.print(output);
	delay(1);

	if(configInfo.driveMode == 0)
	{
		sprintf(output, "\n\r\tDrive Mode: Phase mode");
	}
	else
	{
		sprintf(output, "\n\r\tDrive Mode: PWM mode");
	}
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tMax velocity: %.2f",configInfo.maxVelocity);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tMax acceleration: %.2f",configInfo.maxAcceleration);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tPosition Deadband: %.2f",configInfo.positionDeadband);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tMax Position: %.2f",configInfo.maxPosition);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tMin Position: %.2f",configInfo.minPosition);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tPosition Stall Threshold: %.2f",configInfo.positionStallThreshold);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tPosition Stall Timeout: %.2f",configInfo.positionStallTimeout);
	myCLI.print(output);
	delay(1);

	sprintf(output, "\n\r\tPosition Limits Enabled: %s",configInfo.positionLimitsEnabled ? "True" : "False");
	myCLI.print(output);
	delay(1);





}



int cmd_listServos(const char* argString)
{
	char output[130];

	myCLI.print("\n\n\r ---- List of connected servos ----\r\n");
	delay(50);

	for(int i=0; i <omniSequencer.getNbServos(); i++)
	{
		actuator_Omniservo* servo = omniSequencer.getServoByIndex(i);
		sprintf(output, "\n\r\t Servo [%d]: ID[%d]", i,servo->getMotorID() );
		myCLI.print(output);
		delay(10);
		if(servo == omniSequencer.activeServo())
		{
			sprintf(output," (Active)");
			myCLI.print(output);
			delay(50);
		}

		printServoConfig(servo);

	}
	myCLI.print("\n\n\r ---- End of List ----\r\n");
	delay(50);

	return 0;

}


int cmd_command(const char* argString)
{
	float commandValue = 0;
	angleRad_rct command;
	char output[130];

	if(parseFloat(argString, commandValue)){

		sprintf(output, "\n\rSending %f to ID[%d]", commandValue, omniSequencer.activeServo()->getMotorID());
		myCLI.print(output);

		actuatorError_rct errorCode = omniSequencer.activeServo()->sendCommand(commandValue);
		if(errorCode != ERROR_ROBOCORE_ACTUATOR_NONE)
		{
			sprintf(output, "\n\r--> [ERROR]: Sending command failed with error code %d", (int)errorCode);

			myCLI.print(output);
			return errorCode;
		}
	}
	else
	{
		myCLI.print("\n\n\r--> [ERROR]: Argument not a number");
		return ERROR_ARGUMENT_ERROR;
	}


	return 0;
}

int cmd_restoreFactory(const char* argString)
{
	omniSequencer.activeServo()->restoreFactorySettingsRequest();
	myCLI.print("\n\r-->Sending restore factory request, make sure to synchronise configs after\r\n");

	return 0;
}

int cmd_syncServoConfigs(const char* argString)
{
	omniSequencer.syncAllServoConfig();
	myCLI.print("\n\r-->Syncing all servo configurations\r\n");

	return 0;
}

int cmd_saveServoConfigs(const char* argString)
{
	omniSequencer.activeServo()->saveConfigToFlashRequest();
	myCLI.print("\n\r-->Sending Configuration Save request");
	return 0;
}


int cmd_changeDriveMode(const char* argString)
{

	if(strcmp(argString, "pwm") == 0 )
	{
		myCLI.print("\n\r-->Switching to PWM Drive mode, reboot recommended");
		omniSequencer.activeServo()->changeDriveMode(DriveMode_PWM);
	}
	else if(strcmp(argString, "phase") == 0 )
	{
		myCLI.print("\n\r-->Switching to Phase + Enable drive mode");
		omniSequencer.activeServo()->changeDriveMode(DriveMode_Phase);
	}
	else
	{
		myCLI.print("\n\rUnknown parameter, supported parameters: \n\r\tpwm: Switch to PWM Drive mode\n\r\tphase: Switch to Phase + Enable drive mode");

	}
	return 0;
}

int cmd_setMaxVelocity(const char* argString)
{
	float commandValue = 0;
	char output[130];

	if(parseFloat(argString, commandValue)){

			sprintf(output, "\n\rSetting Max velocity to  %f for servo ID[%d]", commandValue, omniSequencer.activeServo()->getMotorID());
			myCLI.print(output);

			actuatorError_rct errorCode = omniSequencer.activeServo()->setVelocityLimit_HW(angleRad_rct::fromDegrees(commandValue));
			if(errorCode != ERROR_ROBOCORE_ACTUATOR_NONE)
			{
				sprintf(output, "\n\r--> [ERROR]: Sending command failed with error code %d", (int)errorCode);

				myCLI.print(output);
				return errorCode;
			}
		}
		else
		{
			myCLI.print("\n\n\r--> [ERROR]: Argument not a number");
			return ERROR_ARGUMENT_ERROR;
		}

		return 0;
}

int cmd_setMaxAcceleration(const char* argString)
{
	float commandValue = 0;
	char output[130];

		if(parseFloat(argString, commandValue)){

			actuatorError_rct errorCode = omniSequencer.activeServo()->setAccelerationLimit_HW(angleRad_rct::fromDegrees(commandValue));

			if(errorCode != ERROR_ROBOCORE_ACTUATOR_NONE)
			{
				sprintf(output, "\n\r--> [ERROR]: Sending command failed with error code %d", (int)errorCode);

				myCLI.print(output);
				return errorCode;
			}
		}
		else
		{
			myCLI.print("\n\n\r--> [ERROR]: Argument not a number");
			return ERROR_ARGUMENT_ERROR;
		}

		return 0;
}



//				sprintf(debugSerialOutBuffer, "Sending number of turns : %d\r\n", (int16_t)bufferData);
//				currentRCServo->setTurns((int16_t)bufferData);

//void serialReading() {
//	if (inputBuffer[bufferPointer-1] == '\n' || inputBuffer[bufferPointer-1] == '\r' ) {
//		inputBuffer[bufferPointer++] = '\0';
//		bufferData = atof(inputBuffer);
//
//		// If command is not a number, look for characters
//		if (bufferData == 0 && !isdigit(inputBuffer[0])) {
//			switch (inputBuffer[0])
//			{

//		}
//		// Else, read the numerical command
//		else {

//			else { // Else it's a motor movement command
//				sprintf(debugSerialOutBuffer, "Command : %.2f\r\n",bufferData);
//				//currentRCServo->sendCommand(bufferData);
//
//				if(currentRCServo->getControlMode() == operatingMode_t::CONTROL_MODE_POSITION_ANGULAR)
//				{
//					currentRCServo->setAngleCommand(bufferData);
//				}
//				else if(currentRCServo->getControlMode() == operatingMode_t::CONTROL_MODE_VELOCITY_ANGULAR)
//				{
//					currentRCServo->setAngularVelocityCommand(bufferData);
//				}
////				else if(currentRCServo->getOperatingMode() == operatingMode_t::CONTROL_MODE_POSITION_INCREMENT_ANGULAR)
////				{
////					currentRCServo->setAngleCommand(bufferData);
////				}
//				else
//				{
//					sprintf(debugSerialOutBuffer, "Error: Control mode not set or supported\r\n" );
//				}
//			}
//		}
//		// Empty the buffer and return the pointer to 0
//		for (int i=0; i<bufferPointer; i++) {
//			inputBuffer[i] = 0;
//		}
//		bufferPointer = 0;
//		// Print the command
//		//HAL_UART_Transmit(&hlpuart1, (uint8_t*)serialOutBuffer, strlen(serialOutBuffer), 1000);
//		usbSerialOut(debugSerialOutBuffer);
//	}
//}


/**
 * Cette fonction est un callback qui sera appelé par le gestionnaire de omniServo lorsque
 * le data a été copié dans le buffer et est prêt à être envoyer.
 */
uint8_t servoUart2TxCallback(int dataLen)
{
	if(dataLen > 0 && (uint32_t)dataLen <= sizeof(m_uart2TxBuffer))
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
		//omniSequencer.rxUartInterruptHandler((uint8_t*)&uart2_receivedChar, 1);
		omniSequencer.rxUartInterruptHandler((uint8_t*)&uart2_receivedChar, 1);
		uart2Rxstatus = HAL_UART_Receive_IT(&huart2, (uint8_t*) &uart2_receivedChar, 1);
	}

	if(huart->Instance == hlpuart1.Instance) {
		serial1.rxCompleteCallback(huart->RxXferSize);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == LPUART1)
	{
		serial1.txCompleteCallback();
	}

	if(huart->Instance == huart2.Instance) {
		//OmniServo::txUartInterruptHandler(huart);
		HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, RS485_RX_ENABLE);
	}
}


int cmd_getMotorInfo(const char* argString)
{
	actuator_Omniservo* servo;
	if(isdigit(argString[0]))
	{
		servo = omniSequencer.getServo(atoi(argString));
		if(servo == NULL)
		{
			myCLI.print("\n\n\r--> [ERROR]: No servo with this ID");
			return ERROR_ARGUMENT_ERROR;
		}
	}
	else if(argString[0] == '\0')
	{
		servo = omniSequencer.activeServo();
	}
	else
	{
		myCLI.print("\n\n\r--> [ERROR]: Argument not a number");
		myCLI.print("\n\n\r\tOption 1: Enter servo ID");
		return ERROR_ARGUMENT_ERROR;
	}


		printServoConfig(servo);
	return ERROR_NO_ERROR;
}

//TODO
int cmd_findServos(const char* argString)
{
	myCLI.forcePrint("--> NOT IMPLEMENTED YET\r\n");
	//myCLI.forcePrint("--> findServos Success\r\n");

	return 0;
}
