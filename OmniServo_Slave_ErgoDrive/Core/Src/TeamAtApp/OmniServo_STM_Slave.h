/**
 * \file OmniServo.h
 * \brief OmniServo class interface
 * \author Nicolas Breton (nibre28)
 * \version 2.0
 * \date 13/06/2024
 */

#ifndef _OMNISERVO_STM_Slave_h_
#define _OMNISERVO_STM_Slave_h_

#include <stdio.h>
#include <string.h>
#include <usart.h>
#include <adc.h>
#include "TeamAT_AS5600_STM.h"
#include "backupData_OmniServo.h"
#include "AtPositionController.h"


#define DRIVE_MODE_PHASE_ENABLE 0
#define DRIVE_MODE_PWM			1
#define DEFAULT_DRIVE_MODE		DRIVE_MODE_PHASE_ENABLE

#define DEFAULT_MAX_VELOCITY				360.0	//deg/s
#define DEFAULT_MAX_ACCELERATION			1000.0	//deg/s²
#define DEFAULT_POSITION_DEADBAND					0.1		//deg
#define DEFAULT_MAX_POSITION				36000.0	//deg
#define DEFAULT_MIN_POSITION				-36000.0//deg
#define DEFAULT_POSITION_STALL_THRESHOLD 	90.0 //deg
#define DEFAULT_POSITION_STALL_TIMEOUT		5.0 // seconds
#define DEFAULT_POSITION_LIMITS_ENABLED 	false



#define IDLE_MODE_BRAKE 		0
#define IDLE_MODE_COAST			1
#define DEFAULT_IDLE_MODE		IDLE_MODE_COAST

#define DEFAULT_CONTROL_MODE ABS_POSITION
#define DEFAULT_TORQUE_CONSTANT 11.44 // mNm/A (Langyi 2232R-04A)

#define EN_IN1_PWM  TIM2->CCR4
#define PH_IN2_PWM  TIM4->CCR4


#define OMNISERVO_DEFAULT_MOTOR_ID  		1
#define OMNISERVO_DEFAULT_KPP       		15.0
#define OMNISERVO_DEFAULT_KDP       		5.0
#define OMNISERVO_DEFAULT_KIP       		1.0
//#define OMNISERVO_DEFAULT_POSITION_FILTER	0.005
#define OMNISERVO_DEFAULT_TAUP      		0.005
#define OMNISERVO_DEFAULT_KPV       		0.4
#define OMNISERVO_DEFAULT_KDV       		0.0
#define OMNISERVO_DEFAULT_KIV       		12.0
#define OMNISERVO_DEFAULT_VELOCITY_FILTER	0.0
#define OMNISERVO_DEFAULT_TAUV      		0.1
#define OMNISERVO_DEFAULT_ORIGINREF 		0
#define OMNISERVO_DEFAULT_REFDIR    		1
#define OMNISERVO_DEFAULT_CENTERREADANGLE 0.0
#define OMNISERVO_DEFAULT_TORQUE_CONSTANT  DEFAULT_TORQUE_CONSTANT
#define OMNISERVO_DEFAULT_WORKING_UNITS DEGREES

#define DEFAULT_POSITION_DEADBAND 0.16

/**
 * @brief Possible usage modes of the servo
 * @param OFF Servo is turned off without brakes
 * @param ABS_POSITION Servo is in absolute position mode
 * @param INC_POSITION Servo is in incremental position mode
 * @param SPEED Servo is in speed mode
 */
typedef enum {
    NOT_SET = 0,
    ABS_POSITION = 1,
    INC_POSITION = 2,
    SPEED = 3,
	TORQUE = 4
} ServoModes;

/**
 * @brief Possible working units of the servo
 * @param DEGREES Units are deg & deg/s
 * @param RADIANS Units are rad & rad/s
 */
typedef enum {
    DEGREES = 0, //deg & deg/s
    RADIANS = 1  //rad & rad/s
} WorkingUnits;

class OmniServo
{
public:
    OmniServo(float samplingTime)
    	: m_pController(samplingTime)
    {
    	m_sampleTime = samplingTime;
    	loadDefaultConfig();
    }

    void init();
    
	#define KEEP_ID true
	#define DONT_KEEP_ID false


    void saveConfigToFlash();
	void restoreFactorySettings(bool keepID);
	ServoConfigInfo getConfigInfo();


    void setPositionPID(const float& p_kp, const float& p_kd, const float& p_ki, const float& p_filter);

    void setVelocityPID(const float& p_kp, const float& p_kd, const float& p_ki);

	void changeMotorID(const uint8_t& p_motorID);
    void changeMode(ServoModes p_mode);
    void changeControlEnable(bool controlEnable);
    bool getControlEnable();
    void changeWorkingUnits(WorkingUnits p_units);

    void sendComand(const float& p_command);
    void updateCommand();
    void setMotorPwm(float motorPwmPercent); // TODO sortir de Omniservo pour devenir plsu indépendant du HW

    void resetTo360();
    void setOrigin();
    void setOrigin(const float& p_angle);
    void setCurrentAngle(const float& p_angle);
    void switchRef();
    void setTurns(const int16_t& p_turns);
    void setCenterReadAngle(const float& p_angle);
    void centerInitialRead();
    void setTorqueConstant(const float& p_torqueConstant);

    float getAdjustedAngle();
    float updateMultiturnAngle();
    void computeSpeed();
    void updateCurrentAndTemp();

	uint8_t reqMotorID();
    float reqCurrentAngle();
    float reqTargetAngle();
    float reqCurrentSpeed();
	float reqTargetSpeed();
    float reqDriveCommand();
    ServoModes reqCurrentMode();
    WorkingUnits reqWorkingUnits();
    float reqCurrentCurrent();
    float reqCurrentTemp();
    float reqTorqueConstant();

    void setDriveMode(uint8_t driveMode);
    void reset();

    void setMaxVelocity(float velocityMax);
    void setMaxAcceleration(float accelMax);

    teamAT_AS5600 m_encoder = teamAT_AS5600(&hi2c1);

    bool m_initDone = false;

protected:

    void loadDefaultConfig();
    void setConfigData();
    void applyConfigData();
    void readConfigData(uint32_t *Data, uint16_t numberofdoublewords);
    uint32_t writeFlashConfig(uint32_t *Data, uint16_t numberofdoublewords);
    uint32_t GetPage(uint32_t Addr);

    float rad2deg(float p_value);
    float deg2rad(float p_value);
    int angle2raw(float p_angle);
    float raw2angle(int p_raw);
    double dmap(double p_value, double from_low, double from_high, double to_low, double to_high);
    float wrapTo360(float p_angle);
    float lookUpTemp(float p_resitance);



private:



    uint8_t m_motorID;

    float m_sampleTime;

    bool m_controlEnabled = false;
    ServoModes m_currentMode;
    WorkingUnits m_currentUnits;

    float m_kpv;
    float m_kdv;
    float m_kiv;
    float m_tauv;
    float m_PWMCountKiv;

    /*PWM conversion factor from -100% to 100% to PWM counts*/
    float m_PWMConvert = (2000 / 100.0); //TODO configurable

    int32_t m_originRef;
    int8_t m_refDirection;
    int32_t m_nbCurrentTurns;
    float m_centerReadAngle;
    float m_torqueConstant;

    float m_currentAngle;
    float m_targetAngle;
    float m_currentSpeed;
    float m_targetSpeed;

    float m_driveCommand;

    float m_previousReadAngle;
    float m_previousSpeed;
    float m_previousAngle10Hz;

    float m_previousError;
    float m_errorSum;


    uint16_t m_adcBuffer[2];
    float m_currentCurrent;
    float m_currentTemp;

    PositionController m_pController;

    backupStruct_t m_config;

    uint8_t m_idleMode = DEFAULT_IDLE_MODE;
    uint8_t m_driveMode = DEFAULT_DRIVE_MODE;


};

#endif
