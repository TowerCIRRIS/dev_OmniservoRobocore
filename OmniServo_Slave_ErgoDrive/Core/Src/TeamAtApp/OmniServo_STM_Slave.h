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

#define DEFAULT_CONTROL_MODE ABS_POSITION
#define DEFAULT_TORQUE_CONSTANT 11.44 // mNm/A (Langyi 2232R-04A)

//    m_kpp = 100; // 2.5; // 40 400
//    m_kdp = 1.6; // 2.5; // 1.8 15
//    m_kip = 8; // 2.5; // 8 0
//    m_taup = 1/100.0;
//    m_kpv = 0.05*8; // 2.5;
//    m_kdv = 0*8; // 2.5;
//    m_kiv = 1.5*8; // 2.5;
//    m_tauv = 1.0/10.0;
//    m_originRef = 0;
//    m_refDirection = 1;
//    m_centerReadAngle = 0;
//    m_torqueConstant = DEFAULT_TORQUE_CONSTANT;
//    m_currentUnits = DEGREES;
#define OMNISERVO_DEFAULT_MOTOR_ID  1
#define OMNISERVO_DEFAULT_KPP       100.0
#define OMNISERVO_DEFAULT_KDP       1.6
#define OMNISERVO_DEFAULT_KIP       8.0
#define OMNISERVO_DEFAULT_TAUP      0.01
#define OMNISERVO_DEFAULT_KPV       0.4
#define OMNISERVO_DEFAULT_KDV       0.0
#define OMNISERVO_DEFAULT_KIV       12.0
#define OMNISERVO_DEFAULT_TAUV      0.1
#define OMNISERVO_DEFAULT_ORIGINREF 0
#define OMNISERVO_DEFAULT_REFDIR    1
#define OMNISERVO_DEFAULT_CENTERREADANGLE 0.0
#define OMNISERVO_DEFAULT_TORQUE_CONSTANT  DEFAULT_TORQUE_CONSTANT
#define OMNISERVO_DEFAULT_WORKING_UNITS DEGREES

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
    OmniServo();
    void init();
    
	#define KEEP_ID true
	#define DONT_KEEP_ID false
	void restoreFactorySettings(bool keepID);
	ServoConfigInfo getConfigInfo();

    void asgPIDvalues(const float& p_kp, const float& p_kd,
		const float& p_ki);
    void asgPIDpositionValues(const float& p_kp, const float& p_kd, 
        const float& p_ki);
    void asgPIDspeedValues(const float& p_kp, const float& p_kd, 
        const float& p_ki);

	void changeMotorID(const uint8_t& p_motorID);
    void changeMode(ServoModes p_mode);
    void changeControlEnable(bool controlEnable);
    void changeWorkingUnits(WorkingUnits p_units);

    void sendComand(const float& p_command);
    void updateCommand();

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

    void reset();

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

    bool m_initDone = false;
    uint8_t m_motorID;

    bool m_controlEnabled = false;
    ServoModes m_currentMode;
    WorkingUnits m_currentUnits;

    float m_kpp;
    float m_kdp;
    float m_kip;
    float m_taup;

    float m_kpv;
    float m_kdv;
    float m_kiv;
    float m_tauv;

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

    float m_previousAngle;
    float m_previousSpeed;
    float m_previousAngle10Hz;

    float m_previousError;
    float m_errorSum;

    teamAT_AS5600 m_encoder = teamAT_AS5600(&hi2c1);
    uint16_t m_adcBuffer[2];
    float m_currentCurrent;
    float m_currentTemp;

    backupStruct_t m_config;



};

#endif
