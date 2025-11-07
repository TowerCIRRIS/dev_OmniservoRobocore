/**
 * \file OmniServo.cpp
 * \brief OmniServo class implementation
 * \author Nicolas Breton (nibre28)
 * \version 2.1
 * \date 25/06/2024
 */

#include "OmniServo_STM_Slave.h"
#include "math.h"

#define FORCE_MAGNET_DETECTION true

#define PWM_COUNT 2000
#define PWM_LIMIT	(PWM_COUNT-1)

#define PI 3.1416
//#define VrefCurrentSense 2.7107 // Volts
#define RrefCurrentSense 1000 // Ohms
#define currentScaling 0.000450
#define VREF 3.3 //Volts
#define nbTempValues 43
double Aref = 0.0019;
double Bref = 0.000118;
double Cref = 0.0000004776;


float Rratio[nbTempValues] = {3.265, 2.539, 1.99, 1.571, 1.249, 1, 0.8057, 0.6531, 0.5327, 0.4369, 0.3603, 0.2986, 0.2488, 0.2083, 0.1752, 0.1481, 0.1258, 0.1072, 0.09177, 0.07885, 0.068, 0.05886, 0.05112, 0.04454, 0.03893, 0.03417, 0.03009, 0.02654, 0.02348, 0.02083, 0.01853, 0.01653};
float temps[nbTempValues] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155};

//bool adcConvDone = false; // Investigate ADC Interrupt Error

// Removes the ki portion of the PID when the error is too great
#define ERROR_SUM_DELTA 90.0 //deg

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0)) // Function used to retreive number sign

/**
 * @brief Default constructor of the OmniServo object
 */
OmniServo::OmniServo() {

	loadDefaultConfig();
}

/**
 * @brief Method to load the default configuration values into the object
 * @note This will not write the values in the flash memory
 */
void OmniServo::loadDefaultConfig()
{
	// Asign values
	m_motorID = 		OMNISERVO_DEFAULT_MOTOR_ID;
//	m_kpp = 			OMNISERVO_DEFAULT_KPP;
//	m_kdp = 			OMNISERVO_DEFAULT_KDP;
//	m_kip = 			OMNISERVO_DEFAULT_KIP;
	m_pController.setGains(OMNISERVO_DEFAULT_KPP, OMNISERVO_DEFAULT_KIP, OMNISERVO_DEFAULT_KDP);


//	m_PWMCountKip = 	((float)PWM_COUNT)/m_kip;

   // m_taup = 			OMNISERVO_DEFAULT_TAUP;
    m_kpv = 			OMNISERVO_DEFAULT_KPV;
    m_kdv = 			OMNISERVO_DEFAULT_KDV;
    m_kiv = 			OMNISERVO_DEFAULT_KIV;
    m_PWMCountKiv = 	((float)PWM_COUNT)/m_kiv;
    m_tauv = 			OMNISERVO_DEFAULT_TAUV;
    m_originRef = 		OMNISERVO_DEFAULT_ORIGINREF;
    m_refDirection = 	OMNISERVO_DEFAULT_REFDIR;
    m_centerReadAngle = OMNISERVO_DEFAULT_CENTERREADANGLE;
    m_torqueConstant = 	OMNISERVO_DEFAULT_TORQUE_CONSTANT;
    m_currentUnits = 	OMNISERVO_DEFAULT_WORKING_UNITS;


	m_driveMode = DEFAULT_DRIVE_MODE;
	m_pController.m_maxVelocity	= DEFAULT_MAX_VELOCITY;
	m_pController.m_maxAcceleration	= DEFAULT_MAX_ACCELERATION;
	m_pController.setDeadband(DEFAULT_DEADBAND);
	m_pController.m_maxPosition = DEFAULT_MAX_POSITION;
	m_pController.m_minPosition = DEFAULT_MIN_POSITION;
	m_pController.m_stallThreshold = DEFAULT_POSITION_STALL_THRESHOLD;
	m_pController.m_stallTimeout = DEFAULT_POSITION_STALL_TIMEOUT;
	m_pController.m_limitsEnabled = DEFAULT_POSITION_LIMITS_ENABLED;

}
/**
 * @brief Basic servo initialize method
 */
void OmniServo::init() {

	if(m_driveMode == DRIVE_MODE_PHASE_ENABLE)
	{
		HAL_GPIO_WritePin(MTR_PMODE_GPIO_Port, MTR_PMODE_Pin, GPIO_PIN_RESET); // PH/en Control Mode
	}
	else
	{
		HAL_GPIO_WritePin(MTR_PMODE_GPIO_Port, MTR_PMODE_Pin, GPIO_PIN_SET); // PWM Mode
	}

	changeControlEnable(false);
	changeMode(DEFAULT_CONTROL_MODE);

    m_config.startCode = 0;
    m_config.stopCode = 0;

    readConfigData((uint32_t*)&m_config, sizeof(m_config)/4);
	if (m_config.startCode == 0xDEADBEEF && m_config.stopCode == 0xDEADBEEF
			&& m_config.backupStructVersion == BACKUP_STRUCT_VERSION) {
		applyConfigData();
	}
	else {

		restoreFactorySettings(DONT_KEEP_ID);
	}

    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)m_adcBuffer, 2);
    // Check the presence of the magnet for the encoder
    if(m_encoder.detectMagnet() == 0  && FORCE_MAGNET_DETECTION) {
        while(true) {
            if(m_encoder.detectMagnet() == 1 ) {
                break;
            }
            HAL_Delay(1000);
        }
    }
    centerInitialRead();

    m_initDone = true;
}


void OmniServo::saveConfigToFlash()
{
	setConfigData();
	m_config.startCode = 0xDEADBEEF;
	m_config.stopCode = 0xDEADBEEF;
	m_config.backupStructVersion = BACKUP_STRUCT_VERSION;
	writeFlashConfig((uint32_t*)&m_config, sizeof(m_config)/4);
}
/**
 * @brief Command to restore the factory settings of the servo
 * @param[in] keepID If true, the motor ID will be kept
 * @note This will erase all the settings and restore the default values then
 * save them in the flash memory
 */
void OmniServo::restoreFactorySettings(bool keepID)
{
	uint8_t currentID = m_motorID;
	loadDefaultConfig();

	if (keepID) {
		m_motorID = currentID;
	}

	saveConfigToFlash();
}


ServoConfigInfo OmniServo::getConfigInfo()
{
	return m_config.data;
}

/**
 * @brief Command to change the PID values of the position or speed mode. Any position mode
 *        will change the PID values for all position modes.
 * @param[in] p_mode Targeted mode (Position or Speed)
 * @param[in] p_kp Proportional gain
 * @param[in] p_kd Derivative gain
 * @param[in] p_ki Integral gain
 */
void OmniServo::asgPIDvalues(const float& p_kp, const float& p_kd,
	const float& p_ki) {
	if (m_currentMode == SPEED) {asgPIDspeedValues(p_kp, p_kd, p_ki);}
	else {asgPIDpositionValues(p_kp, p_kd, p_ki);}
}

/**
 * @brief Command to change the PID values of the position modes
 * @param[in] p_mode Targeted mode (Position or Speed)
 * @param[in] p_kp Proportional gain
 * @param[in] p_kd Derivative gain
 * @param[in] p_ki Integral gain
 */
void OmniServo::asgPIDpositionValues(const float& p_kp, const float& p_kd, 
    const float& p_ki) {

    m_pController.setGains(p_kp, p_ki, p_kd);

}

/**
 * @brief Command to change the PID values of the speed mode
 * @param[in] p_mode Targeted mode (Position or Speed)
 * @param[in] p_kp Proportional gain
 * @param[in] p_kd Derivative gain
 * @param[in] p_ki Integral gain
 */
void OmniServo::asgPIDspeedValues(const float& p_kp, const float& p_kd, 
    const float& p_ki) {
    m_kpv = p_kp;
    m_kdv = p_kd;
    m_kiv = p_ki;
    m_PWMCountKiv = ((float)PWM_COUNT)/m_kiv;

}

/**
 * @brief Command to change the motor ID
 * @param[in] p_motorID Motor ID
 */
void OmniServo::changeMotorID(const uint8_t& p_motorID) {
	m_motorID = p_motorID;
    setConfigData();
}

/**
 * @brief Command to change the working mode
 * @param[in] p_mode Servo mode
 * @note Reference ServoModes for the list of possible modes
 */
void OmniServo::changeMode(ServoModes p_mode) {


    switch (p_mode)
    {
//    case ServoModes::DISABLED:
//        TIM2->CCR4 = 0;
//        m_currentMode = DISABLED;
//        HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_RESET);
//        break;
    case ServoModes::ABS_POSITION:
        if (m_currentMode != ABS_POSITION) {
            if (m_currentMode != INC_POSITION) {
                m_targetAngle = m_currentAngle;
            }
            changeControlEnable(false);
            m_errorSum = 0;
            m_currentMode = ABS_POSITION;
            //Géré par changeControlEnable()  HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_SET);

            updateCommand();
		}
        break;
    case ServoModes::INC_POSITION:
        if (m_currentMode != INC_POSITION) {
            if (m_currentMode != ABS_POSITION) {
                m_targetAngle = m_currentAngle;
            }
            changeControlEnable(false);
            m_errorSum = 0;
            m_currentMode = INC_POSITION;
            //Géré par changeControlEnable()  HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_SET);
            updateCommand();
        }
        break;
    case ServoModes::SPEED:
        if (m_currentMode != SPEED) {
            m_currentMode = SPEED;
            changeControlEnable(false);
            m_targetSpeed = 0;
            m_errorSum = 0;
           //Géré par changeControlEnable()  HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_SET);
            updateCommand();
        }
        break;
    case ServoModes::TORQUE:
		// Not implemented yet
    case ServoModes::NOT_SET:
	default: //Others not supported yet, disable control
		changeControlEnable(false);
//		 TIM2->CCR4 = 0;
//		m_currentMode = DISABLED;
//		HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_RESET);
		break;
    }
}

/**
 * @brief Command to enable or disable the motor control
 * @param[in] controlEnable True to enable the control, false to disable it
 *
 */
void OmniServo::changeControlEnable(bool controlEnable) {

	if(controlEnable && !m_controlEnabled)
	{
		m_errorSum = 0;
		m_targetAngle = m_currentAngle;
		m_targetSpeed = 0;
		m_controlEnabled = true;

		HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_SET);


	}
	else if(!controlEnable && m_controlEnabled)
	{
		if(m_driveMode == DRIVE_MODE_PHASE_ENABLE)
		{
			EN_IN1_PWM = 0;	// EN = 0
			PH_IN2_PWM = 0; // don'T care
			if(m_idleMode == IDLE_MODE_BRAKE)
			{
				HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_SET);
			}
			else //IDLE_MODE_COAST
			{
				HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_RESET);
			}
		}
		else // DRIVE_MODE_PWM
		{
			if(m_idleMode == IDLE_MODE_BRAKE)
			{
				EN_IN1_PWM = PWM_COUNT;
				PH_IN2_PWM = PWM_COUNT;
			}
			else	//IDLE_MODE_COAST
			{
				EN_IN1_PWM = 0;
				PH_IN2_PWM = 0;
				//TODO tester avec / sans sleep
				HAL_GPIO_WritePin(MTR_nSLEEP_GPIO_Port, MTR_nSLEEP_Pin, GPIO_PIN_RESET);
			}
		}

		m_controlEnabled = false;
	}
}

/**
 * @brief Accessor for the control enable state
 * @return True if the control is enabled, false otherwise
 */
bool OmniServo::getControlEnable()
{
	return m_controlEnabled;
}

/**
 * @brief Command to change the working units
 * @note Reference WorkingUnits for the list of possible units
 * @warning Changes the returned units as well as the expected sent units
 * @param[in] p_units Working units
 */
void OmniServo::changeWorkingUnits(WorkingUnits p_units) {
	m_currentUnits = p_units;

}

/**
 * @brief Command to send a movement command.
 * @note The effect will change depending on the current servo mode. \note
 *         \p Disabled : Will do nothing \note
 *         \p Absolute_Position : Will change the target angle to the one given \note
 *         \p Incremental_Position : Will sum the given angle to the target angle \note
 *         \p Speed : Will change the target speed to the one given
 * @param[in] p_command Angle or speed to send
 */
void OmniServo::sendComand(const float& p_command) {
    float command = p_command * m_refDirection;

    if (m_currentUnits == RADIANS)
    {
    	command = rad2deg(command);
    }
    switch (m_currentMode)
    {
    case NOT_SET:
    	break;
    case ABS_POSITION:
        m_targetAngle = command;
        break;
    case INC_POSITION:
        m_targetAngle += command;
        break;
    case SPEED:
        m_targetSpeed = command;
        break;
    case TORQUE:
   		// Not implemented yet
	default:
		 break;
    }
}

/**
 * @brief Updates the motor command
 * @warning This method must be called at 100Hz for position modes and 10Hz for speed mode
 */
void OmniServo::updateCommand() {

    float readAngle = getAdjustedAngle();
    if (m_previousReadAngle >= 270 && readAngle <= 90) {
    	m_nbCurrentTurns += 1;
    }
    else if (m_previousReadAngle <= 90 && readAngle >= 270) {
    	m_nbCurrentTurns += -1;
    }
    m_currentAngle = readAngle + 360.0*m_nbCurrentTurns;
    m_previousReadAngle = readAngle;

    if (m_currentMode != ServoModes::NOT_SET && m_controlEnabled) {
        if (m_currentMode == ABS_POSITION || m_currentMode == INC_POSITION) {

        	m_driveCommand = m_PWMConvert * m_pController.update(m_currentAngle, m_targetAngle);
        	m_pController.updateStallDetection();
        }

        else if (m_currentMode == SPEED)
        {
        	float currentError = m_targetSpeed - m_currentSpeed;
        	float errorDiff = currentError - m_previousError;
            m_errorSum += currentError*m_tauv;

//            if (m_errorSum > (float)PWM_COUNT/m_kiv)
//            {
//            	m_errorSum = (float)PWM_COUNT/m_kiv;
//            }
//            else if (m_errorSum < -(float)PWM_COUNT/m_kiv)
//            {
//            	m_errorSum = -(float)PWM_COUNT/m_kiv;
//            }
           if (m_errorSum > m_PWMCountKiv)
		   {
        	   m_errorSum = m_PWMCountKiv;
		   }
		   else if (m_errorSum < -m_PWMCountKiv)
		   {
			   m_errorSum = -m_PWMCountKiv;
		   }

            m_driveCommand = m_kpv*currentError + m_kdv*(errorDiff/m_tauv) + m_kiv*m_errorSum;
            m_previousError = currentError;
        }

        // Updates the drive command
        //Limitccr
        if (m_driveCommand > PWM_LIMIT)
        {
        	m_driveCommand = PWM_LIMIT;
        }
        else if (m_driveCommand < -PWM_LIMIT)
        {
        	m_driveCommand = -PWM_LIMIT;
        }

        if(m_driveMode == DRIVE_MODE_PHASE_ENABLE)
        {
        	if (m_driveCommand > 0.0) {
        		EN_IN1_PWM = (uint32_t)(m_driveCommand);
        		PH_IN2_PWM = 0; //anciennement: HAL_GPIO_WritePin(MTR_PH_GPIO_Port, MTR_PH_Pin, GPIO_PIN_SET);
			}
			else
			{
				EN_IN1_PWM = (uint32_t)(-m_driveCommand);
				PH_IN2_PWM = PWM_LIMIT; //anciennement: HAL_GPIO_WritePin(MTR_PH_GPIO_Port, MTR_PH_Pin, GPIO_PIN_RESET);


			}
        }
        else //DRIVE_MODE_PWM
        {
        	if (m_driveCommand > 0) {
				EN_IN1_PWM =  0; //;
				PH_IN2_PWM = (uint32_t)(m_driveCommand);
			}
			else {

				EN_IN1_PWM = (uint32_t)(-m_driveCommand);
				PH_IN2_PWM = 0;
			}
        }


    }
}

/**
 * @brief Wraps the angle between 0 and 360 degrees
 */
void OmniServo::resetTo360() {
    if (m_currentMode != SPEED) {
        if (m_refDirection > 0) {m_nbCurrentTurns = 0;}
        else {m_nbCurrentTurns = -1;}
        m_currentAngle = getAdjustedAngle() + m_nbCurrentTurns*360;
        if (m_currentAngle == -360) { // Particular case of landing on 0 while reversing refDir
        	m_currentAngle = 0;
        	m_nbCurrentTurns = 0;
        }
        m_targetAngle = m_currentAngle;
        m_previousReadAngle = 180;
        m_errorSum = 0;
        updateCommand();
    }
}

/**
 * @brief Sets origin to current angle
 * @warning The method first wraps the current angle to [0,360[ degrees
 */
void OmniServo::setOrigin() {
	if (m_currentMode != SPEED) {
		m_originRef = m_encoder.getRawAngle();
		resetTo360();

	}
}

/**
 * @brief Sets origin to given angle from hardware default
 * @param[in] p_angle Origin angle
 * @warning The method will wrap the given angle to [0,360[ degrees
 */
void OmniServo::setOrigin(const float& p_angle) {
	if (m_currentMode != SPEED) {
		float angle = p_angle * m_refDirection;
		if (m_currentUnits == RADIANS) {angle = rad2deg(angle);}
		m_originRef = angle2raw(wrapTo360(angle));
		resetTo360();

	}
}

/**
 * @brief Changes the origin so that the current angle matches a given one
 * @param[in] p_angle Angle to apply to the current one
 */
void OmniServo::setCurrentAngle(const float& p_angle) {
	if (m_currentMode != SPEED) {
		float angle = p_angle * m_refDirection;
		if (m_currentUnits == RADIANS) {angle = rad2deg(angle);}
		m_originRef = 0;
		m_currentAngle = getAdjustedAngle();
		m_originRef = angle2raw(wrapTo360(m_currentAngle - angle));
		float currentTurns = ((int16_t)(angle) / 360);
		if (angle < 0) {currentTurns += -1;}
		resetTo360();

	    if (m_refDirection < 0) {currentTurns += 1;}
	    setTurns(currentTurns*m_refDirection);
	}
}

/**
 * @brief Switches the reference direction
 * @warning The method will wrap the given angle to [0,360[ degrees
 */
void OmniServo::switchRef() {
	if (m_refDirection > 0) {m_refDirection = -1;}
	else {m_refDirection = 1;}
	resetTo360();

}

/**
 * @brief Sets the number of turns to the turn counter
 * @param[in] p_turns Number of turns to set
 */
void OmniServo::setTurns(const int16_t& p_turns) {
	if (m_currentMode != SPEED) {
		m_nbCurrentTurns = p_turns * m_refDirection;
		if (m_refDirection < 0) {m_nbCurrentTurns += -1;}
		m_currentAngle = getAdjustedAngle() + 360.0*m_nbCurrentTurns;
		m_previousReadAngle = 180;
		m_previousAngle10Hz = m_currentAngle;
		m_currentSpeed = 0;
		m_previousSpeed = 0;
	}
}

/**
 * @brief Sets center of the initial read range
 * @param[in] p_angle Center angle of the range
 */
void OmniServo::setCenterReadAngle(const float& p_angle) {
	float angle = p_angle;
	if (m_currentUnits == RADIANS) {angle = rad2deg(angle);}
	m_centerReadAngle = angle;
	resetTo360();

}

/**
 * @brief Forces the read angle in a given range around a middle value
 * @note This function is meant to be used for the initial angle read of the OmniServo.
 * Since the OmniServo reads its initial angle in the [0,360[ deg range, some programs
 * may struggle on motor wake up if the application is meant to start outside of that range.
 * As an example, giving -90 deg will force the first angle to be read in the [-270,90[ deg range.
 * Note that it is also possible to give a value with boundaries higher than 360 deg or lower than -360 deg.
 * @warning This function must be used right after establishing motor connection.
 */
void OmniServo::centerInitialRead() {
	m_currentAngle = getAdjustedAngle();
	int nbTurns = 0;
	float centerAngle = m_centerReadAngle * m_refDirection;
	float HigherLimit = centerAngle + 180;

	if (centerAngle > 180) {
		nbTurns = (centerAngle+180)/360;
		HigherLimit = centerAngle - 360*nbTurns + 180;
	}
	else if (centerAngle <= -180) {
		nbTurns = (centerAngle-180)/360;
		HigherLimit = centerAngle - 360*nbTurns + 180;
	}

	if (m_currentAngle > HigherLimit) {nbTurns += -1;}
	if (m_refDirection < 0) {nbTurns += 1;} // Added to counter the subtraction of turns in the setTurns function
	setTurns(nbTurns*m_refDirection); // Added direction to counter the same multiplication in the function
}

/**
 * @brief Command to change the torque constant used to compute the torque
 * @note The default value is for the Langyi 2232R-04A motor usually used
 * in the OmniServo. The torque is computed from the electrical current reading.
 * @param[in] p_command New torque constant (in mNm/A) of the DC motor used inside the OmniServo
 */
void OmniServo::setTorqueConstant(const float& p_torqueConstant) {
	m_torqueConstant = p_torqueConstant;
	resetTo360();

}

/**
 * @brief Accessor for the angle ajusted for the zero reference
 * @return Ajusted float angle
 */
float OmniServo::getAdjustedAngle() {
    int intVal = m_encoder.getRawAngle();
    intVal += -m_originRef;
    if (intVal < 0) {intVal += 4096;}
    return raw2angle(intVal);
}

/**
 * @brief Compute the current speed and sets it to its member variable
 */
void OmniServo::computeSpeed() {
    m_currentSpeed = (m_currentAngle - m_previousAngle10Hz)*10.0; //in deg/s

    float percent = 0.6;
    m_currentSpeed = m_previousSpeed*(1-percent) + m_currentSpeed*(percent);

    m_previousAngle10Hz = m_currentAngle;
    m_previousSpeed = m_currentSpeed;
}

/**
 * @brief Updates the current current & temperature values and sets them to their member variables
 */
void OmniServo::updateCurrentAndTemp() {
	/*if (adcConvDone) { // Investigate ADC Interrupt Error
		adcConvDone = false;*/
	if (hadc2.DMA_Handle->State != HAL_DMA_STATE_BUSY) {
	    m_currentCurrent = dmap((double)m_adcBuffer[0], 0, 4095, 0, VREF) / (RrefCurrentSense * currentScaling);
		float Vth = dmap((double)m_adcBuffer[1], 0, 4095, 0 ,VREF); //Volts
		float Rth = 10000 * Vth/(3.3-Vth); //Ohm
		m_currentTemp = lookUpTemp(Rth);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)m_adcBuffer, 2);
	}
}

/**
 * @brief Accessor for the motor ID
 * @return Motor ID
 */
uint8_t OmniServo::reqMotorID() {return m_motorID;}

/**
 * @brief Accessor for the current angle
 * @return Current angle
 */
float OmniServo::reqCurrentAngle() {
    float angle = m_currentAngle * m_refDirection;
    if (m_currentUnits == RADIANS) {angle = deg2rad(angle);}
    return angle;
}

/**
 * @brief Accessor for the target angle
 * @return Target angle
 */
float OmniServo::reqTargetAngle() {
    float angle = m_targetAngle * m_refDirection;
    if (m_currentUnits == RADIANS) {angle = deg2rad(angle);}
    return angle;
}

/**
 * @brief Accessor for the current speed
 * @return Current speed
 */
float OmniServo::reqCurrentSpeed() {
    float speed = m_currentSpeed * m_refDirection;
    if (m_currentUnits == RADIANS) {speed = deg2rad(speed);}
    return speed;
}

/**
 * @brief Accessor for the target speed
 * @return Target speed
 */
float OmniServo::reqTargetSpeed() {
    float speed = m_targetSpeed * m_refDirection;
    if (m_currentUnits == RADIANS) {speed = deg2rad(speed);}
    return speed;
}

/**
 * @brief Accessor for current drive command
 * @return Current drive command
 */
float OmniServo::reqDriveCommand() {return m_driveCommand;}

/**
 * @brief Accessor for the current servo mode
 * @return Current servo mode
 */
ServoModes OmniServo::reqCurrentMode() {return m_currentMode;}

/**
 * @brief Accessor for the current working units
 * @return Current working units
 */
WorkingUnits OmniServo::reqWorkingUnits() {return m_currentUnits;}

/**
 * @brief Accessor for the current current
 * @return Current current (Amps)
 */
float OmniServo::reqCurrentCurrent() {return m_currentCurrent;}

/**
 * @brief Accessor for the current temperature
 * @return Current temperature (Celsius)
 */
float OmniServo::reqCurrentTemp() {return m_currentTemp;}

float OmniServo::reqTorqueConstant() {return m_torqueConstant;}

/**
 * @brief Resets the servo to its default state
 */
void OmniServo::reset() {
	//changeMode(DISABLED);
	changeControlEnable(false);
	resetTo360();
	centerInitialRead();
	//NVIC_SystemReset();
}

/**
 * @brief Sets current values to the config structure
 */
void OmniServo::setConfigData() {
	m_config.data.motorID = m_motorID;
	m_config.data.workingUnits = (uint8_t)m_currentUnits;
	m_config.data.originRef = m_originRef;
	m_config.data.refDirection = m_refDirection;
	m_config.data.centerReadAngle = m_centerReadAngle;
	m_config.data.torqueConstant = m_torqueConstant;
	m_config.data.kpp = m_pController.m_Kp;
	m_config.data.kdp = m_pController.m_Kd;
	m_config.data.kip = m_pController.m_Ki;
	m_config.data.kpv = m_kpv;
	m_config.data.kdv = m_kdv;
	m_config.data.kiv = m_kiv;
	m_config.data.driveMode = m_driveMode;


	m_config.data.driveMode	 = m_driveMode;
	m_config.data.maxVelocity = m_pController.m_maxVelocity;
	m_config.data.maxAcceleration = m_pController.m_maxAcceleration;
	m_config.data.positionDeadband = m_pController.m_deadband;
	m_config.data.maxPosition = m_pController.m_maxPosition;
	m_config.data.minPosition = m_pController.m_minPosition;
	m_config.data.positionStallThreshold = m_pController.m_stallThreshold;
	m_config.data.positionStallTimeout = m_pController.m_stallTimeout;
	m_config.data.positionLimitsEnabled = m_pController.m_limitsEnabled;

}

/**
 * @brief Applies the config values to the used values
 */
void OmniServo::applyConfigData() {
	m_motorID = (uint8_t)m_config.data.motorID;
	m_currentUnits = (WorkingUnits)m_config.data.workingUnits;
	m_originRef = m_config.data.originRef;
	m_refDirection = m_config.data.refDirection;
	m_centerReadAngle = m_config.data.centerReadAngle;
	m_torqueConstant = m_config.data.torqueConstant;
	m_pController.setGains(m_config.data.kpp, m_config.data.kip, m_config.data.kdp);

	//m_PWMCountKip = ((float)PWM_COUNT)/m_kip;
	m_kpv = m_config.data.kpv;
	m_kdv = m_config.data.kdv;
	m_kiv = m_config.data.kiv;
	m_PWMCountKiv = ((float)PWM_COUNT)/m_kiv;

	m_driveMode = m_config.data.driveMode;
	m_pController.m_maxVelocity = m_config.data.maxVelocity;
	m_pController.m_maxAcceleration = m_config.data.maxAcceleration;
	m_pController.m_deadband = m_config.data.positionDeadband;
	m_pController.m_maxPosition = m_config.data.maxPosition;
	m_pController.m_minPosition = m_config.data.minPosition;
	m_pController.m_stallThreshold = m_config.data.positionStallThreshold;
	m_pController.m_stallTimeout = m_config.data.positionStallTimeout;
	m_pController.m_limitsEnabled = m_config.data.positionLimitsEnabled;
}

/**
 * @brief Writes the config to the Flash memory
 * @param[in] Data Pointer to the config data struct casted to uint32_t*
 * @param[in] numberofdoublewords Sizeof(dataStruct)/4
 */
uint32_t OmniServo::writeFlashConfig(uint32_t *Data, uint16_t numberofdoublewords) {
	uint32_t StartPageAddress = BACKUP_START_ADDRESS;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area*/
	uint32_t StartPage = GetPage(StartPageAddress);
	uint32_t EndPageAdress = StartPageAddress + numberofdoublewords*8;
	uint32_t EndPage = GetPage(EndPageAdress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = StartPage;
	EraseInitStruct.NbPages = ((EndPage - StartPage)/FLASH_PAGE_SIZE) +1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		/*Error occurred while page erase.*/
		return HAL_FLASH_GetError ();
	}

	/* Program the user Flash area word by word*/

	while (sofar<numberofdoublewords)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data[sofar]) == HAL_OK)
		{
			StartPageAddress += 8;  // use StartPageAddress += 2 for half word (2 bytes) and 8 for double word (8 bytes)
			sofar++;
		}
		else
		{
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError ();
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	  to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
}

/**
 * @brief Reads the config data from the flash
 * @param[in] Data Pointer to the config data struct casted to uint32_t*
 * @param[in] numberofdoublewords Sizeof(dataStruct)/4
 */
void OmniServo::readConfigData(uint32_t *Data, uint16_t numberofdoublewords) {
	uint32_t StartPageAddress = BACKUP_START_ADDRESS;
	while (1)
	{
		*Data = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 8; // AKA +64 bits or +8 bytes
		Data++;
		if (!(--numberofdoublewords)) break;
	}
}

/**
 * @brief Gets page number from memory address
 * @param[in] Addr Address of the page
 * @return Page number of the address
 */
uint32_t OmniServo::GetPage(uint32_t Addr)
{
  return((Addr - FLASH_BASE) / FLASH_PAGE_SIZE);
}

/**
 * @brief Computes the change from radians to degrees
 * @param p_value Angle in radians
 * @return Converted angle to degrees
 */
float OmniServo::rad2deg(float p_value) {return p_value * (180.0/PI);}

/**
 * @brief Computes the change from degrees to radians
 * @param[in] p_value Angle in degrees
 * @return Converted angle to radians
 */
float OmniServo::deg2rad(float p_value) {return p_value * (PI/180.0);}

/**
 * @brief Computes the change from degrees to raw angle counts
 * @param[in] p_angle Angle in degrees
 * @return Converted angle to raw angle counts
 * @note 1 rotation equals 12 bits (4096 counts)
 */
int OmniServo::angle2raw(float p_angle) {
    // Counting extra turns and maping the rest in range [0,360[
    int nbTurns = ((int)p_angle)/360;
    p_angle = p_angle - 360.0*nbTurns;
    if (p_angle < 0) {p_angle += 360;}
    // Mapping to raw value and adding extra turns
    return dmap(p_angle, 0.0, 360.0, 0.0, 4096.0) + 4096.0*nbTurns;
}

/**
 * @brief Computes the change from raw angle counts to degrees
 * @param[in] p_raw Angle in raw angle counts
 * @return Converted angle to degrees
 * @warning This method is only meant to be used for a 12 bit count ([0,4095[)
 */
float OmniServo::raw2angle(int p_raw) {
    return dmap(p_raw, 0.0, 4096.0, 0.0, 360.0);
}

/**
 * @brief Maps value from an initial interval to a target interval with double precision
 * @param[in] p_value Value to map
 * @param[in] from_low Lower limit of initial interval
 * @param[in] from_high Higher limit of initial interval
 * @param[in] to_low Lower limit of target interval
 * @param[in] to_high Higher limit of target interval
 * @return Mapped value to target interval as a double
 */
double OmniServo::dmap(double p_value, double from_low, double from_high, double to_low, double to_high) {
    return (p_value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low;
}

/**
 * @brief Wraps given angle to [0,360[ degrees
 * @param[in] p_angle Angle to wrap
 * @return Wrapped angle
 */
float OmniServo::wrapTo360(float p_angle) {
	p_angle = fmod(p_angle,360);
	if (p_angle < 0)
		p_angle += 360;
	return p_angle;
}

/*
 * @brief Computes the thermistor temperature
 * @param[in] p_resistance Resistance of the thermistor
 * @return Temperature of the thermistor in celsius
 */
float OmniServo::lookUpTemp(float p_resistance) {
	float ratio = p_resistance/10000;
	float temp = 0;
	if (ratio >= Rratio[0]) {temp = temps[0];}
	else if (ratio <= Rratio[nbTempValues-1]) {temp = temps[nbTempValues-1];}
	else {
		for (int i=0; i<nbTempValues-2; i++) {
			if (Rratio[i] >= ratio && Rratio[i+1] <= ratio) {
				temp = (((float)temps[i]-(float)temps[i+1])/(Rratio[i]-Rratio[i+1])) * (ratio-Rratio[i+1]) + temps[i+1];
				break;
			}
		}
	}
	return temp;
}

/* // Investigate ADC Interrupt Error
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc == &hadc2) {
		adcConvDone = true;
	}
}
*/
