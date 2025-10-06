/*
 * mainApp.h
 *
 *  Created on: 22 sept. 2022
 *      Author: teamat
 */

// Inclusion pour AT COMM
#include "atCommOmniServoDefines.h"
#include "atComm.h"
#include "backupData_OmniServo.h"
#include "OmniServo_STM_Slave.h"

#ifndef TEAMATAPP_MAINAPP_H_
#define TEAMATAPP_MAINAPP_H_

/*********************************************************************/
//#define MY_ID SLAVE_1_ID // À MODIFIER POUR GÉRER L'IDENTITÉ DU MOTEUR
/*********************************************************************/

#define SERIAL_DATA_BUFFER_SIZE			1024
#define ATCOMM_BUFFER_SIZE				1042

extern atComm txComm;
extern atComm rxComm;
extern uint8_t serialDataBuffer[SERIAL_DATA_BUFFER_SIZE];

void setup();
void loop();

#endif /* TEAMATAPP_MAINAPP_H_ */
