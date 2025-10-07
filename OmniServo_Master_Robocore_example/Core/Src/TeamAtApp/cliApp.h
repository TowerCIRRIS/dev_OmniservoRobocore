/*
 * cli.h
 *
 *  Created on: Jan 6, 2022
 *      Author: teamat
 */

#ifndef SRC_CLI_H_
#define SRC_CLI_H_

#include "cli_v1_1.h"
#include "uartApp.h"

extern cli myCLI; // our Command Line instance

extern const int numCliCommands;
extern CLI_FUNC_PTR commands_func[];
extern const char *commands_str[];


void cli_init( CLI_SERIALOUT_CALLBACK serialOutFunction);
int cliAppRun();




//int cmd_setTime(const char* argString);
//int cmd_setDate(const char* argString);
//
//int cmd_startLog(const char* argString);
//int cmd_stopLog(const char* argString);
//
//int cmd_sleep(const char* argString);
//int cmd_wakeup(const char* argString);


#endif /* SRC_CLI_H_ */
