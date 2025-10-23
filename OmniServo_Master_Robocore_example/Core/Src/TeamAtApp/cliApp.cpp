/*
#include <DrinkReminder.h>
 * cli.cpp
 *
 *  Created on: Jan 6, 2022
 *      Author: teamat
 */



#include "cliApp.h"
#include "uartApp.h"

#include "ctype.h"



    cli myCLI; // our Command Line instance

    extern uint8_t serialdebugonoff;

    extern const int numCliCommands;
    extern CLI_FUNC_PTR commands_func[];
    extern const char *commands_str[];

	#define SERIALDEBUG_BUFFER_LINE_SIZE	 128  				//Nb caractères maximale dans la ligne
	char debugSerialReceivedLine[SERIALDEBUG_BUFFER_LINE_SIZE]; // Buffer de travail, lorsqu'une ligne complète est reçue, elle est copié dans ce buffer



//    CLI_FUNC_PTR commands_func[]{
//    		 &cmd_getMotorInfo,
//			 &cmd_findServos
//         };
//
//    const char *commands_str[] = {
//    		 "getMotorInfo",
//			 "findServos"
//         };

    int num_cli_commands = numCliCommands;//sizeof(commands_str) / sizeof(char *); // Used to calcule the number of commands in the list.



void cli_init( CLI_SERIALOUT_CALLBACK serialOutFunction)
{

	uartAppInit(); // Uart App init function

	 myCLI.init(num_cli_commands,  // The number of implemented commands/functions
				  commands_str,     // The string list for all the commands
				  commands_func,    // The pointer lists for the commands
				  serialOutFunction,     // Funtion to used to put data on the serial port
				  VERBOSE_ON);     // Communicate status on serial port if OFF
									// ( on is useful for troubleshooting but slower)


  myCLI.start();  // Start to interpret commands on the serial port.
}


int cliAppRun()
{
	// On regarde si on a reçu une ligne de commande
	int bytesRead = serial1.readToChar(debugSerialReceivedLine,'\r');
	if(bytesRead)
	{
		//debug_newLineReceived = false;
		myCLI.run(debugSerialReceivedLine); // On passe la ligne reçue au cli
		memset(debugSerialReceivedLine, 0, sizeof(debugSerialReceivedLine));

	}

//	if(uart1_dataReadyFlag)
//	{
//		uart1_dataReadyFlag = false;
//
//		// Run the command line interpreter
//		int status = myCLI.run(inputBuffer);
//
//		if (status < 0) // If error
//		{
//
//		}
//		else if (status > 0) // If positive value, user function want to communicate something
//		{
//			// TODO traiter les valeurs de status > 0
//		}
//
//		myCLI.displayCommandPrompt();
//
//	}

	return 0;
}



