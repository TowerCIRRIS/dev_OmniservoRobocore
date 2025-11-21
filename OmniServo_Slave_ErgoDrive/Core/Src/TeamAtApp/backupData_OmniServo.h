#ifndef BACKUPDATA_OMNISERVO_H
#define BACKUPDATA_OMNISERVO_H


#include "atCommOmniServoDefines.h"
#define BACKUP_STRUCT_VERSION   7
/* Version 4
 * Includes the reference direction int
 */
#define BACKUP_START_ADDRESS	0x0801F800 // Last available page

/*
 * The STM32G4 can only program in 64-bit. Since all data programmed is 32-bit, every other 32 bits will be empty.
 */

typedef struct
{
	uint32_t startCode = 0xDEADBEEF;
	uint32_t backupStructVersion = BACKUP_STRUCT_VERSION;

	ServoConfigInfo data;
	uint32_t stopCode = 0xDEADBEEF;
} __attribute__ ((aligned (4))) backupStruct_t;

#endif