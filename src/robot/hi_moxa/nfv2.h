#ifndef NFV2_H
#define NFV2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>			    
#include "nfv2_config.h"
#include "mycrc.h"

/*
* Command codes
*/
#define NF_COMMAND_ReadDeviceStatus		0x02
#define NF_COMMAND_ReadDeviceVitals		0x03

#define NF_COMMAND_SetDrivesMode		0x10
#define NF_COMMAND_SetDrivesSpeed		0x11
#define NF_COMMAND_SetDrivesCurrent		0x12
#define NF_COMMAND_SetDrivesPosition	0x13
#define NF_COMMAND_SetDrivesPWM			0x14
#define NF_COMMAND_SetDrivesMaxCurrent	0x15

#define NF_COMMAND_ReadDrivesPosition	0x1A
#define NF_COMMAND_ReadDrivesCurrent	0x1B
#define NF_COMMAND_SetDrivesMisc		0x1E
#define NF_COMMAND_ReadDrivesStatus		0x1F

#define NF_COMMAND_SetServosMode		0x20
#define NF_COMMAND_SetServosPosition	0x21
#define NF_COMMAND_SetServosSpeed		0x22


#define NF_COMMAND_ReadDigitalInputs	0x30
#define NF_COMMAND_SetDigitalOutputs	0x31

#define NF_COMMAND_ReadAnalogInputs		0x3E


// ########	Device
// ####		Read Status
#ifdef NF_BUFSZ_ReadDeviceStatus
typedef struct{
	#define NF_DATABYTES_ReadDeviceStatus		2
	int16_t data[NF_BUFSZ_ReadDeviceStatus];
	uint8_t addr[NF_BUFSZ_ReadDeviceStatus];
	uint8_t updated;
} NF_STRUCT_ReadDeviceStatus;
#endif
// ####		Read Vitals
#ifdef NF_BUFSZ_ReadDeviceVitals
typedef struct{
	#define NF_DATABYTES_ReadDeviceVitals		2
	int16_t data[NF_BUFSZ_ReadDeviceVitals];
	uint8_t addr[NF_BUFSZ_ReadDeviceVitals];
	uint8_t updated;
} NF_STRUCT_ReadDeviceVitals;
#endif

// ########	Drives
// ####		Set Mode
#ifdef NF_BUFSZ_SetDrivesMode
typedef struct{
	#define NF_DATABYTES_SetDrivesMode		1
	uint8_t data[NF_BUFSZ_SetDrivesMode];
	uint8_t addr[NF_BUFSZ_SetDrivesMode];
	uint8_t updated;
} NF_STRUCT_SetDrivesMode;
// ##		Possible Values
#define NF_DrivesMode_ERROR			0x00
#define NF_DrivesMode_MANUAL		0x01
#define NF_DrivesMode_SPEED			0x02
#define NF_DrivesMode_CURRENT		0x03
#define NF_DrivesMode_POSITION		0x04
#define NF_DrivesMode_PWM			0x05
#define NF_DrivesMode_SYNC_PWM0		0x10
#endif
// ####		Set Speed
#ifdef NF_BUFSZ_SetDrivesSpeed
typedef struct{
	#define NF_DATABYTES_SetDrivesSpeed		2
	int16_t data[NF_BUFSZ_SetDrivesSpeed];
	uint8_t addr[NF_BUFSZ_SetDrivesSpeed];
	uint8_t updated;
} NF_STRUCT_SetDrivesSpeed;
#endif
// ####		Set Current
#ifdef NF_BUFSZ_SetDrivesCurrent
typedef struct{
	#define NF_DATABYTES_SetDrivesCurrent	2
	int16_t data[NF_BUFSZ_SetDrivesCurrent];
	uint8_t addr[NF_BUFSZ_SetDrivesCurrent];
	uint8_t updated;
} NF_STRUCT_SetDrivesCurrent;
#endif
// ####		Set Position
#ifdef NF_BUFSZ_SetDrivesPosition
typedef struct{
	#define NF_DATABYTES_SetDrivesPosition	4
	int32_t data[NF_BUFSZ_SetDrivesPosition];
	uint8_t addr[NF_BUFSZ_SetDrivesPosition];
	uint8_t updated;
} NF_STRUCT_SetDrivesPosition;
#endif
// ####		Set PWM
#ifdef NF_BUFSZ_SetDrivesPWM
typedef struct{
	#define NF_DATABYTES_SetDrivesPWM		2
	int16_t data[NF_BUFSZ_SetDrivesPWM];
	uint8_t addr[NF_BUFSZ_SetDrivesPWM];
	uint8_t updated;
} NF_STRUCT_SetDrivesPWM;
#endif
// ####		Set Max Current
#ifdef NF_BUFSZ_SetDrivesMaxCurrent
typedef struct{
	#define NF_DATABYTES_SetDrivesMaxCurrent	2
	int16_t data[NF_BUFSZ_SetDrivesMaxCurrent];
	uint8_t addr[NF_BUFSZ_SetDrivesMaxCurrent];
	uint8_t updated;
} NF_STRUCT_SetDrivesMaxCurrent;
#endif
// ####		Read Current
#ifdef NF_BUFSZ_ReadDrivesCurrent
typedef struct{
	#define NF_DATABYTES_ReadDrivesCurrent	2
	int16_t data[NF_BUFSZ_ReadDrivesCurrent];
	uint8_t addr[NF_BUFSZ_ReadDrivesCurrent];
	uint8_t updated;
} NF_STRUCT_ReadDrivesCurrent;
#endif
// ####		Read Position
#ifdef NF_BUFSZ_ReadDrivesPosition
typedef struct{
	#define NF_DATABYTES_ReadDrivesPosition	4
	int32_t data[NF_BUFSZ_ReadDrivesPosition];
	uint8_t addr[NF_BUFSZ_ReadDrivesPosition];
	uint8_t updated;
} NF_STRUCT_ReadDrivesPosition;
#endif
// ####		Set Misc
#ifdef NF_BUFSZ_SetDrivesMisc
typedef struct{
	#define NF_DATABYTES_SetDrivesMisc		4
	uint32_t data[NF_BUFSZ_SetDrivesMisc];
	uint8_t addr[NF_BUFSZ_SetDrivesMisc];
	uint8_t updated;
} NF_STRUCT_SetDrivesMisc;
// ##		Possible Values (bitwise OR)
#define NF_DrivesMisc_SetSynchronized		(1 << 0)
#define NF_DrivesMisc_ResetSynchronized		(1 << 1)
#endif
// ####		Read Status
#ifdef NF_BUFSZ_ReadDrivesStatus
typedef struct{
	#define NF_DATABYTES_ReadDrivesStatus	2
	uint16_t data[NF_BUFSZ_ReadDrivesStatus];
	uint8_t addr[NF_BUFSZ_ReadDrivesStatus];
	uint8_t updated;
} NF_STRUCT_ReadDrivesStatus;
// ##		Possible Values (bitwise OR)
#define NF_DrivesStatus_LimitSwitchUp		(1 << 0)
#define NF_DrivesStatus_LimitSwitchDown		(1 << 1)
#define NF_DrivesStatus_SynchroSwitch		(1 << 2)
#define NF_DrivesStatus_EncoderIndexSignal	(1 << 3)
#define NF_DrivesStatus_Synchronized		(1 << 4)
#define NF_DrivesStatus_PositionLimit		(1 << 10)
#define NF_DrivesStatus_SpeedLimit			(1 << 11)
#define NF_DrivesStatus_CurrentLimit		(1 << 12)
#define NF_DrivesStatus_Overcurrent			(1 << 13)
#define NF_DrivesStatus_PowerStageFault		(1 << 14)
#define NF_DrivesStatus_Error				(1 << 15)
#endif
	
// ########	Servos
// ####		Set Mode
#ifdef NF_BUFSZ_SetServosMode
typedef struct{
	#define NF_DATABYTES_SetServosMode		1
	uint8_t data[NF_BUFSZ_SetServosMode];
	uint8_t addr[NF_BUFSZ_SetServosMode];
	uint8_t updated;
} NF_STRUCT_SetServosMode;
#endif
// ####		Set Position
#ifdef NF_BUFSZ_SetServosPosition
typedef struct{
	#define NF_DATABYTES_SetServosPosition	2
	uint16_t data[NF_BUFSZ_SetServosPosition];
	uint8_t addr[NF_BUFSZ_SetServosPosition];
	uint8_t updated;
} NF_STRUCT_SetServosPosition;
#endif
// ####		Set Speed
#ifdef NF_BUFSZ_SetServosSpeed
typedef struct{
	#define NF_DATABYTES_SetServosSpeed		1
	int8_t data[NF_BUFSZ_SetServosSpeed];
	uint8_t addr[NF_BUFSZ_SetServosSpeed];
	uint8_t updated;
} NF_STRUCT_SetServosSpeed;
#endif

// ########	Digital IO
// ####		Read Inputs
#ifdef NF_BUFSZ_ReadDigitalInputs
typedef struct{
	#define NF_DATABYTES_ReadDigitalInputs	1
	int16_t data[NF_BUFSZ_ReadDigitalInputs];
	uint8_t addr[NF_BUFSZ_ReadDigitalInputs];
} NF_STRUCT_ReadDigitalInputs;
#endif
// ####		Set Outputs
#ifdef NF_BUFSZ_SetDigitalOutputs
typedef struct{
	#define NF_DATABYTES_SetDigitalOutputs	1
	int8_t data[NF_BUFSZ_SetDigitalOutputs];
	uint8_t addr[NF_BUFSZ_SetDigitalOutputs];
	uint8_t updated;
} NF_STRUCT_SetDigitalOutputs;
#endif

// ########	Analog IO
// ####		Read Inputs
#ifdef NF_BUFSZ_ReadAnalogInputs
typedef struct{
	#define NF_DATABYTES_ReadAnalogInputs	2
	int16_t data[NF_BUFSZ_ReadAnalogInputs];
	uint8_t addr[NF_BUFSZ_ReadAnalogInputs];
	uint8_t updated;
} NF_STRUCT_ReadAnalogInputs;
#endif

typedef struct{
	// ########	Device
	// ####		Read Status
	#ifdef NF_BUFSZ_ReadDeviceStatus
		NF_STRUCT_ReadDeviceStatus	ReadDeviceStatus;
	#endif
	// ####		Read Vitals
	#ifdef NF_BUFSZ_ReadDeviceVitals
		NF_STRUCT_ReadDeviceVitals	ReadDeviceVitals;
	#endif
	
	// ########	Drives
	// ####		Set Mode
	#ifdef NF_BUFSZ_SetDrivesMode
		NF_STRUCT_SetDrivesMode		SetDrivesMode;
	#endif
	// ####		Set Speed
	#ifdef NF_BUFSZ_SetDrivesSpeed
		NF_STRUCT_SetDrivesSpeed		SetDrivesSpeed;
	#endif
	// ####		Set Current
	#ifdef NF_BUFSZ_SetDrivesCurrent
		NF_STRUCT_SetDrivesCurrent	SetDrivesCurrent;
	#endif
	// ####		Set Position
	#ifdef NF_BUFSZ_SetDrivesPosition
		NF_STRUCT_SetDrivesPosition	SetDrivesPosition;
	#endif
	// ####		Set PWM
	#ifdef NF_BUFSZ_SetDrivesPWM
		NF_STRUCT_SetDrivesPWM	SetDrivesPWM;
	#endif
	// ####		Set Max Current
	#ifdef NF_BUFSZ_SetDrivesMaxCurrent
		NF_STRUCT_SetDrivesMaxCurrent	SetDrivesMaxCurrent;
	#endif
	// ####		Read Current
	#ifdef NF_BUFSZ_ReadDrivesCurrent
		NF_STRUCT_ReadDrivesCurrent	ReadDrivesCurrent;
	#endif
	// ####		Read Position
	#ifdef NF_BUFSZ_ReadDrivesPosition
		NF_STRUCT_ReadDrivesPosition	ReadDrivesPosition;
	#endif
	// ####		Set Misc
	#ifdef NF_BUFSZ_SetDrivesMisc
		NF_STRUCT_SetDrivesMisc	SetDrivesMisc;
	#endif
	// ####		Read Status
	#ifdef NF_BUFSZ_ReadDrivesStatus
		NF_STRUCT_ReadDrivesStatus	ReadDrivesStatus;
	#endif
	// ########	Servos
	// ####		Set Mode
	#ifdef NF_BUFSZ_SetServosMode
		NF_STRUCT_SetServosMode		SetServosMode;
	#endif
	// ####		Set Position
	#ifdef NF_BUFSZ_SetServosPosition
		NF_STRUCT_SetServosPosition	SetServosPosition;
	#endif
	// ####		Set Speed
	#ifdef NF_BUFSZ_SetServosSpeed
		NF_STRUCT_SetServosSpeed		SetServosSpeed;
	#endif
	
	// ########	Digital IO	 
	// ####		Read Inputs
	#ifdef NF_BUFSZ_ReadDigitalInputs
		NF_STRUCT_ReadDigitalInputs	ReadDigitalInputs;
	#endif
	// ####		Set Outputs
	#ifdef NF_BUFSZ_SetDigitalOutputs
		NF_STRUCT_SetDigitalOutputs	SetDigitalOutputs;
	#endif
	
	// ########	Analog IO
	// ####		Read Inputs
	#ifdef NF_BUFSZ_ReadAnalogInputs
		NF_STRUCT_ReadAnalogInputs	ReadAnalogInputs;
	#endif

	// ########	Common Fields
	uint8_t myAddress;

	uint8_t dataReceived		:1;
	uint8_t unknownCommandRec	:1;
	uint8_t unknownCommandSend	:1;
} NF_STRUCT_ComBuf;



uint8_t NF_Interpreter(NF_STRUCT_ComBuf *NFComBuf, volatile uint8_t *rxBuf, volatile uint8_t *rxPt, volatile uint8_t *commandArray, volatile uint8_t *commandCnt);
uint8_t NF_MakeCommandFrame(NF_STRUCT_ComBuf *NFComBuf, uint8_t *txBuf, const uint8_t *commandArray, uint8_t commandCnt, uint8_t addr);
void NF_ComBufReset(NF_STRUCT_ComBuf *NFComBuf);

#ifdef __cplusplus
}
#endif

#endif //NFV2_H
