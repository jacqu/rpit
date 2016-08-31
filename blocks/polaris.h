/*
 * 	polaris.h:	API definitions for accessing the NDI Polaris localizer.
 * 
 * 	JG, 25.01.15
 */

/*
 * 	Basic includes
 */
 
// General C includes

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <errno.h>
#include <termios.h>

// Basic defines

#define POLARIS_ROM_FILE		"/home/pi/RTW/polaris.rom"

#define POLARIS_INI_TIMEOUT	200

#define POLARIS_SERIAL_PORT	"/dev/ttyUSB0"
#define POLARIS_SERIAL_BR		BR115200
#define POLARIS_STOP_BITS		0
#define POLARIS_PARITY			0
#define POLARIS_DATA_BITS		0
#define POLARIS_HARD_FLOW		0

#define POLARIS_FIRING_RATE	FR60

#define POLARIS_CUR_HANDLE	1

#define DEB      						true       /* DEBUG informations */
#define DEB_COMM 						false      /* Print all serial i/o */

// Let's make some noise
#define bBeepOnError                true
#define bBeepOnWarning              false
#define nNoErrorBeeps               1
#define	nNoWarningBeeps             0

// Basic typedefs

typedef int bool;

/* 
 * 	Low level communication API
 */

// Serial data formats

#define COMM_8N1	0x00
#define COMM_8N2	0x01
#define COMM_8O1	0x02
#define COMM_8O2	0x03
#define COMM_8E1	0x04
#define COMM_8E2	0x05

#define COMM_7N1 	0x06
#define COMM_7N2	0x07
#define COMM_7O1	0x08
#define COMM_7O2	0x09
#define COMM_7E1	0x0A
#define COMM_7E2	0x0B

#define COMM_OK                  	1
#define COMM_IRQ_IN_USE         	-1
#define COMM_OUT_OF_MEMORY      	-2
#define COMM_NOT_INITIALIZED    	-3
#define COMM_OUT_BUFFER_FULL    	-4
#define COMM_NOT_PRESENT        	-5
#define COMM_NO_DATA							-6

#define COMM_ERROR_SETTIMEOUT			-10
#define COMM_ERROR_SETBREAK				-11
#define COMM_ERROR_CLEARBREAK			-12
#define COMM_ERROR_CLEARBUFFER		-13
#define COMM_ERROR_CREATEEVENT		-14
#define COMM_ERROR_WRITE					-15
#define COMM_ERROR_WRITETIMEOUT		-16
#define COMM_ERROR_READ						-17
#define COMM_ERROR_READTIMEOUT		-18
#define COMM_ERROR_WAIT						-19
#define COMM_ERROR_WAITTIMEOUT		-20
#define COMM_ERROR_GETOVERLAPPED	-21

#define COMM_ERR_OVERRUN        	0x02
#define COMM_ERR_PARITY         	0x04
#define COMM_ERR_FRAME          	0x08
#define COMM_ERR_BREAK          	0x10

// Buffer size

#define READ_BUFFER_SIZE					3000
#define WRITE_BUFFER_SIZE					200


// Structures

typedef enum { BR9600 = 9600,
               BR14400 = 14400,
               BR19200 = 19200,
               BR38400 = 38400,
               BR57600 = 57600,
               BR115200 = 115200
} BaudRate;

typedef enum { FR20 = 0,
               FR30 = 1,
               FR60 = 2
} FiringRate;


typedef struct Comm32Port Comm32Port;
struct Comm32Port
{
  int							m_bRtsCts;					/* hardware handshaking */
	unsigned long 	m_ulSerBreakDelay;	/* delay after a serial break */

	unsigned char 	m_gruchInputBuffer[READ_BUFFER_SIZE];	/* input buffer */
	int							m_nIndexBuffer;			/* index of buffer */
	int							m_nNumBytes;				/* number of bytes read or sent */

	int							m_fdCommPort;				/* com port file descriptor */
};
Comm32Port* pCOMPort;

// Routine Definitions

void Comm32Port_construct( void );
void Comm32Port_destruct( void );

void Comm32Port_SerialClose( void );
int Comm32Port_SerialOpen( 	const char* portId, BaudRate ulBaudRate,
														unsigned uFormat, int bRtsCts );
int Comm32Port_SerialSetBaud( BaudRate ulBaudRate, unsigned uFormat, int bRtsCts );

int Comm32Port_SerialBreak( void );

int Comm32Port_SerialPutChar( unsigned char uch );
int Comm32Port_SerialPutString( unsigned char *pszString, unsigned long ulLen );

int  Comm32Port_SerialGetChar( void );
int  Comm32Port_SerialCharsAvailable( void );
int  Comm32Port_SerialGetString( unsigned char *pszString, unsigned long ulMaxLen );


void m_sleep( unsigned long ms );

/*
 * 	Helper function: CRC, char handling, ...
 */
 
// Routine Definitions

unsigned int 	uASCIIToHex( char szStr[], int nLen );
bool 					bExtractValue( 	char *pszVal, unsigned uLen,
															float fDivisor, float *pfValue );

int 					nGetHex2( char *sz );
int 					nGetHex1( char *sz );
int 					nGetHex4( char *sz );
float 				fGetFloat( char *sz );


void 					CRC_InitCrcTable( void );
unsigned int 	CRC_CalcCrc16( unsigned int crc, int data );
unsigned 			CRC_CalcCRCByLen( char *pszString, int nLen );
int 					CRC_SystemCheckCRC( char *psz );
unsigned int 	CRC_SystemGetCRC( char *psz, int nLength );

/*
 * 	API structures
 */
 
// Defines

// Number of handles the system can support

#define NO_HANDLES							0xFF

// Reply definitions

#define REPLY_ERROR							0x00
#define REPLY_OKAY							0x01
#define REPLY_RESET							0x02
#define REPLY_OTHER							0x04
#define REPLY_BADCRC						0x08
#define REPLY_WARNING						0x10
#define REPLY_INVALID						0x20

// Command variables

#define CARRIAGE_RETURN					0xD

#define MAX_REPLY_MSG						3072

#define MAX_COMMAND_MSG					1024
#define LINE_FEED    						0xA

// Transform return values

#define TRANSFORM_VALID					0x0000
#define	TRANSFORM_MISSING				0x1000
#define TRANSFORM_UNOCCUPIED		0x2000
#define TRANSFORM_DISABLED			0x3000
#define TRANSFORM_ERROR					0x4000

// Timeout returns

#define ERROR_TIMEOUT_CLOSE			0
#define ERROR_TIMEOUT_RESTART		1
#define ERROR_TIMEOUT_CONT			2

// Math numbers

#define BAD_FLOAT               (float)-3.697314E28
#define MAX_NEGATIVE            (float)-3.0E28

// Message handler

#define WM_COM_PORT_TO					WM_USER+2

// Structures

// POSITION 3D Structure

typedef struct Position3dStruct
{
    float   x;
    float   y;
    float   z;
} Position3d;

// QUATERNION Structure

typedef struct QuatRotationStruct
{
    float   q0;
    float   qx;
    float   qy;
    float   qz;
} QuatRotation;

// Transformation Information Structure

typedef struct
{
	unsigned long
		ulFlags,
		ulFrameNumber;
	QuatRotation
		rotation;
	Position3d
		translation;
	float
		fError;
}	TransformInformation;

// Handle Status Structure

typedef struct
{
	int
		bToolInPort,
		bGPIO1,
		bGPIO2,
		bGPIO3,
		bInitialized,
		bEnabled,
		bOutOfVolume,
		bPartiallyOutOfVolume,
		bDisturbanceDet,
		bSignalTooSmall,
		bSignalTooBig,
		bProcessingException,
		bHardwareFailure,

		bCoilBreak,
		bTIPCurrentSensing;
}	HandleStatus;

// Handle Information Structure

typedef struct
{
	char
		szPhysicalPort[5],
		szToolType[9],
		szManufact[13],
		szSerialNo[9],
		szRev[4],
		szChannel[3];
	TransformInformation
		Xfrms;
	HandleStatus
		HandleInfo;
}	HandleInformation;

// System Information Structure

typedef struct
{
	int
			nTypeofSystem;
	char
			szVersionInfo[1024];
	int
			bActivePortsAvail,
			bPassivePortsAvail,
			bMultiVolumeParms,
			bTIPSensing,
			bActiveWirelessAvail,
			bMagneticPortsAvail,
			bFieldGeneratorAvail;
	/* POLARIS ONLY FIELDS */
	int
			nNoActivePorts,
			nNoPassivePorts,
			nNoActTIPPorts,
			nNoActWirelessPorts;
	/* AURORA ONLY FIELDS */
	int
			nNoMagneticPorts,
			nNoFGCards,
			nNoFGs;
	/* TRACKING INFORMATION */
	int	bCommunicationSyncError;
	int bTooMuchInterference;
	int bSystemCRCError;
	int bRecoverableException;
	int bHardwareFailure;
	int bHardwareChange;
	int bPortOccupied;
	int bPortUnoccupied;
}	SystemInformation;

/*
 *	Command handling
 */
 
// Defines

#define NUM_COM_PORTS				10		/* number of com ports */
#define POLARIS_SYSTEM			0x01	/* type of system, POLARS */
#define AURORA_SYSTEM				0x02	/*  or AURORA */
#define ACCEDO_SYSTEM				0x04	/*  or ACCEDO */

#define MAX_COMM_ATTEMPTS   2     /* Max attempts sending a command before communication considered lost */
#define NB_ROM_FILES        12

// Routine Definitions

void p_construct( bool enableLog, int timeout );
void p_destruct( void );

int p_nCloseComPorts( void );
int p_nOpenComPort( const char* port );
int p_nHardWareReset( void );
int p_nSetSystemComParms(	BaudRate nBaudRate,
													int nDataBits,
													int nParity,
													int nStopBits,
													int nHardware );
int p_nSetCompCommParms( 	BaudRate  nBaud,
													int nDataBits,
													int nParity,
													int nStop,
													int nFlowControl );
int p_nBeepSystem( int nBeeps );
int p_nInitializeSystem( void );
int p_nSetFiringRate( FiringRate );
int p_nGetSystemInfo( void );
int p_nInitializeAllPorts( const char* pszROMFileName[NB_ROM_FILES] );
int p_nInitializeHandle( int nHandle );
int p_nEnableAllPorts( void );
int p_nEnableOnePorts( int nPortHandle );
int p_nDisablePort( int nPortHandle );
int p_nActivateAllPorts( void );
int p_nLoadTTCFG( char * pszPortID );
int p_nGetHandleForPort( char * pszPortID );
int p_nLoadVirtualSROM( const char * pszFileName, char * pszPhysicalPort, int bPassive );
int p_nFreePortHandles( void );
int p_nGetPortInformation( int nPortHandle );
int p_nStartTracking( void );

int p_nGetBXTransforms( int bReportOOV );
int p_nStopTracking( void );

void p_ErrorMessage( void );

int p_nSendMessage( char * pszCommand, int bAddCRC );
int p_nGetResponse( void );
int p_nGetBinaryResponse( void );
int p_nVerifyResponse( char * pszReply, int bCheckCRC );
int p_nCheckResponse( int nResponse );

int p_nAddCRCToCommand( char * pszCommandString );
int p_nAddCRToCommand( char * pszCommandString );
int p_nBuildCommand( char * pszCommandString, int bAddCRC );

// Structures

typedef struct ComPortState ComPortState;
struct ComPortState {
    char* port;
    int state;
};

typedef struct CCommandHandling CommandHandling;
struct CCommandHandling
{
	int
		m_bLog;															/* log protocol to cout */
	char
		m_szCommand[MAX_COMMAND_MSG],				/* command to send to the system */
		m_szLastReply[MAX_REPLY_MSG];

	SystemInformation
		m_dtSystemInformation;							/* System Information variable - structure */

	HandleInformation
		m_dtHandleInformation[NO_HANDLES];	/* Handle Information varaible - structure */

	Comm32Port
		*pCOMPort;													/* pointer to the Comm32 class */

	int
		m_bClearLogFile,										/* clear log file on intialization */
		m_bDisplayErrorsWhileTracking;			/* display the error while tracking */
	int
		m_nTimeout;													/* timeout value in seconds */

	int
		m_nPortsEnabled;										/* the number of port enable by nEnableAllPorts */
	int 	nSendMes;

};

CommandHandling* pCommandAurora;

/*
 *	MAIN USER ROUTINES DEFINES: START, STOP, UPDATE
 */
 
int polaris_start( void );
void polaris_stop( void );
int polaris_update( void );
