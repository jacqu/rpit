/******************************************************************************
 *	Pololu TReX power amplifier API
 *	JG, may 2018
 *	To access serial port as user : sudo usermod -a -G dialout $USER 
 *	(logout needed)
 * 	To compile : gcc -Wall -o trex trex.c
 *****************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include "trex.h"

//#define TREX_LIB										// Flag to activate/deactivate main

struct trex_config_struct	{
	unsigned char 	address;
	const char*		definition;
};

#define TREX_BAUDRATE 			B115200					// Serial baudrate
#define TREX_MODEMDEVICE 		"/dev/ttyUSB0"			// Serial port
#define TREX_READ_TIMEOUT		5						// Tenth of second
#define TREX_SIGNATURE_SIZE		9						// Size of TReX signature
#define TREX_WRITE_SIZE			10						// Max write buffer size
#define TREX_SIGNATURE			"TReXJr"				// Prefix of the signature
#define TREX_DEFAULT_ID			0						// Default device ID
#define TREX_MAX_DUTY_CYCLE		127						// Max output value
#define TREX_MIN_DUTY_CYCLE		0						// Min output value

#define TREX_EXT_PROTOCOL		0x80					// Switch to extended protocol
#define TREX_GET_SIGNATURE		0x81					// Get signature
#define TREX_GET_STATUS			0x84					// Get status
#define TREX_GET_UART_ERROR		0x85					// Get UART error
#define TREX_GET_CONFIG_PARAM	0x9F					// Get configuration parameter
#define TREX_SET_CONFIG_PARAM	0xAF					// Set configuration parameter
#define TREX_SET_MOTOR_1_REV	0xC1					// Motor 1 reverse
#define TREX_SET_MOTOR_2_REV	0xC9					// Motor 2 reverse
#define TREX_SET_MOTOR_1_FWD	0xC2					// Motor 1 forward
#define TREX_SET_MOTOR_2_FWD	0xCA					// Motor 2 forward
#define TREX_SET_MOTOR_1_2		0xD0					// Set motor 1 and 2

#define TREX_MAX_CONFIG_ADDR	0x7F					// Highest configuration address
#define TREX_MAX_CONFIG_VAL		0x7F					// Highest configuration value
#define TREX_FORMAT_BYTE_1		0x55
#define TREX_FORMAT_BYTE_2		0x2A

int								trex_fd = -1;			// Serial port file descriptor
struct termios 					trex_oldtio;			// Backup of old configuration
struct trex_config_struct trex_config[] =	{
	{ 0x00, "Device Number" },
	{ 0x01, "Required Channels" },
	{ 0x02, "Ignored Channels" },
	{ 0x03, "Reversed Channels" },
	{ 0x04, "Parabolic Channels" },
	{ 0x05, "Motor 1 Deadband Brake PWM" },
	{ 0x06, "Motor 2 Deadband Brake PWM" },
	{ 0x07, "Serial Timeout" },
	{ 0x08, "UART-Error Shutdown" },
	{ 0x09, "Motor 1 PWM Prescaler" },
	{ 0x0A, "Motor 2 PWM Prescaler" },
	{ 0x0B, "Motor 1 PWM Maximum" },
	{ 0x0C, "Motor 2 PWM Maximum" },
	{ 0x0D, "Auxiliary Motor PWM Maximum" },
	{ 0x0E, "Motor 1 Acceleration" },
	{ 0x0F, "Motor 2 Acceleration" },
	{ 0x10, "Auxiliary Motor Acceleration" },
	{ 0x11, "Motor 1 Brake Duration" },
	{ 0x12, "Motor 2 Brake Duration" },
	{ 0x13, "Motor 1 Current Limit" },
	{ 0x14, "Motor 2 Current Limit" },
	{ 0x15, "Motor 1 Current Limit Proportionality Constant P" },
	{ 0x16, "Motor 2 Current Limit Proportionality Constant P" },
	{ 0x17, "Enable UART Response Delay (Version 1.2+ only)" },
	{ 0x7B, "Motor Mode" },
	{ 0x7C, "Channel Input Source" },
	{ 0x7D, "CRC-7 Polynomial" },
	{ 0x7E, "UART Settings" },
	{ 0x7F, "Reset All Parameters to Factory Defaults" }
};


/******************************************************************************
 *	trex_init_port : initialize serial port
 *****************************************************************************/
int trex_init_port( char *portname )	{
	struct termios 	newtio;

	/* Open device */
	trex_fd = open( portname, O_RDWR | O_NOCTTY ); 
	if ( trex_fd < 0 )  { 
		perror( TREX_MODEMDEVICE ); 
		return -1; 
	}

	/* Save current port settings */
	tcgetattr( trex_fd, &trex_oldtio ); 

	/* Define new settings */
	bzero( &newtio, sizeof(newtio) );
	cfmakeraw( &newtio );
	newtio.c_cflag = TREX_BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;

	/* Set input mode (non-canonical, no echo,...) */
	newtio.c_lflag = 0;

	/* Inter-character timer  */
	newtio.c_cc[VTIME] = TREX_READ_TIMEOUT;   
	newtio.c_cc[VMIN] = 0;

	/* Apply the settings */
	tcflush( trex_fd, TCIFLUSH );
	tcsetattr( trex_fd, TCSANOW, &newtio );

	return 0;
}

/******************************************************************************
 *	trex_init_port : release serial port
 *****************************************************************************/
void trex_release_port( void )	{

	/* Restore initial settings */
	tcsetattr( trex_fd, TCSANOW, &trex_oldtio );
	close( trex_fd );
	trex_fd = -1;
}

/******************************************************************************
 *	trex_check_presence : send a get signature command and check for the result
 *	0 : no TReX detected
 *	1 : TReX detected
 *****************************************************************************/
int trex_check_presence( unsigned char id )	{
	char 			buf[TREX_SIGNATURE_SIZE+1];
	unsigned char 	cmd[TREX_WRITE_SIZE];
	int				res;
	
	if ( trex_fd == -1 )
		return 0;
	
	/* Set ext protocol cmd */
	cmd[0] = TREX_EXT_PROTOCOL;
	
	/* Set ID */
	cmd[1] = id;

	/* Set signature cmd */
	cmd[2] = TREX_GET_SIGNATURE & 0x7F;
	
	/* Write command */
	res = write( trex_fd, cmd, 3 );
	if ( res < 0 )  { 
		perror( "trex_check_presence write command" ); 
		return 0; 
	}

	/* Read signature */
	res = read( trex_fd, buf, TREX_SIGNATURE_SIZE );
	if ( res != TREX_SIGNATURE_SIZE )  { 
		perror( "trex_check_presence read signature" ); 
		return 0; 
	}
	else	{
		buf[res] = 0;
		printf( "*** TReX signature: %s\n", buf );
		if ( strstr( buf, TREX_SIGNATURE ) )
			return 1;
		else
			return 0;
	}
}

/******************************************************************************
 *	trex_print_config : display all configuration bytes
 *	0 : no error
 *****************************************************************************/
int trex_print_config( unsigned char id )	{
	unsigned char 	cmd[TREX_WRITE_SIZE];;
	int				res, i = 0;
	unsigned char 	cfg;
	
	if ( trex_fd == -1 )
		return -1;
	
	while ( trex_config[i].address != TREX_MAX_CONFIG_ADDR )	{
		
		/* Set ext protocol cmd */
		cmd[0] = TREX_EXT_PROTOCOL;
	
		/* Set ID */
		cmd[1] = id;
		
		/* Set get status cmd */
		cmd[2] = TREX_GET_CONFIG_PARAM & 0x7F;
		
		/* Set config address */
		cmd[3] = trex_config[i].address;
		
		/* Send command */
		res = write( trex_fd, cmd, 4 );
		if ( res < 0 )  { 
			perror( "trex_print_config write command" ); 
			return -2; 
		}
		
		/* Read config */
		res = read( trex_fd, &cfg, 1 );
		if ( res != 1 )  { 
			perror( "trex_print_config read config" ); 
			return -3; 
		}
		else	
			printf ( 	"> %s (0x%x): 0x%x\n", 
						trex_config[i].definition, 
						trex_config[i].address, 
						cfg );
		
		i++;
	}
	
	return 0;
 }
 
/******************************************************************************
 *	trex_output : output control
 *	0 : no error
 *****************************************************************************/
int trex_output( unsigned char id, signed char out1, signed char out2 )	{
	unsigned char 	cmd[TREX_WRITE_SIZE];
	int				res;

	if ( trex_fd == -1 )
		return -1;
	
	/* Set ext protocol cmd */
	cmd[0] = TREX_EXT_PROTOCOL;
	
	/* Set ID */
	cmd[1] = id;

	/* Set cmd for motor 1 and 2 */
	cmd[2] = ( TREX_SET_MOTOR_1_2 | ( ( (out2 < 0) ? 1 : 2 ) * 4 ) | ( (out1 < 0) ? 1 : 2 ) ) & 0x7F;

	/* Set speed for motor 1 */
	cmd[3] = abs( out1 );
	
	/* Set speed for motor 2 */
	cmd[4] = abs( out2 );
	
	/* Send command */
	res = write( trex_fd, cmd, 5 );
	if ( res < 0 )  { 
		perror( "trex_output write command" ); 
		return -2; 
	}

	return 0;
}

/******************************************************************************
 *	trex_get_status : get status byte
 *	0 : no error
 *****************************************************************************/
int trex_get_status( unsigned char id )	{
	unsigned char 	cmd[TREX_WRITE_SIZE];;
	int				res;
	unsigned char	status;
	
	if ( trex_fd == -1 )
		return -1;
		
	/* Set ext protocol cmd */
	cmd[0] = TREX_EXT_PROTOCOL;
	
	/* Set ID */
	cmd[1] = id;
	
	/* Set get status cmd */
	cmd[2] = TREX_GET_STATUS & 0x7F;
	
	/* Send command */
	res = write( trex_fd, cmd, 3 );
	if ( res < 0 )  { 
		perror( "trex_get_status write command" ); 
		return -2; 
	}

	/* Read status */
	res = read( trex_fd, &status, 1 );
	if ( res != 1 )  { 
		perror( "trex_get_status read status" ); 
		return -3; 
	}
	else	
		return status;
}

/******************************************************************************
 *	trex_get_uart_error : get uart error byte
 *	0 : no error
 *****************************************************************************/
int trex_get_uart_error( unsigned char id )	{
	unsigned char 	cmd[TREX_WRITE_SIZE];
	int				res;
	unsigned char	status;
	
	if ( trex_fd == -1 )
		return -1;
	
	/* Set ext protocol cmd */
	cmd[0] = TREX_EXT_PROTOCOL;
	
	/* Set ID */
	cmd[1] = id;
	
	/* Set get status cmd */
	cmd[2] = TREX_GET_UART_ERROR & 0x7F;
	
	/* Send command */
	res = write( trex_fd, cmd, 3 );
	if ( res < 0 )  { 
		perror( "trex_get_uart_error write command" ); 
		return -2; 
	}

	/* Read error code */
	res = read( trex_fd, &status, 1 );
	if ( res != 1 )  { 
		perror( "trex_get_uart_error read UART error" ); 
		return -3; 
	}
	else	
		return status;
}

/******************************************************************************
 *	trex_set_config : set config bypte
 *	0 : no error
 *****************************************************************************/
int trex_set_config( unsigned char id, unsigned char addr, unsigned char val )	{
	unsigned char 	cmd[TREX_WRITE_SIZE];
	int				res;
	unsigned char	status;
	
	if ( trex_fd == -1 )
		return -1;
	
	/* Set ext protocol cmd */
	cmd[0] = TREX_EXT_PROTOCOL;
	
	/* Set ID */
	cmd[1] = id;
	
	/* Set config cmd */
	cmd[2] = TREX_SET_CONFIG_PARAM & 0x7F;
	
	/* Set config address */
	cmd[3] = addr;
	
	/* Set config value */
	cmd[4] = val;
	
	/* Set format byte 1 and 2 */
	cmd[5] = TREX_FORMAT_BYTE_1;
	cmd[6] = TREX_FORMAT_BYTE_2;
	
	/* Send command */
	res = write( trex_fd, cmd, 7 );
	if ( res < 0 )  { 
		perror( "trex_set_config write command" ); 
		return -2; 
	}

	/* Read error code */
	res = read( trex_fd, &status, 1 );
	if ( res != 1 )  { 
		perror( "trex_set_config read UART error" ); 
		return -3; 
	}
	else
	{
		switch( status )	{
			case 0:
				printf( "trex_set_config: parameter 0x%x is now set to 0x%x.\n", addr, val );
				break;
			case 1:
				printf( "trex_set_config: bad address.\n" );
				break;
			case 2:
				printf( "trex_set_config: bad value.\n" );
				break;
			case 3:
				printf( "trex_set_config: TReX Jr isn't in serial mode.\n" );
				break;
			default:
				printf( "trex_set_config: unknown error %d.\n", status );
		}
		
		return status;
	}
}

#ifndef TREX_LIB
int main( int argc, char *argv[] )	{
	int ret;
	int i = -TREX_MAX_DUTY_CYCLE, slope = 1;
	
	if ( argc <= 1 )
		goto display_help;
	
	/* Help command */
	if ( !strcmp( argv[1], "help" ) )
		goto display_help;
	
	/* Test command */
	if ( !strcmp( argv[1], "test" ) )	{
		if ( trex_init_port( TREX_MODEMDEVICE ) )	{
			printf( "Error while initializing TReX port.\n" );
			exit( -1 );
		}

		if ( trex_check_presence( TREX_DEFAULT_ID ) )	{
			printf( "TReX detected.\n" );
			
			/* Display config */
			
			trex_print_config( TREX_DEFAULT_ID );
			
			printf( "* Cycling through all duty cycles from -127 through 127 to -127.\n" );
			printf( "* [CTRL-C] to exit program.\n" ); 
			
			while ( 1 )	{
				/* Set duty cycle to max on output 1,2 then min on output 1,2 */
				
				trex_output( TREX_DEFAULT_ID, i, -i );
				if ( ( ret = trex_get_status( TREX_DEFAULT_ID ) ) )	{
					printf( "Status error: %d\n", ret );
					if ( ret == 1 )
						printf ( "UART error: %d\n", trex_get_uart_error( TREX_DEFAULT_ID ) );
				}
				
				i += slope;
				if ( ( i >= TREX_MAX_DUTY_CYCLE ) || (i <= -TREX_MAX_DUTY_CYCLE ) )
					slope *= -1;
			}
			
		}
		else	{
			printf( "TReX not detected.\n" );
		}
		
		trex_release_port( );
		
		return 0;
	}
	
	/* Blink command */
	if ( !strcmp( argv[1], "blink" ) )	{
		if ( trex_init_port( TREX_MODEMDEVICE ) )	{
			printf( "Error while initializing TReX port.\n" );
			exit( -1 );
		}

		if ( trex_check_presence( TREX_DEFAULT_ID ) )	{
			printf( "TReX detected.\n" );
			
			/* Display config */
			
			trex_print_config( TREX_DEFAULT_ID );
			
			printf( "* Alternate duty cycle periodically from -%d to %d.\n",
					TREX_MAX_DUTY_CYCLE, TREX_MAX_DUTY_CYCLE );
			printf( "* [CTRL-C] to exit program.\n" ); 
			
			while ( 1 )	{
				/* Set duty cycle to max on output 1,2 then min on output 1,2 */
				
				trex_output( TREX_DEFAULT_ID, TREX_MAX_DUTY_CYCLE, -TREX_MAX_DUTY_CYCLE );
				trex_output( TREX_DEFAULT_ID, -TREX_MAX_DUTY_CYCLE, TREX_MAX_DUTY_CYCLE );
			}
			
		}
		else	{
			printf( "TReX not detected.\n" );
		}
		
		trex_release_port( );
		
		return 0;
	}
	
	/* Config command */
	if ( !strcmp( argv[1], "config" ) )	{
		if ( argc != 4 )	{
			printf( "Arguments missing.\n" );
			goto display_help;
		}
		if ( ( atoi( argv[2] ) < 0 ) || ( atoi( argv[2] ) > TREX_MAX_CONFIG_ADDR ) )	{
			printf( "Confihuration address is out of range.\n" );
			exit( -1 );
		}
		if ( ( atoi( argv[3] ) < 0 ) || ( atoi( argv[3] ) > TREX_MAX_CONFIG_VAL ) )	{
			printf( "Configuration value is out of range.\n" );
			exit( -1 );
		}
		if ( trex_init_port( TREX_MODEMDEVICE ) )	{
			printf( "Error while initializing TReX port.\n" );
			exit( -1 );
		}
		if ( trex_check_presence( TREX_DEFAULT_ID ) )	{
			printf( "TReX detected.\n" );
			trex_set_config( 	TREX_DEFAULT_ID, 
								(unsigned char)strtol(argv[2], NULL, 0), 
								(unsigned char)strtol(argv[3], NULL, 0) );
		}
		else
		{
			printf( "TReX not detected.\n" );
		}
			
		trex_release_port( );
		
		return 0;
	}
	
	/* Default action */	
	display_help:
	printf( "* trex help: print this help.\n" );
	printf( "* trex test: print config and test the outputs.\n" );
	printf( "* trex blink: print config and make the outputs blink at max rate.\n" );
	printf( "* trex config addr val: wirte config value val at address addr.\n" ); 
	
	return 0;
}
#endif
