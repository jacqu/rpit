/*
 * rpit_js.c
 * 
 * Copyright 2016 Jacques GANGLOFF <jacques.gangloff@unistra.fr>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "rpit_js.h"

/* Flags */

//#define RPITJS_API													// Uncomment to use as an API

/* Defines */

#define RPITJS_MAX_JS						5									// Maximum number of joysticks
#define RPITJS_DEV_PREFIX				"/dev/input/js"		// Prefix for accessing joystick device
#define RPITJS_MAX_DEVNAME_LEN	32								// Max device name length

/* Main structure */

typedef struct
{
	int						fd;
	unsigned char	nb_axes;
	unsigned char	nb_buttons;
	int 					*axis_values;
	char 					*button_values;
}	rpitjs_struct_t;

rpitjs_struct_t rpitjs_struct[RPITJS_MAX_JS] = { { -1, 0, 0, NULL, NULL } };

/* Functions */

/*
 * 
 * Name: rpitjs_init
 * 
 * Parameter(s): 
 * 			device : id of joystick device (e.g. 0->js0) 
 * 
 * Return value:
 * 			0		if no error
 * 			<0 	the error number
 * 
 */
int rpitjs_init( int device )	{
	char rpitjs_devname[RPITJS_MAX_DEVNAME_LEN];

	/* Check device id */
	
	if ( ( device < 0 ) || ( device >= RPITJS_MAX_JS ) )
		return -1;

	/* If device already initialized, skip */
	
	if ( rpitjs_struct[device].fd >= 0 )
		return -2;
		
	/* Open device */
	
	snprintf( rpitjs_devname, 
						RPITJS_MAX_DEVNAME_LEN, 
						"%s%d", 
						RPITJS_DEV_PREFIX, 
						device );
	if ( ( rpitjs_struct[device].fd = open( rpitjs_devname, O_RDONLY ) ) < 0 ) {
		perror("rpitjs trying opening joystick device");
		return -3;
	}
	
	/* Set non blocking read */
	
	if ( fcntl( rpitjs_struct[device].fd, F_SETFL, O_NONBLOCK ) < 0 )	{
		perror("rpitjs trying to set the non blocking flag");
		return -4;
	}
	
	/* Get number of axes and buttons */
	
	if ( ioctl( rpitjs_struct[device].fd, JSIOCGAXES, &rpitjs_struct[device].nb_axes ) < 0 )	{
		perror("rpitjs trying to get number of axes");
		return -5;
	}
	
	if ( ioctl( rpitjs_struct[device].fd, JSIOCGBUTTONS, &rpitjs_struct[device].nb_buttons ) < 0 )	{
		perror("rpitjs trying to get number of buttons");
		return -6;
	}
	
	/* Initialize memory for axis and buttons states */
	
	rpitjs_struct[device].axis_values = calloc( rpitjs_struct[device].nb_axes, sizeof(int) );
	rpitjs_struct[device].button_values = calloc( rpitjs_struct[device].nb_buttons, sizeof(char) );
		
	return 0;
}

/*
 * 
 * Name: rpitjs_terminate
 * 
 * Parameter(s): 
 * 			device : id of joystick device (e.g. 0->js0) 
 * 
 * Return value:
 * 			0		if no error
 * 			<0 	the error number
 * 
 */
int rpitjs_terminate( int device )	{
	
	/* Check device id */
	
	if ( ( device < 0 ) || ( device >= RPITJS_MAX_JS ) )
		return -1;
		
	/* If device already closed, skip */
	
	if ( rpitjs_struct[device].fd < 0 )
		return -2;
	
	/* Close device */
	
	close( rpitjs_struct[device].fd  );
	
	/* Free memory */
	
	free( rpitjs_struct[device].axis_values );
	rpitjs_struct[device].axis_values = NULL;
	free( rpitjs_struct[device].button_values );
	rpitjs_struct[device].button_values = NULL;
	
	/* Reset number of axis and buttons */
	
	rpitjs_struct[device].nb_axes = 0;
	rpitjs_struct[device].nb_buttons = 0;
	
	return 0;
}

/*
 * 
 * Name: rpitjs_read
 * 
 * Parameter(s): 
 * 			device : id of joystick device (e.g. 0->js0) 
 * 
 * Return value:
 * 			0		if no error
 * 			<0 	the error number
 * 
 */
int rpitjs_read( int device )	{
	struct js_event js;
	
	/* Check device id */
	
	if ( ( device < 0 ) || ( device >= RPITJS_MAX_JS ) )
		return -1;
		
	/* If device is not initialized, try to initialize */
	
	if ( rpitjs_struct[device].fd < 0 )
		if ( rpitjs_init( device ) < 0 )
			return -2;
		
	/* Non blocking read: flush read buffer to the lastest event */
	while ( read( rpitjs_struct[device].fd, &js, sizeof( struct js_event ) ) == sizeof( struct js_event ) )	{
		switch( js.type & ~JS_EVENT_INIT ) {
			case JS_EVENT_BUTTON:
				rpitjs_struct[device].button_values[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				rpitjs_struct[device].axis_values[js.number] = js.value;
				break;
			default:
				fprintf( stderr, "rpitjs_read: unknown event.\n" );
		}
	}
	
	return 0;
}

#ifndef RPITJS_API
/*
 * 
 * Name: main
 * 
 * Return value:
 * 			0		if no error
 * 			<0 	the error number
 * 
 */

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

#define RPITJS_DEV_NB				0							// ID of the joystick device to be tested
#define RPITJS_SAMPLING_T		10000					// Sampling period of the read loop in us

int main( void )	{
	int i;
	
	/* Initialize */
	
	rpitjs_init( RPITJS_DEV_NB );
	
	/* Read loop */
	
	while ( !kbhit( ) )	{
		rpitjs_read( RPITJS_DEV_NB );
		
		/* Display joystick state */
		
		printf( "AXES:\t" );
		for ( i = 0; i < rpitjs_struct[RPITJS_DEV_NB].nb_axes; i++ )
			printf( "%d\t", rpitjs_struct[RPITJS_DEV_NB].axis_values[i] );
		printf( "\n" );
		printf( "BUTT:\t" );
		for ( i = 0; i < rpitjs_struct[RPITJS_DEV_NB].nb_buttons; i++ )
			printf( "%d\t", rpitjs_struct[RPITJS_DEV_NB].button_values[i] );
		printf( "\n" );
		
		usleep( RPITJS_SAMPLING_T );
	}
	
	/* Cleanup */
	
	rpitjs_terminate( RPITJS_DEV_NB );
	
	return 0;
}
#endif
