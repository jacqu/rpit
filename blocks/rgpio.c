/*
 * RGPIO : access Raspberry Pi gpio ports with sysfs
 * 
 * To allow user gpio access: sudo usermod -a -G gpio $USER
 *
 * JG, may 2018
 */
 
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rgpio.h"

#define RGPIO_IN            0
#define RGPIO_OUT           1
 
#define RGPIO_LOW           0
#define RGPIO_HIGH          1

#define RGPIO_BUFFER_MAX		50

#define RGPIO_UDEV_DELAY		100000

/* Definition of the output GPIOs */

#define RGPIO_NB_OUTPUT			7
#define OUT_BCM17_PIN11			17
#define OUT_BCM27_PIN13			27
#define OUT_BCM22_PIN15			22
#define OUT_BCM5_PIN29			5
#define OUT_BCM6_PIN31			6
#define OUT_BCM19_PIN35			19
#define OUT_BCM26_PIN37			26

/* Definition of the input GPIOs */

#define RGPIO_NB_INPUT			5
#define IN_BCM18_PIN12			18
#define IN_BCM23_PIN16			23
#define IN_BCM24_PIN18			24
#define IN_BCM25_PIN22			25
#define IN_BCM16_PIN36			16

/* Definition of the PWM GPIOS */

#define RGPIO_NB_PWM        2
#define PWM_BCM12_PIN32			12
#define PWM_BCM13_PIN33			13

unsigned int rgpio_output_port[RGPIO_NB_OUTPUT] =	{
	OUT_BCM17_PIN11,
	OUT_BCM27_PIN13,
	OUT_BCM22_PIN15,
	OUT_BCM5_PIN29,
	OUT_BCM6_PIN31,
	OUT_BCM19_PIN35,
	OUT_BCM26_PIN37
};

unsigned int rgpio_input_port[RGPIO_NB_INPUT] =	{
	IN_BCM18_PIN12,
	IN_BCM23_PIN16,
	IN_BCM24_PIN18,
	IN_BCM25_PIN22,
	IN_BCM16_PIN36
};

int rgpio_export_gpio( unsigned char pin )	{
	char buffer[RGPIO_BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
 
	fd = open( "/sys/class/gpio/export", O_WRONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_export: failed to open export for writing!\n" );
		return -1;
	}
 
	bytes_written = snprintf( buffer, RGPIO_BUFFER_MAX, "%d", pin );
	write( fd, buffer, bytes_written );
	close( fd );
	return 0;
}
 
int rgpio_unexport_gpio( unsigned char pin )	{
	char buffer[RGPIO_BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
 
	fd = open( "/sys/class/gpio/unexport", O_WRONLY );
	if ( -1 == fd ) {
		fprintf(stderr, "rgpio_unexport: failed to open unexport for writing!\n");
		return -1;
	}
 
	bytes_written = snprintf(buffer, RGPIO_BUFFER_MAX, "%d", pin );
	write( fd, buffer, bytes_written );
	close( fd );
	return 0;
}
 
int rgpio_direction_gpio( unsigned char pin, unsigned char dir )
{
	static const char 	s_directions_str[]  = "in\0out";
	char 				path[RGPIO_BUFFER_MAX];
	int 				fd;
 
	snprintf( path, RGPIO_BUFFER_MAX, "/sys/class/gpio/gpio%d/direction", pin );
	fd = open( path, O_WRONLY );
	if ( -1 == fd ) {
		fprintf(stderr, "rgpio_direction: failed to open gpio direction for writing!\n");
		return -1;
	}
 
	if ( -1 == write( fd, &s_directions_str[RGPIO_IN == dir ? 0 : 3], RGPIO_IN == dir ? 2 : 3 ) ) {
		fprintf( stderr, "rgpio_direction: failed to set direction!\n" );
		close( fd );
		return -2;
	}
 
	close( fd );
	return 0;
}

int rgpio_read_gpio( unsigned char pin )	{
	char path[RGPIO_BUFFER_MAX];
	char value_str[RGPIO_BUFFER_MAX];
	int fd;
 
	snprintf( path, RGPIO_BUFFER_MAX, "/sys/class/gpio/gpio%d/value", pin );
	fd = open( path, O_RDONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_read: failed to open gpio value for reading!\n" );
		return -1;
	}
 
	if ( -1 == read( fd, value_str, RGPIO_BUFFER_MAX ) ) {
		fprintf( stderr, "rgpio_read: failed to read value!\n" );
		close( fd );
		return -2;
	}
 
	close( fd );
 
	return( atoi( value_str ) );
}
 
int rgpio_write_gpio( unsigned char pin, unsigned char value )	{
	static const char s_values_str[] = "01";
 
	char 	path[RGPIO_BUFFER_MAX];
	int 	fd;
 
	snprintf( path, RGPIO_BUFFER_MAX, "/sys/class/gpio/gpio%d/value", pin );
	fd = open( path, O_WRONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_write: failed to open gpio value for writing!\n" );
		return -1;
	}
 
	if ( 1 != write( fd, &s_values_str[RGPIO_LOW == value ? 0 : 1], 1 ) ) {
		fprintf(stderr, "rgpio_write: failed to write value!\n");
		close( fd );
		return -2;
	}
 
	close( fd );
	return 0;
}

int rgpio_export_pwm( unsigned char pwm )	{
	char buffer[RGPIO_BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
	
	if ( pwm >= RGPIO_NB_PWM )	{
		fprintf( stderr, "rgpio_export_pwm: number of pwm devices exceeded!\n" );
		return -1;
	}
	
	fd = open( "/sys/class/pwm/pwmchip0/export", O_WRONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_export_pwm: failed to open export for writing!\n" );
		return -2;
	}
 
	bytes_written = snprintf( buffer, RGPIO_BUFFER_MAX, "%d", pwm );
	write( fd, buffer, bytes_written );
	close( fd );
	return 0;
}

int rgpio_unexport_pwm( unsigned char pwm )	{
	char buffer[RGPIO_BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
	
	if ( pwm >= RGPIO_NB_PWM )	{
		fprintf( stderr, "rgpio_unexport_pwm: number of pwm devices exceeded!\n" );
		return -1;
	}
	
	fd = open( "/sys/class/pwm/pwmchip0/unexport", O_WRONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_unexport_pwm: failed to open export for writing!\n" );
		return -2;
	}
 
	bytes_written = snprintf( buffer, RGPIO_BUFFER_MAX, "%d", pwm );
	write( fd, buffer, bytes_written );
	close( fd );
	return 0;
}

int rgpio_set_period_pwm( unsigned char pwm, unsigned int period )	{
	char buffer[RGPIO_BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
	
	if ( pwm >= RGPIO_NB_PWM )	{
		fprintf( stderr, "rgpio_set_period_pwm: number of pwm devices exceeded!\n" );
		return -1;
	}
	
	snprintf( buffer, RGPIO_BUFFER_MAX, "/sys/class/pwm/pwmchip0/pwm%d/period", pwm );
	
	fd = open( buffer, O_WRONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_set_period_pwm: failed to open period for writing!\n" );
		return -2;
	}
	
	bytes_written = snprintf( buffer, RGPIO_BUFFER_MAX, "%d", period );
	write( fd, buffer, bytes_written );
	close( fd );
	
	return 0;
}

int rgpio_set_duty_pwm( unsigned char pwm, unsigned int duty )	{
	char buffer[RGPIO_BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
	
	if ( pwm >= RGPIO_NB_PWM )	{
		fprintf( stderr, "rgpio_set_duty_pwm: number of pwm devices exceeded!\n" );
		return -1;
	}
	
	snprintf( buffer, RGPIO_BUFFER_MAX, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pwm );
	
	fd = open( buffer, O_WRONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_set_duty_pwm: failed to open duty_cycle for writing!\n" );
		return -2;
	}
	
	bytes_written = snprintf( buffer, RGPIO_BUFFER_MAX, "%d", duty );
	write( fd, buffer, bytes_written );
	close( fd );
	
	return 0;
}

int rgpio_set_enable_pwm( unsigned char pwm, unsigned char value )	{
	char buffer[RGPIO_BUFFER_MAX];
	ssize_t bytes_written;
	int fd;
	
	if ( pwm >= RGPIO_NB_PWM )	{
		fprintf( stderr, "rgpio_set_enable_pwm: number of pwm devices exceeded!\n" );
		return -1;
	}
	
	snprintf( buffer, RGPIO_BUFFER_MAX, "/sys/class/pwm/pwmchip0/pwm%d/enable", pwm );
	
	fd = open( buffer, O_WRONLY );
	if ( -1 == fd ) {
		fprintf( stderr, "rgpio_set_enable_pwm: failed to open enable for writing!\n" );
		return -2;
	}
	
	bytes_written = snprintf( buffer, RGPIO_BUFFER_MAX, "%d", value );
	write( fd, buffer, bytes_written );
	close( fd );
	
	return 0;
}

int rgpio_init( void )	{
	int		i, ret;
	
	/* Initialize outputs */
	
	for ( i = 0; i < RGPIO_NB_OUTPUT; i++ )	{
		ret = rgpio_export_gpio( rgpio_output_port[i] );
		usleep( RGPIO_UDEV_DELAY );
		ret += rgpio_direction_gpio( rgpio_output_port[i], RGPIO_OUT );
		if ( ret )	{
			fprintf( stderr, "rgpio_init: failed to initialize output %d!\n", rgpio_output_port[i] );
			exit( -1 );
		}
	}
	
	/* Initialize inputs */
	
	for ( i = 0; i < RGPIO_NB_INPUT; i++ )	{
		ret = rgpio_export_gpio( rgpio_input_port[i] );
		usleep( RGPIO_UDEV_DELAY );
		ret += rgpio_direction_gpio( rgpio_input_port[i], RGPIO_IN );
		if ( ret )	{
			fprintf( stderr, "rgpio_init: failed to initialize input %d!\n", rgpio_input_port[i] );
			exit( -1 );
		}
	}
	
	return 0;
}

void rgpio_shutdown( void )	{
	int		i;
	
	/* Shutdown outputs */
	
	for ( i = 0; i < RGPIO_NB_OUTPUT; i++ )	{
		rgpio_write_gpio( rgpio_output_port[i], RGPIO_LOW );
		rgpio_unexport_gpio( rgpio_output_port[i] );
	}
	
	/* Shutdown inputs */
	
	for ( i = 0; i < RGPIO_NB_INPUT; i++ )	{
		rgpio_unexport_gpio( rgpio_input_port[i] );
	}
}