/*
 * Definitions for rgpio.c
 */

int rgpio_export_gpio( unsigned char );
int rgpio_unexport_gpio( unsigned char );
int rgpio_direction_gpio( unsigned char, unsigned char );
int rgpio_read_gpio( unsigned char );
int rgpio_write_gpio( unsigned char, unsigned char );
int rgpio_export_pwm( unsigned char );
int rgpio_unexport_pwm( unsigned char );
int rgpio_set_period_pwm( unsigned char, unsigned int );
int rgpio_set_duty_pwm( unsigned char, unsigned int );
int rgpio_set_enable_pwm( unsigned char, unsigned char );
int rgpio_init( void );
void rgpio_shutdown( void );