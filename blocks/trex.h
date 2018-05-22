/******************************************************************************
 *	Pololu TReX power amplifier API definitions
 *	JG, may 2018
 *****************************************************************************/
 
int trex_init_port( char* );
void trex_release_port( void );
int trex_check_presence( unsigned char );
int trex_print_config( unsigned char );
int trex_output( unsigned char, signed char, signed char );
int trex_get_status( unsigned char );
int trex_get_uart_error( unsigned char );
int trex_set_config( unsigned char, unsigned char, unsigned char );
