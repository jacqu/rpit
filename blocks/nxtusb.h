/* Common definitions for client and server */

#define NXT_NB_MOTORS               3
#define NXT_NB_SENSORS              4
#define NXT_TIMEOUT_MS							10
#define NXT_MAX_CONTROL							100.0

struct nxt_control_struct	{
	unsigned char		init_encoders;
	signed char 		motor_power[NXT_NB_MOTORS];
};

struct nxt_measurement_struct	{
	signed int 			motor_angle[NXT_NB_MOTORS];
	unsigned int		input_ADC[NXT_NB_SENSORS];
	unsigned int		battery_voltage;
};
