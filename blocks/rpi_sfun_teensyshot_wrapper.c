
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* External function declarations */
#ifndef MATLAB_MEX_FILE
#include "host.c"

#define ESCPID_WRAPPER_MAX_RPM_R    10000.0   	// Max x10 RPM
#define ESCPID_WRAPPER_RESET_WAIT	2			// Waiting time for a teensy reset to complete (s)
#endif
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void rpi_sfun_teensyshot_Start_wrapper(const real_T *rpi_Ts, const int_T p_width0,
			const real_T *P1, const int_T p_width1,
			const real_T *I1, const int_T p_width2,
			const real_T *D1, const int_T p_width3,
			const real_T *f1, const int_T p_width4,
			const real_T *P2, const int_T p_width5,
			const real_T *I2, const int_T p_width6,
			const real_T *D2, const int_T p_width7,
			const real_T *f2, const int_T p_width8,
			const real_T *P3, const int_T p_width9,
			const real_T *I3, const int_T p_width10,
			const real_T *D3, const int_T p_width11,
			const real_T *f3, const int_T p_width12,
			const real_T *P4, const int_T p_width13,
			const real_T *I4, const int_T p_width14,
			const real_T *D4, const int_T p_width15,
			const real_T *f4, const int_T p_width16,
			const real_T *P5, const int_T p_width17,
			const real_T *I5, const int_T p_width18,
			const real_T *D5, const int_T p_width19,
			const real_T *f5, const int_T p_width20,
			const real_T *P6, const int_T p_width21,
			const real_T *I6, const int_T p_width22,
			const real_T *D6, const int_T p_width23,
			const real_T *f6, const int_T p_width24,
			const uint32_T *Port, const int_T p_width25)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
	* Custom Start code goes here.
	*/
	#ifndef MATLAB_MEX_FILE
	int                 ret;
	int16_T             RPM_r[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_P[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_I[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_D[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_f[ESCPID_MAX_ESC] = { 0 };
	ESCPIDcomm_struct_t *comm;

	// Check parameter dimension
	if ( 	p_width0 *
				p_width1 *
				p_width2 *
				p_width3 *
				p_width4 *
				p_width5 *
				p_width6 *
				p_width7 *
				p_width8 *
				p_width9 *
				p_width10 *
				p_width11 *
				p_width12 *
				p_width13 *
				p_width14 *
				p_width15 *
				p_width16 *
				p_width17 *
				p_width18 *
				p_width19 *
				p_width20 *
				p_width21 *
				p_width22 *
				p_width23 *
				p_width24 *
				p_width25 != 1 )
		return;
	
	// Check parameter value
	if (
		( *rpi_Ts < 0 ) ||
		( *P1 < 0 ) ||
		( *I1 < 0 ) ||
		( *D1 < 0 ) ||
		( *f1 < 0 ) ||
		( *P2 < 0 ) ||
		( *I2 < 0 ) ||
		( *D2 < 0 ) ||
		( *f2 < 0 ) ||
		( *P3 < 0 ) ||
		( *I3 < 0 ) ||
		( *D3 < 0 ) ||
		( *f3 < 0 ) ||
		( *P4 < 0 ) ||
		( *I4 < 0 ) ||
		( *D4 < 0 ) ||
		( *f4 < 0 ) ||
		( *P5 < 0 ) ||
		( *I5 < 0 ) ||
		( *D5 < 0 ) ||
		( *f5 < 0 ) ||
		( *P6 < 0 ) ||
		( *I6 < 0 ) ||
		( *D6 < 0 ) ||
		( *f6 < 0 ) ||
		( *Port == 0 )
		)
		return;

	// Open serial port
	Host_init_port( *Port );

	// Configure PID params of ESC #0 with reset command
	PID_P[0] = ESCPID_RESET_GAIN;
	PID_I[0] = ESCPID_RESET_GAIN;
	PID_D[0] = ESCPID_RESET_GAIN;
	PID_f[0] = ESCPID_RESET_GAIN;

	// Send command to teensy
	Host_comm_update( 	*Port,
						RPM_r,
						PID_P,
						PID_I,
						PID_D,
						PID_f,
						&comm );
	
	// Wait for the reset command to complete
	sleep( ESCPID_WRAPPER_RESET_WAIT );

	#endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void rpi_sfun_teensyshot_Outputs_wrapper(const real_T *u1,
			const real_T *u2,
			const real_T *u3,
			const real_T *u4,
			const real_T *u5,
			const real_T *u6,
			const real_T *en,
			real_T *y1,
			real_T *y2,
			real_T *y3,
			real_T *y4,
			real_T *y5,
			real_T *y6,
			const real_T *rpi_Ts, const int_T p_width0,
			const real_T *P1, const int_T p_width1,
			const real_T *I1, const int_T p_width2,
			const real_T *D1, const int_T p_width3,
			const real_T *f1, const int_T p_width4,
			const real_T *P2, const int_T p_width5,
			const real_T *I2, const int_T p_width6,
			const real_T *D2, const int_T p_width7,
			const real_T *f2, const int_T p_width8,
			const real_T *P3, const int_T p_width9,
			const real_T *I3, const int_T p_width10,
			const real_T *D3, const int_T p_width11,
			const real_T *f3, const int_T p_width12,
			const real_T *P4, const int_T p_width13,
			const real_T *I4, const int_T p_width14,
			const real_T *D4, const int_T p_width15,
			const real_T *f4, const int_T p_width16,
			const real_T *P5, const int_T p_width17,
			const real_T *I5, const int_T p_width18,
			const real_T *D5, const int_T p_width19,
			const real_T *f5, const int_T p_width20,
			const real_T *P6, const int_T p_width21,
			const real_T *I6, const int_T p_width22,
			const real_T *D6, const int_T p_width23,
			const real_T *f6, const int_T p_width24,
			const uint32_T *Port, const int_T p_width25)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
		y0[0] = u0[0]; 
	For complex signals use: y0[0].re = u0[0].re; 
		y0[0].im = u0[0].im;
		y1[0].re = u1[0].re;
		y1[0].im = u1[0].im;
	*/
	#ifdef MATLAB_MEX_FILE
	y1[0] = 0.0;
	y1[1] = 0.0;
	y1[2] = 0.0;
	y1[3] = 0.0;
	y1[4] = 0.0;
	y1[5] = 0.0;

	y2[0] = 0.0;
	y2[1] = 0.0;
	y2[2] = 0.0;
	y2[3] = 0.0;
	y2[4] = 0.0;
	y2[5] = 0.0;

	y3[0] = 0.0;
	y3[1] = 0.0;
	y3[2] = 0.0;
	y3[3] = 0.0;
	y3[4] = 0.0;
	y3[5] = 0.0;

	y4[0] = 0.0;
	y4[1] = 0.0;
	y4[2] = 0.0;
	y4[3] = 0.0;
	y4[4] = 0.0;
	y4[5] = 0.0;

	y5[0] = 0.0;
	y5[1] = 0.0;
	y5[2] = 0.0;
	y5[3] = 0.0;
	y5[4] = 0.0;
	y5[5] = 0.0;

	y6[0] = 0.0;
	y6[1] = 0.0;
	y6[2] = 0.0;
	y6[3] = 0.0;
	y6[4] = 0.0;
	y6[5] = 0.0;
	#else
	int                 i, ret;
	int16_T             RPM_r[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_P[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_I[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_D[ESCPID_MAX_ESC] = { 0 };
	uint16_T            PID_f[ESCPID_MAX_ESC] = { 0 };
	ESCPIDcomm_struct_t *comm;

	// Check parameter dimension
	if ( 	p_width0 *
				p_width1 *
				p_width2 *
				p_width3 *
				p_width4 *
				p_width5 *
				p_width6 *
				p_width7 *
				p_width8 *
				p_width9 *
				p_width10 *
				p_width11 *
				p_width12 *
				p_width13 *
				p_width14 *
				p_width15 *
				p_width16 *
				p_width17 *
				p_width18 *
				p_width19 *
				p_width20 *
				p_width21 *
				p_width22 *
				p_width23 *
				p_width24 *
				p_width25 != 1 )
		return;
	
	// Check parameter value
	if (
		( *rpi_Ts < 0 ) ||
		( *P1 < 0 ) ||
		( *I1 < 0 ) ||
		( *D1 < 0 ) ||
		( *f1 < 0 ) ||
		( *P2 < 0 ) ||
		( *I2 < 0 ) ||
		( *D2 < 0 ) ||
		( *f2 < 0 ) ||
		( *P3 < 0 ) ||
		( *I3 < 0 ) ||
		( *D3 < 0 ) ||
		( *f3 < 0 ) ||
		( *P4 < 0 ) ||
		( *I4 < 0 ) ||
		( *D4 < 0 ) ||
		( *f4 < 0 ) ||
		( *P5 < 0 ) ||
		( *I5 < 0 ) ||
		( *D5 < 0 ) ||
		( *f5 < 0 ) ||
		( *P6 < 0 ) ||
		( *I6 < 0 ) ||
		( *D6 < 0 ) ||
		( *f6 < 0 ) ||
		( *Port == 0 )
		)
		return;

	// Check for errors
	if ( ( p_width0 > 1 ) || ( *rpi_Ts < 0.002 ) )	{
		y1[0] = 0.0;
		y1[1] = 0.0;
		y1[2] = 0.0;
		y1[3] = 0.0;
		y1[4] = 0.0;
		y1[5] = 0.0;

		y2[0] = 0.0;
		y2[1] = 0.0;
		y2[2] = 0.0;
		y2[3] = 0.0;
		y2[4] = 0.0;
		y2[5] = 0.0;

		y3[0] = 0.0;
		y3[1] = 0.0;
		y3[2] = 0.0;
		y3[3] = 0.0;
		y3[4] = 0.0;
		y3[5] = 0.0;

		y4[0] = 0.0;
		y4[1] = 0.0;
		y4[2] = 0.0;
		y4[3] = 0.0;
		y4[4] = 0.0;
		y4[5] = 0.0;

		y5[0] = 0.0;
		y5[1] = 0.0;
		y5[2] = 0.0;
		y5[3] = 0.0;
		y5[4] = 0.0;
		y5[5] = 0.0;

		y6[0] = 0.0;
		y6[1] = 0.0;
		y6[2] = 0.0;
		y6[3] = 0.0;
		y6[4] = 0.0;
		y6[5] = 0.0;

		return;
	}

	// Initialize tunable PID data
  
	for ( i = 0; i < ESCPID_MAX_ESC; i++ )  {
		switch( i )	{
			case 0:
        if ( *u1 > ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = ESCPID_WRAPPER_MAX_RPM_R;
        else if ( *u1 < -ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = -ESCPID_WRAPPER_MAX_RPM_R;
        else
          RPM_r[i] = (int16_T)*u1;
				PID_P[i] = (uint16_T)*P1;
				PID_I[i] = (uint16_T)*I1;
				PID_D[i] = (uint16_T)*D1;
				PID_f[i] = (uint16_T)*f1;
			break;
			case 1:
				if ( *u2 > ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = ESCPID_WRAPPER_MAX_RPM_R;
        else if ( *u2 < -ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = -ESCPID_WRAPPER_MAX_RPM_R;
        else
          RPM_r[i] = (int16_T)*u2;
				PID_P[i] = (uint16_T)*P2;
				PID_I[i] = (uint16_T)*I2;
				PID_D[i] = (uint16_T)*D2;
				PID_f[i] = (uint16_T)*f2;
			break;
			case 2:
				if ( *u3 > ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = ESCPID_WRAPPER_MAX_RPM_R;
        else if ( *u3 < -ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = -ESCPID_WRAPPER_MAX_RPM_R;
        else
          RPM_r[i] = (int16_T)*u3;
				PID_P[i] = (uint16_T)*P3;
				PID_I[i] = (uint16_T)*I3;
				PID_D[i] = (uint16_T)*D3;
				PID_f[i] = (uint16_T)*f3;
			break;
			case 3:
				if ( *u4 > ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = ESCPID_WRAPPER_MAX_RPM_R;
        else if ( *u4 < -ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = -ESCPID_WRAPPER_MAX_RPM_R;
        else
          RPM_r[i] = (int16_T)*u4;
				PID_P[i] = (uint16_T)*P4;
				PID_I[i] = (uint16_T)*I4;
				PID_D[i] = (uint16_T)*D4;
				PID_f[i] = (uint16_T)*f4;
			break;
			case 4:
				if ( *u5 > ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = ESCPID_WRAPPER_MAX_RPM_R;
        else if ( *u5 < -ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = -ESCPID_WRAPPER_MAX_RPM_R;
        else
          RPM_r[i] = (int16_T)*u5;
				PID_P[i] = (uint16_T)*P5;
				PID_I[i] = (uint16_T)*I5;
				PID_D[i] = (uint16_T)*D5;
				PID_f[i] = (uint16_T)*f5;
			break;
			case 5:
				if ( *u6 > ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = ESCPID_WRAPPER_MAX_RPM_R;
        else if ( *u6 < -ESCPID_WRAPPER_MAX_RPM_R )
          RPM_r[i] = -ESCPID_WRAPPER_MAX_RPM_R;
        else
          RPM_r[i] = (int16_T)*u6;
				PID_P[i] = (uint16_T)*P6;
				PID_I[i] = (uint16_T)*I6;
				PID_D[i] = (uint16_T)*D6;
				PID_f[i] = (uint16_T)*f6;
			break;
			default:
			break;
		}
	}

	// Roundtrip USB communication with teensy
  ret = -1;
  if ( *en )  {
    ret = Host_comm_update( *Port,
                            RPM_r,
                            PID_P,
                            PID_I,
                            PID_D,
                            PID_f,
                            &comm );
  }
	
  // Update return values if no errors
  if ( !ret )	{
    for ( i = 0; i < ESCPID_MAX_ESC; i++ )  {
      switch( i )	{
        case 0:
          y1[0] = (real_T)comm->err[i];
          y1[1] = (real_T)comm->deg[i];
          y1[2] = (real_T)comm->cmd[i];
          y1[3] = (real_T)comm->volt[i];
          y1[4] = (real_T)comm->amp[i];
          y1[5] = (real_T)comm->rpm[i];
        break;
        case 1:
          y2[0] = (real_T)comm->err[i];
          y2[1] = (real_T)comm->deg[i];
          y2[2] = (real_T)comm->cmd[i];
          y2[3] = (real_T)comm->volt[i];
          y2[4] = (real_T)comm->amp[i];
          y2[5] = (real_T)comm->rpm[i];
        break;
        case 2:
          y3[0] = (real_T)comm->err[i];
          y3[1] = (real_T)comm->deg[i];
          y3[2] = (real_T)comm->cmd[i];
          y3[3] = (real_T)comm->volt[i];
          y3[4] = (real_T)comm->amp[i];
          y3[5] = (real_T)comm->rpm[i];
        break;
        case 3:
          y4[0] = (real_T)comm->err[i];
          y4[1] = (real_T)comm->deg[i];
          y4[2] = (real_T)comm->cmd[i];
          y4[3] = (real_T)comm->volt[i];
          y4[4] = (real_T)comm->amp[i];
          y4[5] = (real_T)comm->rpm[i];
        break;
        case 4:
          y5[0] = (real_T)comm->err[i];
          y5[1] = (real_T)comm->deg[i];
          y5[2] = (real_T)comm->cmd[i];
          y5[3] = (real_T)comm->volt[i];
          y5[4] = (real_T)comm->amp[i];
          y5[5] = (real_T)comm->rpm[i];
        break;
        case 5:
          y6[0] = (real_T)comm->err[i];
          y6[1] = (real_T)comm->deg[i];
          y6[2] = (real_T)comm->cmd[i];
          y6[3] = (real_T)comm->volt[i];
          y6[4] = (real_T)comm->amp[i];
          y6[5] = (real_T)comm->rpm[i];
        break;
        default:
        break;
      }
    }
  }
  else  {
    y1[0] = 0.0;
		y1[1] = 0.0;
		y1[2] = 0.0;
		y1[3] = 0.0;
		y1[4] = 0.0;
		y1[5] = 0.0;

		y2[0] = 0.0;
		y2[1] = 0.0;
		y2[2] = 0.0;
		y2[3] = 0.0;
		y2[4] = 0.0;
		y2[5] = 0.0;

		y3[0] = 0.0;
		y3[1] = 0.0;
		y3[2] = 0.0;
		y3[3] = 0.0;
		y3[4] = 0.0;
		y3[5] = 0.0;

		y4[0] = 0.0;
		y4[1] = 0.0;
		y4[2] = 0.0;
		y4[3] = 0.0;
		y4[4] = 0.0;
		y4[5] = 0.0;

		y5[0] = 0.0;
		y5[1] = 0.0;
		y5[2] = 0.0;
		y5[3] = 0.0;
		y5[4] = 0.0;
		y5[5] = 0.0;

		y6[0] = 0.0;
		y6[1] = 0.0;
		y6[2] = 0.0;
		y6[3] = 0.0;
		y6[4] = 0.0;
		y6[5] = 0.0;
  }
	#endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
void rpi_sfun_teensyshot_Terminate_wrapper(const real_T *rpi_Ts, const int_T p_width0,
			const real_T *P1, const int_T p_width1,
			const real_T *I1, const int_T p_width2,
			const real_T *D1, const int_T p_width3,
			const real_T *f1, const int_T p_width4,
			const real_T *P2, const int_T p_width5,
			const real_T *I2, const int_T p_width6,
			const real_T *D2, const int_T p_width7,
			const real_T *f2, const int_T p_width8,
			const real_T *P3, const int_T p_width9,
			const real_T *I3, const int_T p_width10,
			const real_T *D3, const int_T p_width11,
			const real_T *f3, const int_T p_width12,
			const real_T *P4, const int_T p_width13,
			const real_T *I4, const int_T p_width14,
			const real_T *D4, const int_T p_width15,
			const real_T *f4, const int_T p_width16,
			const real_T *P5, const int_T p_width17,
			const real_T *I5, const int_T p_width18,
			const real_T *D5, const int_T p_width19,
			const real_T *f5, const int_T p_width20,
			const real_T *P6, const int_T p_width21,
			const real_T *I6, const int_T p_width22,
			const real_T *D6, const int_T p_width23,
			const real_T *f6, const int_T p_width24,
			const uint32_T *Port, const int_T p_width25)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
	* Custom Terminate code goes here.
	*/
	#ifndef MATLAB_MEX_FILE
	// Check parameter dimension
	if ( 	p_width0 *
				p_width1 *
				p_width2 *
				p_width3 *
				p_width4 *
				p_width5 *
				p_width6 *
				p_width7 *
				p_width8 *
				p_width9 *
				p_width10 *
				p_width11 *
				p_width12 *
				p_width13 *
				p_width14 *
				p_width15 *
				p_width16 *
				p_width17 *
				p_width18 *
				p_width19 *
				p_width20 *
				p_width21 *
				p_width22 *
				p_width23 *
				p_width24 *
				p_width25 != 1 )
		return;
	
	// Check parameter value
	if (
		( *rpi_Ts < 0 ) ||
		( *P1 < 0 ) ||
		( *I1 < 0 ) ||
		( *D1 < 0 ) ||
		( *f1 < 0 ) ||
		( *P2 < 0 ) ||
		( *I2 < 0 ) ||
		( *D2 < 0 ) ||
		( *f2 < 0 ) ||
		( *P3 < 0 ) ||
		( *I3 < 0 ) ||
		( *D3 < 0 ) ||
		( *f3 < 0 ) ||
		( *P4 < 0 ) ||
		( *I4 < 0 ) ||
		( *D4 < 0 ) ||
		( *f4 < 0 ) ||
		( *P5 < 0 ) ||
		( *I5 < 0 ) ||
		( *D5 < 0 ) ||
		( *f5 < 0 ) ||
		( *P6 < 0 ) ||
		( *I6 < 0 ) ||
		( *D6 < 0 ) ||
		( *f6 < 0 ) ||
		( *Port == 0 )
		)
		return;

	// Close serial port
	Host_release_port( *Port );
	#endif
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

