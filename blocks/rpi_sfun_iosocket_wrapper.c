
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 150
#define y_width 150

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
#define RPIT_SOCKET_API
#include "rpit_socket_client.c"
#endif

#define RPIT_SOCKET_PKG_SIZE 150
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void rpi_sfun_iosocket_Start_wrapper(real_T *xD,
			const real_T *rpi_Ts, const int_T p_width0,
			const uint8_T *rpi_ip1, const int_T p_width1,
			const uint8_T *rpi_ip2, const int_T p_width2,
			const uint8_T *rpi_ip3, const int_T p_width3,
			const uint8_T *rpi_ip4, const int_T p_width4,
			const uint32_T *rpi_port, const int_T p_width5)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
  (void)*xD;
  (void)*rpi_Ts;
  (void)p_width0;
  (void)p_width1;
  (void)p_width2;
  (void)p_width3;
  (void)p_width4;
  (void)p_width5;
  
  rpit_socket_client_add( *rpi_ip1, *rpi_ip2, *rpi_ip3, *rpi_ip4, *rpi_port );
  
#endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void rpi_sfun_iosocket_Outputs_wrapper(real_T *y0,
			const real_T *xD,
			const real_T *rpi_Ts, const int_T p_width0,
			const uint8_T *rpi_ip1, const int_T p_width1,
			const uint8_T *rpi_ip2, const int_T p_width2,
			const uint8_T *rpi_ip3, const int_T p_width3,
			const uint8_T *rpi_ip4, const int_T p_width4,
			const uint32_T *rpi_port, const int_T p_width5)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
#ifdef MATLAB_MEX_FILE
  for (int ii = 0; ii < RPIT_SOCKET_PKG_SIZE; ii++) {
    y0[ii] = 0.0; 
  }
#else
  (void)*xD;
  (void)p_width0;
  (void)p_width1;
  (void)p_width2;
  (void)p_width3;
  (void)p_width4;
  (void)p_width5;
  
  if ( *rpi_Ts < 0.001 )	{
    fprintf( stderr, "** Max sampling rate = 1000Hz **\n" );
    for (int ii = 0; ii < RPIT_SOCKET_PKG_SIZE; ii++) {
        y0[ii] = 0.0; 
    }
    return;
  }
  
  /* Read measurements */
  
  rpit_socket_client_read( *rpi_ip1, *rpi_ip2, *rpi_ip3, *rpi_ip4, *rpi_port, y0 );
  
#endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void rpi_sfun_iosocket_Update_wrapper(const real_T *u0,
			real_T *y0,
			real_T *xD,
			const real_T *rpi_Ts, const int_T p_width0,
			const uint8_T *rpi_ip1, const int_T p_width1,
			const uint8_T *rpi_ip2, const int_T p_width2,
			const uint8_T *rpi_ip3, const int_T p_width3,
			const uint8_T *rpi_ip4, const int_T p_width4,
			const uint32_T *rpi_port, const int_T p_width5)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
  (void)*y0;
  (void)*xD;
  (void)*rpi_Ts;
  (void)p_width0;
  (void)p_width1;
  (void)p_width2;
  (void)p_width3;
  (void)p_width4;
  (void)p_width5;

  /* Send control signals */
  
  rpit_socket_client_write( *rpi_ip1, *rpi_ip2, *rpi_ip3, *rpi_ip4,*rpi_port,  u0 );
  
#endif
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void rpi_sfun_iosocket_Terminate_wrapper(real_T *xD,
			const real_T *rpi_Ts, const int_T p_width0,
			const uint8_T *rpi_ip1, const int_T p_width1,
			const uint8_T *rpi_ip2, const int_T p_width2,
			const uint8_T *rpi_ip3, const int_T p_width3,
			const uint8_T *rpi_ip4, const int_T p_width4,
			const uint32_T *rpi_port, const int_T p_width5)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
  (void)*xD;
  (void)*rpi_Ts;
  (void)p_width0;
  (void)p_width1;
  (void)p_width2;
  (void)p_width3;
  (void)p_width4;
  (void)p_width5;

  rpit_socket_client_close( *rpi_ip1, *rpi_ip2, *rpi_ip3, *rpi_ip4, *rpi_port);
  
#endif
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

