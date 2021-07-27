
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
#define SFUN_BLK_MIN_PERIOD                0.003            // Minimum sampling period
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 8
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
#if defined(MATLAB_MEX_FILE)
#include <stdint.h>
#include <pthread.h>
#include "betalink.h"
#else
#include "betalink.c"
#endif
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void rpi_sfun_betalink_Start_wrapper(const real_T *rpi_Ts, const int_T p_width0,
			const real_T *usb_serial_number, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
// Silence unused variable warning
(void)p_width0;
(void)p_width1;

int                ret;

// Test for minimum sampling period
if ( *rpi_Ts < SFUN_BLK_MIN_PERIOD )
    fprintf( stderr, "rpi_sfun_betalink: minimum sampling period is %f\n", SFUN_BLK_MIN_PERIOD );

// Initialize USB port
ret = blk_init_port( (uint32_t)*usb_serial_number );
if ( ret )
    fprintf( stderr, "rpi_sfun_betalink: error %d opening USB port SN%u\n", ret, (unsigned int)*usb_serial_number );
#endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void rpi_sfun_betalink_Outputs_wrapper(const real_T *throttle,
			real_T *timestamp,
			real_T *rpm,
			real_T *inv,
			real_T *acc,
			real_T *gyr,
			real_T *mag,
			real_T *roll,
			real_T *pitch,
			real_T *yaw,
			real_T *bat_volt,
			real_T *bat_amp,
			real_T *bat_mah,
			const real_T *rpi_Ts, const int_T p_width0,
			const real_T *usb_serial_number, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */
  // Silence unused variable warning
  (void)p_width0;
  (void)p_width1;

  int                 i;
  #ifndef MATLAB_MEX_FILE
  int                 ret;
  static blk_state_t  state;
  uint16_t            throttle_msp[BLK_MAX_MOTORS];
  #endif

  // Default output is zero
  *timestamp = 0;
  for ( i = 0; i < BLK_MAX_MOTORS; i++ )    {
    rpm[i] = 0.0;
    inv[i] = 0.0;
  }
  for ( i = 0; i < 3; i++ )    {
    acc[i] = gyr[i] = mag[i] = 0.0;
  }
  *roll = *pitch = *yaw = 0.0;
  *bat_volt = *bat_amp = *bat_mah = 0.0;

  // Check min sampling period
  if ( *rpi_Ts < SFUN_BLK_MIN_PERIOD )    {
    return;
  }

  #ifndef MATLAB_MEX_FILE
  // USB transaction with Betaflight
  for ( i = 0; i < BLK_MAX_MOTORS; i++ )
    throttle_msp[i] = lrint( throttle[i] );
  ret = blk_update_threaded( (uint32_t)*usb_serial_number, throttle_msp );
  if ( ret ) {
    fprintf( stderr, "rpi_sfun_betalink: error %d in blk_update_threaded.\n", ret );
  }
  // Copy data structure locally
  ret = blk_copy_state( (uint32_t)*usb_serial_number, &state );
  if ( ret ) {
    fprintf( stderr, "rpi_sfun_betalink: error %d in blk_copy_state.\n", ret );
    // Error <-> bad serial number: force all outputs to zero
    memset( &state, 0, sizeof( blk_state_t ) );
  }
  // Copy states to block outputs
  *timestamp = state.timestamp;
  for ( i = 0; i < BLK_MAX_MOTORS; i++ )    {
      rpm[i] = state.rpm[i];
      inv[i] = state.inv[i] * BLK_MSP_INV_SCALING;
  }
  for ( i = 0; i < 3; i++ )    {
      acc[i] = state.acc[i] * BLK_MSP_ACC_SCALING;
      gyr[i] = state.gyr[i] * BLK_MSP_GYR_SCALING;
      mag[i] = state.mag[i] * BLK_MSP_MAG_SCALING;
  }
  *roll = state.roll;
  *pitch = state.pitch;
  *yaw = state.yaw;
  *bat_volt = state.bat_volt * BLK_MSP_BATV_SCALING;
  *bat_amp = state.bat_amp * BLK_MSP_BATA_SCALING;
  *bat_mah = state.bat_mah;
  #endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
void rpi_sfun_betalink_Terminate_wrapper(const real_T *rpi_Ts, const int_T p_width0,
			const real_T *usb_serial_number, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
// Silence unused variable warning
    (void)rpi_Ts;
    (void)p_width0;
    (void)p_width1;
    
    #ifndef MATLAB_MEX_FILE
    blk_release_port( (uint32_t)*usb_serial_number );
    #endif
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

