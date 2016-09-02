/*
  *
  *   --- THIS FILE GENERATED BY S-FUNCTION BUILDER: 3.0 ---
  *
  *   This file is a wrapper S-function produced by the S-Function
  *   Builder which only recognizes certain fields.  Changes made
  *   outside these fields will be lost the next time the block is
  *   used to load, edit, and resave this file. This file will be overwritten
  *   by the S-function Builder block. If you want to edit this file by hand, 
  *   you must change it only in the area defined as:  
  *
  *        %%%-SFUNWIZ_wrapper_XXXXX_Changes_BEGIN 
  *            Your Changes go here
  *        %%%-SFUNWIZ_wrapper_XXXXXX_Changes_END
  *
  *   For better compatibility with the Real-Time Workshop, the
  *   "wrapper" S-function technique is used.  This is discussed
  *   in the Real-Time Workshop User's Manual in the Chapter titled,
  *   "Wrapper S-functions".
  *
  *   Created: Wed Aug 31 16:16:04 2016
  */


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

#ifndef MATLAB_MEX_FILE

#define DXL_API
#include "dxl.c"

#define DXL_SIMULINK_MAX_DEV                  20
#define DXL_SIMULINK_MAX_ID                   10
#define DXL_SIMULINK_DEFAULT_PORTNAME         "/dev/ttyUSB0"
#define DXL_SIMULINK_PRORTNAME_MAXSIZE        32

/* Convert port number into serial device name */
char  dxl_simulink_portname[DXL_SIMULINK_PRORTNAME_MAXSIZE];
char* dxl_portnb2portname( uint8_t portname )  {
  char buf[DXL_SIMULINK_PRORTNAME_MAXSIZE];
  
  strncpy( dxl_simulink_portname, DXL_SIMULINK_DEFAULT_PORTNAME, DXL_SIMULINK_PRORTNAME_MAXSIZE );
  
  if ( portname < 10 )  {
    snprintf( buf, DXL_SIMULINK_PRORTNAME_MAXSIZE, "/dev/ttyUSB%d", portname );
    strncpy( dxl_simulink_portname, buf, DXL_SIMULINK_PRORTNAME_MAXSIZE );
  }
  
  if ( ( portname >= 10 ) && ( portname < 20 ) )  {
    snprintf( buf, DXL_SIMULINK_PRORTNAME_MAXSIZE, "/dev/ttyS%d", portname - 10 );
    strncpy( dxl_simulink_portname, buf, DXL_SIMULINK_PRORTNAME_MAXSIZE );
  }

  return dxl_simulink_portname;
}

#endif

/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output functions
 *
 */
void rpi_sfun_dxl_Outputs_wrapper(
                          const real_T *u1,
                          const real_T *u2,
                          const real_T *u3,
                          const real_T *u4,
                          const real_T *u5,
                          const real_T *u6,
                          const real_T *u7,
                          const real_T *u8,
                          const real_T *u9,
                          const real_T *u10,
                          real_T *y1,
                          real_T *y2,
                          real_T *y3,
                          real_T *y4,
                          real_T *y5,
                          real_T *y6,
                          real_T *y7,
                          real_T *y8,
                          real_T *y9,
                          real_T *y10, 
                          const real_T  *rpi_Ts,
                          const uint8_T  *rpi_portname,
                          const uint32_T  *rpi_baudrate,
                          const uint8_T  *rpi_startid,
                          const uint8_T  *rpi_nbid,
                          const uint8_T  *rpi_proto,
                          const uint16_T  *rpi_write_addr,
                          const uint8_T  *rpi_write_length, 
                          const uint16_T  *rpi_read_addr,
                          const uint8_T  *rpi_read_length,
                          const uint8_T  *rpi_read_sign)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
  
  #ifdef MATLAB_MEX_FILE

  y1[0] = 0.0;
  y2[0] = 0.0;
  y3[0] = 0.0;
  y4[0] = 0.0;
  y5[0] = 0.0;
  y6[0] = 0.0;
  y7[0] = 0.0;
  y8[0] = 0.0;
  y9[0] = 0.0;
  y10[0] = 0.0;

  #else

  int           i, err;
  double        data[DXL_SIMULINK_MAX_ID];
  static double old_y[DXL_SIMULINK_MAX_DEV][DXL_SIMULINK_MAX_ID] = {{ 0.0 }};
	
	(void)rpi_baudrate;
	
  /* Consistency check */

  if ( *rpi_nbid > DXL_SIMULINK_MAX_ID )	{
    fprintf( stderr, "** Max devices = %d **\n", DXL_SIMULINK_MAX_ID );
    y1[0] = 0.0;
    y2[0] = 0.0;
    y3[0] = 0.0;
    y4[0] = 0.0;
    y5[0] = 0.0;
    y6[0] = 0.0;
    y7[0] = 0.0;
    y8[0] = 0.0;
    y9[0] = 0.0;
    y10[0] = 0.0;
    return;
  }
  
  if ( *rpi_nbid < 1 )	{
    fprintf( stderr, "** Min devices = 1 **\n" );
    y1[0] = 0.0;
    y2[0] = 0.0;
    y3[0] = 0.0;
    y4[0] = 0.0;
    y5[0] = 0.0;
    y6[0] = 0.0;
    y7[0] = 0.0;
    y8[0] = 0.0;
    y9[0] = 0.0;
    y10[0] = 0.0;
    return;
  }
  
  if ( *rpi_portname > DXL_SIMULINK_MAX_DEV - 1 ) {
    fprintf( stderr, "** Max port number = %d **\n", DXL_SIMULINK_MAX_DEV - 1 );
    y1[0] = 0.0;
    y2[0] = 0.0;
    y3[0] = 0.0;
    y4[0] = 0.0;
    y5[0] = 0.0;
    y6[0] = 0.0;
    y7[0] = 0.0;
    y8[0] = 0.0;
    y9[0] = 0.0;
    y10[0] = 0.0;
    return;
  }

  if ( *rpi_Ts < 0.005 )	{
    fprintf( stderr, "** Max sampling rate = 200Hz **\n" );
    y1[0] = 0.0;
    y2[0] = 0.0;
    y3[0] = 0.0;
    y4[0] = 0.0;
    y5[0] = 0.0;
    y6[0] = 0.0;
    y7[0] = 0.0;
    y8[0] = 0.0;
    y9[0] = 0.0;
    y10[0] = 0.0;
    return;
  }

  /* Write inputs */

  for ( i = 0; i < *rpi_nbid; i++ ) {

    switch( i ) {
      case 0:
        data[i] = u1[0];
        break;

      case 1:
        data[i] = u2[0];
        break;

      case 2:
        data[i] = u3[0];
        break;

      case 3:
        data[i] = u4[0];
        break;

      case 4:
        data[i] = u5[0];
        break;

      case 5:
        data[i] = u6[0];
        break;

      case 6:
        data[i] = u7[0];
        break;

      case 7:
        data[i] = u8[0];
        break;

      case 8:
        data[i] = u9[0];
        break;

      case 9:
        data[i] = u10[0];
        break;

      default:
        fprintf( stderr, "** Internal error. **\n" );
    }
  }

  err = dxl_write(  dxl_portnb2portname( *rpi_portname ),
                    *rpi_proto,
                    *rpi_startid,
                    *rpi_nbid,
                    *rpi_write_addr,
                    *rpi_write_length,
                    data );
  if ( err )
    fprintf( stderr, "** dxl_write: error %d while writing device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );

  /* Read outputs */
  
  y1[0] = old_y[*rpi_portname][0];
  y2[0] = old_y[*rpi_portname][1];
  y3[0] = old_y[*rpi_portname][2];
  y4[0] = old_y[*rpi_portname][3];
  y5[0] = old_y[*rpi_portname][4];
  y6[0] = old_y[*rpi_portname][5];
  y7[0] = old_y[*rpi_portname][6];
  y8[0] = old_y[*rpi_portname][7];
  y9[0] = old_y[*rpi_portname][8];
  y10[0] = old_y[*rpi_portname][9];
    
  err = dxl_read( dxl_portnb2portname( *rpi_portname ),
                  *rpi_proto,
                  *rpi_startid,
                  *rpi_nbid,
                  *rpi_read_addr,
                  *rpi_read_length,
                  *rpi_read_sign,
                  data );
  
  if ( err )  {
    fprintf( stderr, "** dxl_read: error %d while reading device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  for ( i = 0; i < *rpi_nbid; i++ ) {
    
    old_y[*rpi_portname][i] = data[i];
    
    switch( i ) {
      case 0:
        y1[0] = data[i];
        break;

      case 1:
        y2[0] = data[i];
        break;

      case 2:
        y3[0] = data[i];
        break;

      case 3:
        y4[0] = data[i];
        break;

      case 4:
        y5[0] = data[i];
        break;

      case 5:
        y6[0] = data[i];
        break;

      case 6:
        y7[0] = data[i];
        break;

      case 7:
        y8[0] = data[i];
        break;

      case 8:
        y9[0] = data[i];
        break;

      case 9:
        y10[0] = data[i];
        break;

      default:
        fprintf( stderr, "** Internal error. **\n" );
    }
  }

  #endif

/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/* Start function */

void rpi_sfun_dxl_Start_wrapper(
                          const real_T *u1,
                          const real_T *u2,
                          const real_T *u3,
                          const real_T *u4,
                          const real_T *u5,
                          const real_T *u6,
                          const real_T *u7,
                          const real_T *u8,
                          const real_T *u9,
                          const real_T *u10,
                          real_T *y1,
                          real_T *y2,
                          real_T *y3,
                          real_T *y4,
                          real_T *y5,
                          real_T *y6,
                          real_T *y7,
                          real_T *y8,
                          real_T *y9,
                          real_T *y10, 
                          const real_T  *rpi_Ts,
                          const uint8_T  *rpi_portname,
                          const uint32_T  *rpi_baudrate,
                          const uint8_T  *rpi_startid,
                          const uint8_T  *rpi_nbid,
                          const uint8_T  *rpi_proto,
                          const uint16_T  *rpi_write_addr,
                          const uint8_T  *rpi_write_length, 
                          const uint16_T  *rpi_read_addr,
                          const uint8_T  *rpi_read_length,
                          const uint8_T  *rpi_read_sign,
                          const uint16_T  *rpi_init1_addr,
                          const uint8_T  *rpi_init1_length,
                          const real_T  *rpi_init1_data,
                          const uint16_T  *rpi_init2_addr,
                          const uint8_T  *rpi_init2_length,
                          const real_T  *rpi_init2_data,
                          const uint16_T  *rpi_init3_addr,
                          const uint8_T  *rpi_init3_length,
                          const real_T  *rpi_init3_data,
                          const uint16_T  *rpi_init4_addr,
                          const uint8_T  *rpi_init4_length,
                          const real_T  *rpi_init4_data,
                          const uint16_T  *rpi_init5_addr,
                          const uint8_T  *rpi_init5_length,
                          const real_T  *rpi_init5_data,
                          const uint16_T  *rpi_init6_addr,
                          const uint8_T  *rpi_init6_length,
                          const real_T  *rpi_init6_data,
                          const uint16_T  *rpi_init7_addr,
                          const uint8_T  *rpi_init7_length,
                          const real_T  *rpi_init7_data,
                          const uint16_T  *rpi_init8_addr,
                          const uint8_T  *rpi_init8_length,
                          const real_T  *rpi_init8_data)
{
  #ifndef MATLAB_MEX_FILE
  
  int     i, err;
  double  data[DXL_SIMULINK_MAX_ID];
  
  (void)u1;
  (void)u2;
  (void)u3;
  (void)u4;
  (void)u5;
  (void)u6;
  (void)u7;
  (void)u8;
  (void)u9;
  (void)u10;
  (void)y1;
  (void)y2;
  (void)y3;
  (void)y4;
  (void)y5;
  (void)y6;
  (void)y7;
  (void)y8;
  (void)y9;
  (void)y10;
  (void)rpi_Ts;
  (void)rpi_write_addr;
  (void)rpi_write_length;
  (void)rpi_read_addr;
  (void)rpi_read_length;
  (void)rpi_read_sign;
  
  /* Consistency check */
  
  if ( ( *rpi_nbid > DXL_SIMULINK_MAX_ID ) ||
       ( *rpi_nbid < 1 ) ||
       ( *rpi_portname > DXL_SIMULINK_MAX_DEV - 1 ) ) {
    fprintf( stderr, "** Device %s: out of range parameters error. **\n", dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Open serial device */
  
  err = dxl_open( dxl_portnb2portname( *rpi_portname ), *rpi_baudrate );
  if ( err )  {
    fprintf( stderr, "** dxl_open: error %d while opening device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Initialize sequence */
  
  /* Instruction 1 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init1_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init1_addr,
          *rpi_init1_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 1 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
 
 /* Instruction 2 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init2_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init2_addr,
          *rpi_init2_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 2 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Instruction 3 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init3_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init3_addr,
          *rpi_init3_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 3 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Instruction 4 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init4_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init4_addr,
          *rpi_init4_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 4 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Instruction 5 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init5_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init5_addr,
          *rpi_init5_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 5 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Instruction 6 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init6_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init6_addr,
          *rpi_init6_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 6 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Instruction 7 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init7_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init7_addr,
          *rpi_init7_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 7 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Instruction 8 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_init8_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_init8_addr,
          *rpi_init8_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing initialization instruction 8 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  fprintf( stderr, "** Device %s successfully initialized **\n", dxl_portnb2portname( *rpi_portname ) );
  
  #endif
}

/* Terminate function */

void rpi_sfun_dxl_Terminate_wrapper(
                          const real_T  *rpi_Ts,
                          const uint8_T  *rpi_portname,
                          const uint32_T  *rpi_baudrate,
                          const uint8_T  *rpi_startid,
                          const uint8_T  *rpi_nbid,
                          const uint8_T  *rpi_proto,
                          const uint16_T  *rpi_write_addr,
                          const uint8_T  *rpi_write_length, 
                          const uint16_T  *rpi_read_addr,
                          const uint8_T  *rpi_read_length,
                          const uint8_T  *rpi_read_sign,
                          const uint16_T  *rpi_halt1_addr,
                          const uint8_T  *rpi_halt1_length,
                          const real_T  *rpi_halt1_data,
                          const uint16_T  *rpi_halt2_addr,
                          const uint8_T  *rpi_halt2_length,
                          const real_T  *rpi_halt2_data)
{
  #ifndef MATLAB_MEX_FILE
  
  int     i, err;
  double  data[DXL_SIMULINK_MAX_ID];
  
  (void)rpi_Ts;
  (void)rpi_baudrate;
  (void)rpi_write_addr;
  (void)rpi_write_length;
  (void)rpi_read_addr;
  (void)rpi_read_length;
  (void)rpi_read_sign;
  
  /* Consistency check */
  
  if ( ( *rpi_nbid > DXL_SIMULINK_MAX_ID ) ||
       ( *rpi_nbid < 1 ) ||
       ( *rpi_portname > DXL_SIMULINK_MAX_DEV - 1 ) ) {
    return;
  }
  
  /* Shutdown instructions */
  
  /* Instruction 1 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_halt1_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_halt1_addr,
          *rpi_halt1_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing halt instruction 1 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
 
 /* Instruction 2 */
  
  for ( i = 0; i < *rpi_nbid; i++ )
    data[i] = *rpi_halt2_data;
  err = dxl_write(
          dxl_portnb2portname( *rpi_portname ), 
          *rpi_proto,
          *rpi_startid,
          *rpi_nbid,
          *rpi_halt2_addr,
          *rpi_halt2_length,
          data );
  if ( err )  {
    fprintf( stderr, "** dxl_write: error %d while writing halt instruction 2 on device %s. **\n", err, dxl_portnb2portname( *rpi_portname ) );
    return;
  }
  
  /* Close serial link */
  
  dxl_close( dxl_portnb2portname( *rpi_portname ) );
  
  fprintf( stderr, "** Device %s successfully halted **\n", dxl_portnb2portname( *rpi_portname ) );
  
  #endif
}
