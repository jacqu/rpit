
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
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
#define RPITJS_NB_AXES    8
#define RPITJS_NB_BUTTONS 15

#ifndef MATLAB_MEX_FILE

#define RPITJS_API
#include "rpit_js.c"

#endif
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void rpi_sfun_xboxone_Outputs_wrapper(real_T *Axes,
			real_T *Buttons,
			const real_T *rpi_ID, const int_T p_width0,
			const real_T *rpi_Ts, const int_T p_width1)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
int i;
  (void)p_width0;
  (void)p_width1;

  #ifdef MATLAB_MEX_FILE

  for ( i = 0; i < RPITJS_NB_AXES; i++ )
    Axes[i] = 0.0;
  for ( i = 0; i < RPITJS_NB_BUTTONS; i++ )
    Buttons[i] = 0.0;

  #else
  
  /* Consistency check */
          
  if ( ( *rpi_ID >= RPITJS_MAX_JS ) || ( *rpi_ID < 0 ) || ( *rpi_Ts < 0.001 ) )  {
    for ( i = 0; i < RPITJS_NB_AXES; i++ )
      Axes[i] = 0.0;
    for ( i = 0; i < RPITJS_NB_BUTTONS; i++ )
      Buttons[i] = 0.0;
    return;
  }
  
  /* Read jostick values */
  
  if ( rpitjs_read( (int)*rpi_ID ) < 0 )  {
    for ( i = 0; i < RPITJS_NB_AXES; i++ )
      Axes[i] = 0.0;
    for ( i = 0; i < RPITJS_NB_BUTTONS; i++ )
      Buttons[i] = 0.0;
  }
  else {
    for ( i = 0; i < RPITJS_NB_AXES; i++ )
      if ( i < rpitjs_struct[(int)*rpi_ID].nb_axes )
        Axes[i] = rpitjs_struct[(int)*rpi_ID].axis_values[i];
      else
        Axes[i] = 0.0;
    for ( i = 0; i < RPITJS_NB_BUTTONS; i++ )
      if ( i < rpitjs_struct[(int)*rpi_ID].nb_buttons )
        Buttons[i] = rpitjs_struct[(int)*rpi_ID].button_values[i];
      else
        Buttons[i] = 0.0;
  }
  
  #endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


