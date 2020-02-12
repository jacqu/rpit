
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
/* extern double func(double a); */
#ifndef MATLAB_MEX_FILE
#include "uhubctl.c"
#endif

double errCode;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void rpi_sfun_resetusb_Start_wrapper(const real_T *rpi_Ts, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */
#ifndef MATLAB_MEX_FILE
    if ( p_width0 > 1 )	{
        fprintf( stderr, "** Multiple rates not allowed in this block **\n" );
    }

    if ( *rpi_Ts < 0.001 )	{
        fprintf( stderr, "** Warning : sampling rates beyond 1000Hz may yield overruns  **\n" );
    }
    
    int rc;

    errCode = 0; 
    opt_location[0] = '2'; 
    opt_action = POWER_CYCLE; 
    opt_delay = 5; 

    rc = libusb_init(NULL);
    if (rc < 0) {
        fprintf(stderr,
            "Error initializing USB!\n"
        );
        errCode = -1; 
        exit(1);
    }

    rc = libusb_get_device_list(NULL, &usb_devs);
    if (rc < 0) {
        fprintf(stderr,
            "Cannot enumerate USB devices!\n"
        );
        rc = 1;
        errCode = -2; 
        goto cleanup;
    }

    rc = usb_find_hubs();
    if (rc <= 0) {
        fprintf(stderr,
            "No compatible smart hubs detected%s%s!\n"
            "Run with -h to get usage info.\n",
            strlen(opt_location) ? " at location " : "",
            opt_location
        );
        errCode = -3; 
#ifdef __gnu_linux__
        if (rc < 0 && geteuid() != 0) {
            fprintf(stderr,
                "There were permission problems while accessing USB.\n"
                "To fix this, run this tool as root using 'sudo uhubctl',\n"
                "or add one or more udev rules like below\n"
                "to file '/etc/udev/rules.d/52-usb.rules':\n"
                "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2001\", MODE=\"0666\"\n"
                "then run 'sudo udevadm trigger --attr-match=subsystem=usb'\n"
            );
            errCode = -4; 
        }
#endif
        rc = 1;
        goto cleanup;
    }

    int k; /* k=0 for power OFF, k=1 for power ON */
    for (k=0; k<2; k++) { /* up to 2 power actions - off/on */
        if (k == 0 && opt_action == POWER_ON )
            continue;
        if (k == 1 && opt_action == POWER_OFF)
            continue;
        if (k == 1 && opt_action == POWER_KEEP)
            continue;
        int i;
        for (i=0; i<hub_count; i++) {
            if (hubs[i].actionable == 0)
                continue;
            printf("Current status for hub %s [%s]\n",
                hubs[i].location, hubs[i].ds.description
            );
            print_port_status(&hubs[i], opt_ports);
            if (opt_action == POWER_KEEP) { /* no action, show status */
                continue;
            }
            struct libusb_device_handle * devh = NULL;
            rc = libusb_open(hubs[i].dev, &devh);
            if (rc == 0) {
                /* will operate on these ports */
                int ports = ((1 << hubs[i].nports) - 1) & opt_ports;
                int request = (k == 0) ? LIBUSB_REQUEST_CLEAR_FEATURE
                                       : LIBUSB_REQUEST_SET_FEATURE;
                int port;
                for (port=1; port <= hubs[i].nports; port++) {
                    if ((1 << (port-1)) & ports) {
                        int port_status = get_port_status(devh, port);
                        int power_mask = hubs[i].bcd_usb < USB_SS_BCD ? USB_PORT_STAT_POWER
                                                                      : USB_SS_PORT_STAT_POWER;
                        if (k == 0 && !(port_status & power_mask))
                            continue;
                        if (k == 1 && (port_status & power_mask))
                            continue;
                        int repeat = 1;
                        if (k == 0)
                            repeat = opt_repeat;
                        if (!(port_status & ~power_mask))
                            repeat = 1;
                        while (repeat-- > 0) {
                            rc = libusb_control_transfer(devh,
                                LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_OTHER,
                                request, USB_PORT_FEAT_POWER,
                                port, NULL, 0, USB_CTRL_GET_TIMEOUT
                            );
                            if (rc < 0) {
                                perror("Failed to control port power!\n");
                                errCode = -5; 
                            }
                            if (repeat > 0) {
                                sleep_ms(opt_wait);
                            }
                        }
                    }
                }
                /* USB3 hubs need extra delay to actually turn off: */
                if (k==0 && hubs[i].bcd_usb >= USB_SS_BCD)
                    sleep_ms(150);
                printf("Sent power %s request\n",
                    request == LIBUSB_REQUEST_CLEAR_FEATURE ? "off" : "on"
                );
                printf("New status for hub %s [%s]\n",
                    hubs[i].location, hubs[i].ds.description
                );
                print_port_status(&hubs[i], opt_ports);

                if (k == 1 && opt_reset == 1) {
                    printf("Resetting hub...\n");
                    rc = libusb_reset_device(devh);
                    if (rc < 0) {
                        perror("Reset failed!\n");
                    } else {
                        printf("Reset successful!\n");
                    }
                }
            }
            libusb_close(devh);
        }
        if (k == 0 && opt_action == POWER_CYCLE) {
            printf("Delay %d\n",(int)(opt_delay * 1000));
            sleep_ms((int)(opt_delay * 1000));
        }
        if (k == 1 && opt_action == POWER_CYCLE) {
            printf("Delay %d\n",(int)(opt_delay * 1000));
            sleep_ms((int)(opt_delay * 1000));
        }
    }
    rc = 0;
cleanup:
    if (usb_devs)
        libusb_free_device_list(usb_devs, 1);
    usb_devs = NULL;
    libusb_exit(NULL);
 
#endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void rpi_sfun_resetusb_Outputs_wrapper(real_T *err,
			const real_T *rpi_Ts, const int_T p_width0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
#ifdef MATLAB_MEX_FILE
    err[0] = 0.0; 
#else

    if ( p_width0 > 1 )	{
        fprintf( stderr, "** Multiple rates not allowed in this block **\n" );
    }

    if ( *rpi_Ts < 0.001 )	{
        fprintf( stderr, "** Warning : sampling rates beyond 1000Hz may yield overruns  **\n" );
    }

    err[0] = (real_T)errCode;
#endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


