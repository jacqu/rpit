
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/procfs.h>
#include <sys/states.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/syspage.h>
#include <sys/socket.h>
#include <sys/siginfo.h>
#include <rpc/rpc.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <process.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <semaphore.h>
#include "ert_qnx_utils.h"


/* Function: tgtCreateThread ========================================================
 *
 * Abstract:
 *    Creates a thread for the target
 */



pthread_t  tgtCreateThread(char *threadName,  void* (*threadRoot)(void* ),
                                   void *threadArg, int priority, int stackSize)
{
    int             prio_max;
    int             prio_min;
    struct _clockperiod clockperiodstruct;
    struct _clockperiod new_clockperiodstruct;
    int             clock_period_stat;
    int             policy;
    pthread_attr_t  attr;
    sched_param_t   schedParam;
    int             GBL_chid;
    int             pulse_id;
    unsigned int    GBL_cpu_freq;
    int             i;
    int             thIndex;
    int             sched_th_index;
    int             ct;
    int             status;
    double          dbl;
    pthread_t       threadId;

    
    /*
     * Get the default attributes of threads and modify them.
     */
  
     
     pthread_attr_init(&attr); 
     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED); 
     pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);   
     pthread_attr_getschedparam(&attr, &schedParam);
     
     /*
      * Create the user thread.
      */
     
     
      pthread_attr_setstacksize(&attr, stackSize);
      pthread_create(&threadId, &attr,
                     threadRoot, threadArg);
      
      /*
       * Set the schedulng parameters for the thread.
       */
      
      policy = SCHED_RR;
      schedParam.sched_priority = priority;
      pthread_setschedparam (threadId, policy, &schedParam);

      
      /*
       * Check the scheduling parameters
       */
       pthread_getschedparam(threadId, &policy,&schedParam);
      
       printf("In tgtCreateThread %s\n", threadName );
       
      return threadId;     
}