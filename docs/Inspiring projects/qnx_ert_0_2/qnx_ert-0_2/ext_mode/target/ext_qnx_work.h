/*
 * Copyright 1994-2005 The MathWorks, Inc.
 *
 * File: ext_work.h     $Revision: 1.1.6.10 $
 *
 * Abstract:
 *   
 */

#ifndef __EXT_WORK_OBJECT__
#define __EXT_WORK_OBJECT__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef EXT_MODE
#include "ext_types.h"

    
#if defined(VXWORKS)
    /*VxWorks headers*/
    #include <vxWorks.h>
    #include <taskLib.h>
    #include <sysLib.h>
    #include <semLib.h>
    #include <rebootLib.h>
    #include <logLib.h>

    extern void rtExtModeTornadoStartup(RTWExtModeInfo *ei,
                                        int_T          numSampTimes,
                                        boolean_T      *stopReqPtr,
                                        int_T          priority,
                                        int32_T        stack_size,
                                        SEM_ID         startStopSem);

    extern void rtExtModeTornadoCleanup(int_T numSampTimes);

    extern void rtExtModeTornadoSetPortInExtUD(const int_T port);
    
#elif defined(QNX_OS)

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
    extern void rtExtModeTornadoStartup(RTWExtModeInfo *ei,
                                        int_T          numSampTimes,
                                        boolean_T      *stopReqPtr,
                                        int_T          priority,
                                        int32_T        stack_size,
                                        sem_t          *startStopSem);

    extern void rtExtModeTornadoCleanup(int_T numSampTimes);

    extern void rtExtModeTornadoSetPortInExtUD(const int_T port);
        
        
#elif defined(C6000_EXT_MODE)
    /* C6000 headers*/
    #include <std.h>
    #include <stddef.h>
    #include <sem.h>

    extern void rtExtModeC6000Startup( RTWExtModeInfo *ei,
                                       int_T          numSampTimes,
                                       boolean_T      *stopReqPtr);

    extern void rtExtModeC6000Cleanup(int_T numSampTimes);
#else
    extern void rtExtModePauseIfNeeded(RTWExtModeInfo *ei,
                                       int_T          numSampTimes,
                                       boolean_T      *stopReqPtr);

    extern void rtExtModeWaitForStartPkt(RTWExtModeInfo *ei,
                                         int_T          numSampTimes,
                                         boolean_T      *stopReqPtr);
    void rtExtModeInitUD(void);
#endif /* #if defined(VXWORKS) */

extern void rtExtModeOneStep(RTWExtModeInfo *ei,
                             int_T          numSampTimes,
                             boolean_T      *stopReqPtr);

extern void rtExtModeCheckEndTrigger(void);

extern void rtExtModeUploadCheckTrigger(int_T numSampTimes);

extern void rtExtModeUpload(int_T tid,
                            real_T taskTime);

extern void rtExtModeCheckInit(int_T numSampTimes);

extern void rtExtModeShutdown(int_T numSampTimes);

extern void rtExtModeParseArgs(int_T        argc, 
                               const char_T *argv[],
                               real_T       *rtmFinal);

extern void rtERTExtModeSetTFinal(real_T *rtmTFinal);

extern void rtERTExtModeParseArgs(int_T        argc, 
                                  const char_T *argv[]);

#else /* #ifdef EXTMODE */

#if defined(VXWORKS)
    #define rtExtModeTornadoStartup(ei,                          \
                                    numSampTimes,                \
                                    stopReqPtr,                  \
                                    priority,                    \
                                    stack_size,                  \
                                    startStopSem) /* do nothing */
    #define rtExtModeTornadoCleanup(numSampTimes); /* do nothing */
    #define rtExtModeTornadoSetPortInExtUD(port); /* do nothing */
#elif defined(C6000_EXT_MODE)  
    #define rtExtModePauseIfNeeded(ei,st,sr) /* do nothing */
    #define rtExtModeWaitForStartPkt(ei,st,sr) /* do nothing */
#else
    #define rtExtModePauseIfNeeded(ei,st,sr) /* do nothing */
    #define rtExtModeWaitForStartPkt(ei,st,sr) /* do nothing */
#endif /* #ifdef VXWORKS */

 
    
    
#define rtExtModeOneStep(ei,st,sr) /* do nothing */
#define rtExtModeCheckEndTrigger() /* do nothing */
#define rtExtModeUploadCheckTrigger(numSampTimes) /* do nothing */
#define rtExtModeUpload(t,ttime) /* do nothing */
#define rtExtModeCheckInit(numSampTimes) /* do nothing */
#define rtExtModeShutdown(numSampTimes) /* do nothing */
#define rtExtModeParseArgs(argc, argv, tf); /* do nothing */
#define rtERTExtModeSetTFinal(tf); /* do nothing */
#define rtERTExtModeParseArgs(argc, argv); /* do nothing */

#endif  /* #ifdef EXTMODE */

#ifdef __cplusplus
}
#endif

#endif /* __EXT_WORK_OBJECT__ */
