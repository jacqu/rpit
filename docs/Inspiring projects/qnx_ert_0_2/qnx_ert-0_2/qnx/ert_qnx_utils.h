#ifndef  __ERT_QNX_UTILS_H__
#define  __ERT_QNX_UTILS_H__

extern pthread_t  tgtCreateThread(char *threadName,  void* (*threadRoot)(void* ),
                                   void *threadArg, int priority, int stackSize);


#endif