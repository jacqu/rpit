#ifndef __EXT_QNX_DEFS__
#define __EXT_QNX_DEFS__

typedef  struct pktServerArgTag {
    RTWExtModeInfo *ei;
    int_T          numSampTimes;
    boolean_T      *stopReq;
}pktServerArgT;

#endif
   