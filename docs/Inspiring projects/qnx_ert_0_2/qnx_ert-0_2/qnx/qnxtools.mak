# Copyright 1994-2000 by The MathWorks, Inc.
#
# File    : lcctools.mak   $Revision: 1.5.2.1 $
# Abstract:
#	Setup LCC tools for gmake.

CC     = $(QNX_HOST)/usr/bin/qcc
LD     = $(QNX_HOST)/usr/bin/qcc
LIBPATH    = $(QNX_TARGET)/armle-v7/lib
LIBCMD = $(QNX_HOST)/usr/bin/ntoarmv7-ar

DEFAULT_OPT_OPTS = 

#------------------------------------#
# Setup INCLUDES, DSP_MEX source dir #
#------------------------------------#

MATLAB_INCLUDES = \
	-I$(MATLAB_ROOT)/simulink/include \
	-I$(MATLAB_ROOT)/extern/include \
	-I$(MATLAB_ROOT)/rtw/c/src \
	-I$(MATLAB_ROOT)/rtw/c/libsrc \
	-I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common \
	-I$(MATLAB_ROOT)/rtw/c/src/ext_mode/tcpip \
	-I$(MATLAB_ROOT)/rtw/c/src/ext_mode/serial \
	-I$(MATLAB_ROOT)/rtw/c/src/ext_mode/custom

# DSP Blockset non-TLC S-fcn source path
# and additional file include paths
DSP_MEX      = $(MATLAB_ROOT)/toolbox/dspblks/dspmex
DSP_SIM      = $(MATLAB_ROOT)/toolbox/dspblks/src/sim
DSP_RT       = $(MATLAB_ROOT)/toolbox/dspblks/src/rt
DSP_INCLUDES = \
	-I$(DSP_SIM) \
	-I$(DSP_RT)

QNX_TGT_INCLUDES = -I$(QNX_MW_ROOT) -I$(QNX_MW_ROOT)/ext_mode -I$(QNX_MW_ROOT)/qnx
BLOCKSET_INCLUDES = $(DSP_INCLUDES) \
                   -I$(MATLAB_ROOT)/toolbox/commblks/commmex

COMPILER_INCLUDES = -I$(QNX_TARGET)/usr/include

INCLUDES = -I. -I..  -I$(RELATIVE_PATH_TO_ANCHOR) $(MODELREF_INC_PATH)  $(ADD_INCLUDES) $(MATLAB_INCLUDES) $(BLOCKSET_INCLUDES) \
           $(COMPILER_INCLUDES) $(USER_INCLUDES) $(SHARED_INCLUDES) $(QNX_TGT_INCLUDES)


#[EOF] qnxtools.mak


