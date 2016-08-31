Version: Comedi Toolbox 1.0

*********************************************************************
* About
*********************************************************************

The Comedi Toolbox for Real-Time Workshop is designed for use with 
Linux based Real-Time Workshop targets such as the LNX target. It 
provides four Simulink blocks, analogue and digital I/O, which 
communicate with the installed I/O devices through the Comedi
interface.

*********************************************************************
* Installation
*********************************************************************

Copy all files to a directory named e.g. "comedi-toolbox/". Put the
"comedi-toolbox/" directory in the Matlab path. The Comedi Toolbox
should now appear in the Simulink Library Browser under Blocksets &
Utilities. The library can also be opened by typing "comedi_lib" in
the Matlab prompt.

The four C files in the "comedi-toolbox" directory must be MEX'ed on
your system. This command is part of the Matlab distribution. Simply
execute the following four commands in the "comedi-toolbox" directory:

mex sfun_comedi_ai.c
mex sfun_comedi_ao.c
mex sfun_comedi_di.c
mex sfun_comedi_do.c

It is necessary to add the following user source to the Make command
in the Real-Time Workshop Configuration Parameters:

Make command: make_lnx USER_SRCS="comedi_lib.c"

When compiling the system Comedi library must be linked in. In the LNX
target this is easily done in the Real-Time Workshop options specific
for the LNX target. Just add "-lcomedi" to the Linker Options where it
already says "-lm" by default.

*********************************************************************
* Compatibility
*********************************************************************

Comedi Toolbox 1.0 is tested with Matlab 7.4 (2007a) for Linux.

Version details:
LNX 2.3
Matlab 7.4.0.336 (R2007a)
Simulink 6.6
Real-Time Workshop 6.6

*********************************************************************
* Releases
*********************************************************************

Version 1.00:
- Initial release.

*********************************************************************
* Author
*********************************************************************

Dan D. V. Bhanderi, 2007
dan@bhanderi.dk
