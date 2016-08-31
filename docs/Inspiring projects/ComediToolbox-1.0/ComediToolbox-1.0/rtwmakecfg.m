function makeInfo=rtwmakecfg()
% RTWMAKECFG adds include and source directories to rtw make files. 
% makeInfo=RTWMAKECFG returns a structured array containing build info. 
% Please refer to the rtwmakecfg API section in the Real-Time workshop 
% Documentation for details on the format of this structure. 

% Dan Bhanderi, 2007
% $Id: rtwmakecfg.m,v 1.4 2007/08/01 16:42:55 danji Exp $

% Get hold of the fullpath to this file, without the filename itself
rootpath = rtw_alt_pathname(fileparts(mfilename('fullpath')));

% Comedi block source path for sfun_comedi_*.c
makeInfo.includePath{1} = rootpath;

% Comedi block source comedi_lib.c
makeInfo.source{1} = 'comedi_lib.c';

makeInfo.precompile = 1;

return;
