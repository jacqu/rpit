function lrt_make_rtw_hook(hookMethod, modelName, rtwroot, templateMakefile, buildOpts, buildArgs, buildInfo) %#ok<INUSL>
% AUTOSAR_MAKE_RTW_HOOK - This is the standard ERT hook file for the RTW build
% process (make_rtw).
%
% This hook file (i.e., file that implements various RTW callbacks) is
% called by RTW for system target file ert.tlc.  The file leverages
% strategic points of the RTW process.  A brief synopsis of the callback
% API is as follows:
%
% autosar_make_rtw_hook(hookMethod, modelName, rtwroot, templateMakefile,
%                   buildOpts, buildArgs)
%
% hookMethod:
%   Specifies the stage of the RTW build process.  Possible values are
%   entry, before_tlc, after_tlc, before_make, after_make and exit, etc.
%
% modelName:
%   Name of model.  Valid for all stages.
%
% rtwroot:
%   Reserved.
%
% templateMakefile:
%   Name of template makefile.  Valid for stages 'before_make' and 'exit'.
%
% buildOpts:
%   Valid for stages 'before_make' and 'exit', a MATLAB structure
%   containing fields
%
%   modules:
%     Char array specifying list of generated C files: model.c, model_data.c,
%     etc.
%
%   codeFormat:
%     Char array containing code format: 'RealTime', 'RealTimeMalloc',
%     'Embedded-C', and 'S-Function'
%
%   noninlinedSFcns:
%     Cell array specifying list of non-inlined S-Functions.
%
%   compilerEnvVal:
%     String specifying compiler environment variable value, e.g.,
%     D:\Applications\Microsoft Visual
%
% buildArgs:
%   Char array containing the argument to make_rtw.  When pressing the build
%   button through the Configuration Parameter Dialog, buildArgs is taken
%   verbatim from whatever follows make_rtw in the make command edit field.
%   From MATLAB, it's whatever is passed into make_rtw.  For example, itsmodelName= deblank(s(5:end));
%   'optimized_fixed_point=1' for make_rtw('optimized_fixed_point=1').
%
%   This file implements these buildArgs:
%     optimized_fixed_point=1
%     optimized_floating_point=1
%
% You are encouraged to add other configuration options, and extend the
% various callbacks to fully integrate ERT into your environment.

% Copyright 2007 The MathWorks, Inc.
% $Revision: 1.1.6.3 $ $Date: 2008/01/15 18:56:12 $
global GenCodeOnlyOrig
global epath
global lpath
global gpath
global npath
global mcmd


  switch hookMethod
   case 'error'
    % Called if an error occurs anywhere during the build.  If no error occurs
    % during the build, then this hook will not be called.  Valid arguments
    % at this stage are hookMethod and modelName. This enables cleaning up
    % any static or global data used by this hook file.
    disp(sprintf(['### Lego NXT Real-Time Workshop build procedure for model: %s'
          ' aborted due to an error.', modelName]));
    
   case 'entry'
    % Called at start of code generation process (before anything happens.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs.
    disp(sprintf(['\n### Starting Lego NXT Real-Time Workshop build procedure for ', ...
                  'model: %s'],modelName));
    cs = getActiveConfigSet(modelName);
    set_param(cs,'TemplateMakefile','lrt_ecrobot.tmf');
    set_param(modelName, 'GenCodeOnly', 'on' );
    
    % set up 'ecrobot.mak' path as a make variable for later use
    epath = which('ecrobot.mak');
    epath = strrep(epath,'\ecrobot\ecrobot.mak','');
    epath = regexprep(epath,{'[a-z]:\','\'},{'','/'}, 'ignorecase');
    epath = strcat('ECROBOT_PATH=',epath);
    
    lpath = which('lrt_main.c');
    lpath = strrep(lpath,'\lrt_main.c','');
    lpath = regexprep(lpath,{'[a-z]:\','\'},{'','/'}, 'ignorecase');
    lpath = strrep(lpath,'Progra~1','PROGRA~1');
    lpath = strcat('LRT_PATH=',lpath);
    
    gpath = which('arm-elf-gcc.exe');
    gpath = strrep(gpath,'\bin\arm-elf-gcc.exe','');
    gpath = regexprep(gpath,{'[a-z]:\','\'},{'','/'}, 'ignorecase');
    gpath = strcat('GNUARM_ROOT=/cygdrive/C/',gpath);
    
    npath = which('NeXTTooL.exe');
    npath = strrep(npath,'\NeXTTool.exe','');
    npath = regexprep(npath,{'[a-z]:\','\'},{'','/'}, 'ignorecase');
    npath = strcat('NEXTTOOL_ROOT=/cygdrive/C/',npath);
    
    %Add Cygwin/bin to the System PATH since many of the commands that get executed
    %during Make (not just Make itself) need to be able to find Cygwin/bin.
    %Note1: Cygwin\bin needs to be first in the path, to avoid confusion with
    %Windows commands having the same names as cygwin, eg. sed.exe
    %Note2: setenv only holds during the current Matlab session
    if isempty(mcmd),
        mcmd = which('make.exe');  %assumes cygwin\bin is on the Matlab path
        setenv('PATH', [fileparts(mcmd) ';' getenv('PATH')]);
    end;
    
    cgen = 'integer';

    cgentype = get_param(modelName, 'codeGenType');
    switch cgentype(2)
        case 'n'
            cgen = 'integer';
        case 'l'
            cgen = 'float';
    end

    % floating point - needs implementation and testing. this does not
    % work??
    
    %if isequal(cgen, 'integer')
    %    set_param(cs,'PurelyIntegerCode','on');
    %elseif isequal(cgen, 'float')
    %    set_param(cs,'PurelyIntegerCode','off');
    %end

   case 'before_tlc'
    % Called just prior to invoking TLC Compiler (actual code generation.)/scratchbox/users/tyler/home/tyler/matlab_code
    % Valid arguments at this disp('Not In External Mode');stage are hookMethod, modelName, and
    % buildArgs

   case 'after_tlc'
    % Called just after to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs

   case 'before_make'
    % Called after code generation is complete, and just prior to kicking
    % off make process (assuming code generation only is not selected.)  All
    % arguments are valid at this stage.


   case 'after_make'
    disp(['### Compiled for Lego NXT successfully.']);
    % Convert RTW generated makefile for cygwin's GNU Make
    disp(['### Configuring generated makefile for cygwin...']);
    inid = fopen(sprintf('%s.mk',modelName),'r');
    outid = fopen('Makefile','w+');
    if inid==-1
        error(sprintf('### Failed to open file: %s.mk',modelName));
    end 
    if outid==-1
        error('### Failed to open or create file: Makefile');
    end 
    while ~feof(inid)
        s = fgetl(inid);
        % Convert windows naming conventions to POSIX conventions for
        % GNU make
        if ~isempty(strfind(s, 'MATLAB')) || ~isempty(strfind(s, 'MODULES'))
            s = cygwin_naming(s);
        end       
        fprintf(outid,'%s\n',s);
    end
    fclose(outid);
    fclose(inid);
    
    % Generate the necessary OIL (OSEK Implementation Language) file to
    % build this model.
    disp(sprintf('### Writing %s.oil....', modelName));
    generate_oil(modelName);
    disp(sprintf('### %s.oil written successfully!', modelName));
    
    % Process the model file using a text editor in order to generate the
    % necessary OSEK hook functions.
    inid = fopen(sprintf('%s.mdl',modelName),'r');
    if inid==-1
        error(sprintf('### Failed to open file: %s.mdl',modelName));
    end 
    disp(['### Writing ecrobot_hooks.h...']);
    ecrobot_hooks(inid);
    fclose(inid);
    
    % If 'Build All' is selected, clean up make directory before building.
    makeClean=get_param(modelName, 'cleanMake');
    if makeClean(2)=='n'
        disp(['### Cleaning up make directory...']);
        [status,result] = system(sprintf('%s %s %s %s %s clean', mcmd, epath, lpath, gpath, npath),'-echo');
        if status~=0, error('Build failed during Make clean'); end;
    end
    
    % Build.
    [status,result] = system(sprintf('%s %s %s %s %s all', mcmd, epath, lpath, gpath, npath),'-echo');
    if status~=0, error('Build failed during Make all'); end;
    

   case 'exit'
    % Called at the end of the RTW build process.  All arguments are valid at this stage.
    % If 'Flash to Lego NXT' is selected, flash the program to the brick immediately after compiling.
    flashNXT=get_param(modelName, 'flashNXT');
    if flashNXT(2)=='n'
        fprintf(['\n### Flashing Lego NXT via USB.\n'], modelName);
        [result,status]= nexttool(['-download=' modelName '.rxe']);
        while status==-1,
            userCmd= input('Try again? [y/n]: ','s');
            if isempty(userCmd) || (lower(userCmd(1))~='y'), break; end;
            [result,status]= nexttool(['-download=' modelName '.rxe']);
        end; %while
        [result,status]= nexttool('-listfiles');
        if status==-1, 
            disp('You can download files manually to the NXT using the command: nexttool(''-download=modelName.rxe'');')
            disp('where the modelName.rxe executable file can be found in the .\modelName_lrt_rtw directory')
        else
            disp('Files now resident on the NXT:-');
            disp(result);
        end; %if
    end
	disp(['### Finished']);
    
end