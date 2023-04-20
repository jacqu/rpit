function ert_rpi_make_rtw_hook(hookMethod,modelName,rtwroot,templateMakefile,buildOpts,buildArgs)
% ERT_MAKE_RTW_HOOK - This is the standard ERT hook file for the RTW build
% process (make_rtw), and implements automatic configuration of the
% models configuration parameters.  When the buildArgs option is specified
% as 'optimized_fixed_point=1' or 'optimized_floating_point=1', the model
% is configured automatically for optimized code generation.
%
% This hook file (i.e., file that implements various RTW callbacks) is
% called by RTW for system target file ert.tlc.  The file leverages
% strategic points of the RTW process.  A brief synopsis of the callback
% API is as follows:
%
% ert_make_rtw_hook(hookMethod, modelName, rtwroot, templateMakefile,
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
%   From MATLAB, it's whatever is passed into make_rtw.  For example, its
%   'optimized_fixed_point=1' for make_rtw('optimized_fixed_point=1').
%
%   This file implements these buildArgs:
%     optimized_fixed_point=1
%     optimized_floating_point=1
%
% You are encouraged to add other configuration options, and extend the
% various callbacks to fully integrate ERT into your environment.

% Copyright 1996-2009 The MathWorks, Inc.
% $Revision: 1.1.6.9 $ $Date: 2009/12/28 04:28:42 $
  
  switch hookMethod
   case 'error'
    % Called if an error occurs anywhere during the build.  If no error occurs
    % during the build, then this hook will not be called.  Valid arguments
    % at this stage are hookMethod and modelName. This enables cleaning up
    % any static or global data used by this hook file.
    msg = DAStudio.message('RTW:makertw:buildAborted', modelName);
    disp(msg);
   case 'entry'
    % Called at start of code generation process (before anything happens.)
    % Valid arguments at this stage are hookMethod, modelName, and buildArgs.
    
    % Check if we are in the right directory
    modelPath = which( modelName );
    if ~strcmp( fileparts( modelPath ), pwd )
      % Wrong directory, aborting
      disp( '### Please change current directory to the one containing the model.' );
      error( '### Wrong current directory. Aborting procedure.' );
    end
    
    msg = DAStudio.message('RTW:makertw:enterRTWBuild', modelName);
    disp(msg);
    
    option = LocalParseArgList(buildArgs);
    
    if ~strcmp(option,'none')
        ert_unspecified_hardware(modelName);
        cs = getActiveConfigSet(modelName);
        cscopy = cs.copy;
        ert_auto_configuration(modelName,option);
        locReportDifference(cscopy, cs);
    end
    
    % Suppress warning: saving external mode simulation output as a single object is not supported
    set_param(modelName,'ReturnWorkspaceOutputs','off');
    
   case 'before_tlc'
    % Called just prior to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
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
    % Called after make process is complete. All arguments are valid at 
    % this stage.
    
    % Define some parameters
    
    rtwdir = pwd;
    fpath = which( mfilename() );
    [ rpitdir,  ~ ] = fileparts( fpath );
    cd( rpitdir );cd  ..;
    rpidir = pwd;
    rpidir = [ '"' rpidir '"' ];
    cd( rtwdir );
    
    % Get and check the target IP address
    
    piip = get_param( modelName, 'ExtModeMexArgs' );
    quotes_in_piip = strfind(piip,'''');
    size_quotes_in_piip = size( quotes_in_piip );
    size_quotes_in_piip = size_quotes_in_piip( 2 );
    if ( size_quotes_in_piip ~= 2 )
      error('### Error: target IP address mispelled. Try put quotes around.');
    else
      piip = piip( quotes_in_piip(1)+1:quotes_in_piip(2)-1 );
      disp(['### Target IP address: ' piip]);
    end
    
    %
    % PC specific commands
    %
    if ispc
      plinkexe = [ rpidir '\tools\plink.exe' ];
      pscpexe = [ rpidir '\tools\pscp.exe' ];
      pip = [ rpidir '\tools\private_key.ppk' ];
      ssh_command = [ plinkexe ' -i ' pip ];
      scp_command = [ pscpexe ' -P 22 -i ' pip ];
    end
    
    %
    % UNIX specific commands
    %
    if isunix
      pip = [ rpidir '/tools/key' ];
      ssh_command = [ 'LD_LIBRARY_PATH=;ssh -i ' pip ];
      scp_command = [ 'LD_LIBRARY_PATH=;scp -i ' pip ];
    end
    

    % Check if the target responds
    command = sprintf('%s pi@%s pwd', ssh_command, piip );
    [ ~, out ] = system( command );
    if contains( out, '/home/pi' )

      % Check if the target is a RPI. If yes, synchronize the clocks.
      command = sprintf( '%s pi@%s sudo cat /etc/os-release', ssh_command, piip );
      [ ~, out ] = system( command );
      if  ~contains( out, 'raspbian' ) 
        disp( '### Target is not a Raspberry Pi.' );
      else
        disp( '### Target is a Raspberry Pi.' );
        
        % Synchronize target clock with the host clock
        disp( '### Synchronizing host and target clocks.' );
        curr_time = datestr( now, 'mmmm dd yyyy HH:MM:SS.FFF AM' );
        command = sprintf( '%s pi@%s sudo date --set=''%s''', ssh_command, piip, curr_time );
        [ ~, ~ ] = system( command );
      end
      
      % Kill any pending "rpi" process
      disp( '### Kill any pending rtw process.' );
      command = sprintf( '%s pi@%s killall -9 rpi', ssh_command, piip );
      [ ~, ~ ] = system( command );

      % Send the model directory to the target
      disp(['### Uploading ', modelName, ' to the target.']);
      cd ..;
      command = sprintf('%s -r -p * pi@%s:./RTW', scp_command, piip );
      [ ~, ~ ] = system( command );
      cd( rtwdir );

      % Compile the model on the target
      disp(['### Compiling ', modelName, ' on the target (may take awhile).']);
      command = sprintf('%s pi@%s "cd ~/RTW/%s_ert_rtw;make -j -f %s.mk"', ssh_command, piip, modelName, modelName );
      [ ~, out ] = system( command );
      disp( out );
      if contains( out, '### Created executable' )

        disp( '### Compilation successful. Starting model.' );

        % Rename the executable
        command = sprintf( '%s pi@%s mv RTW/%s RTW/rpi', ssh_command, piip, modelName );
        [ ~, ~ ] = system( command );

        % Start the executable within screen
        command = sprintf( '%s pi@%s "rm -f RTW/rpi.log"', ssh_command, piip );
        [ ~, ~ ] = system( command );
        command = sprintf( '%s pi@%s screen -h 32 -dmS RPIt RTW/rpi -tf inf -w', ssh_command, piip );
        [ ~, ~ ] = system( command );

        % Check if the model is started: check if socket is open
        rpi_start_cnt = 0;
        while 1
          pause(1);
          command = sprintf( '%s pi@%s sudo netstat -lntup', ssh_command, piip );
          [ ~, out ] = system( command );
          if contains( out, '17725' ) && contains( out, 'rpi' )
            disp('### Real-time code successfully started on the target.');
            % Renice the rpi process
            command = sprintf( '%s pi@%s "sudo renice -5 -p `ps -eo pid,comm | awk ''/rpi$/  {print $1; exit}''`"', ssh_command, piip );
            [ ~, ~ ] = system( command );
            % Autoatically start external mode
            disp('### Starting external mode...');
            set_param(modelName, 'SimulationMode', 'external');
            set_param(modelName, 'SimulationCommand', 'connect');
            set_param(modelName, 'SimulationCommand', 'start');
            disp('### Simulink started and running in external mode.');
            disp('### Target program output:');
            command = sprintf('%s pi@%s "screen -S RPIt -p 0 -X hardcopy RTW/rpi.log;head RTW/rpi.log"', ssh_command, piip );
            [ ~, out ] = system( command );
            disp( out );
            break;
          end
          rpi_start_cnt = rpi_start_cnt + 1;
          if rpi_start_cnt == 10
            error('### Target program did not start: did you install ''screen'' on the target ?');
          end 
        end
      else
        error('### Compilation failed. Check the errors and tweak file ''ert_rpi.tmf''');
      end
    else
      disp('    Did you enter the correct address in Real-Time Workshop->Interface->Mex file arguments ?');
      disp('    Example : ''192.168.1.50'' with the single quotes around.');
      disp('    Did you configure a passworless putty session into the target ?');
      disp('    See the comment at the end of ''setup.m''.');
      error('### Error: target unreachable.');
    end
    
   case 'exit'
    % Called at the end of the RTW build process.  All arguments are valid
    % at this stage.
    if strcmp(get_param(modelName,'GenCodeOnly'),'off')
        msgID = 'RTW:makertw:exitRTWBuild';
    else
        msgID = 'RTW:makertw:exitRTWGenCodeOnly';
    end
    msg = DAStudio.message(msgID,modelName);
    disp(msg);
  end


% Simple parse function to find:
%   optimized_fixed_point=1
%   optimized_floating_point=1
function option = LocalParseArgList(args)
  
  if contains(args,'optimized_fixed_point=1')
    option = 'optimized_fixed_point';
  elseif contains(args,'optimized_floating_point=1')
    option = 'optimized_floating_point';
  else
    option = 'none';
  end

% local function: report difference between the configuration set settings
% before and after running auto-configuration script.  
function locReportDifference(cs1, cs2)
    [iseq, diffs] = slprivate('diff_config_sets', cs1, cs2, 'string');
    if ~iseq
        msg = DAStudio.message('RTW:makertw:incompatibleParamsUpdated', diffs);
        summary = DAStudio.message('RTW:makertw:autoconfigSummary');
        rtwprivate('rtw_disp_info',...
                   get_param(cs2.getModel, 'Name'),...
                   summary,...
                   msg);
    end

