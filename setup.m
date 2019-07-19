% Run this script to install the RPI target
% Author : jacques.gangloff@unistra.fr, July 2019

clear;
clc;

global rpitdir;
rpitdir = pwd;

disp( 'C O N F I G U R A T I O N    O F    R P I t' );
disp( '===========================================' );

disp( ' ' );

rpit_message({...
'IMPORTANT NOTES:';...
'  - In what follows, ''target'' means the distant Linux system you want';...
'    to prepare for RPIt. The system type is automatically detected.';...
'  - If the target is a Raspberry Pi, the setup should go without problem.';...
'    In this case, some specific configurations will be done.';...
'  - If the target is a Debian-based distribution (Debian, Ubuntu),';...
'    you should first add a user ''pi'' to the system and add it to the';...
'    passwordless sudoers (sudo without password prompt with user ''pi'').';...
'    To do so, create an account ''pi'' with ''sudo adduser pi''';...
'    and issue the following command as root: ';...
'    ''echo "pi ALL=(ALL) NOPASSWD:ALL" >';...
'      /etc/sudoers.d/00-RPIt;chmod 0440 /etc/sudoers.d/00-RPIt''';...
'    You should also install some basic additional packages by running:';...
'    ''sudo apt-get install build-essential ssh''';...
'  - If the target is not Debian-based, you cannot use it with RPIt.';});

cont_quest = input( '  > Continue with the setup (''y'' to continue or ''n'' to abort) ? ', 's' );
if strcmp( cont_quest, 'y' )
  disp( '  > Starting RPIt setup.' );
else
  if strcmp( cont_quest, 'n' )
    disp( '  > Aborting RPIt setup.' );
    clear;
    return;
  else
    rpit_error( 'Unrecognized answer. Aborting RPIt setup.' );
    clear;
    return;
  end
end

% Add target paths to matlab paths
addpath( [ rpitdir '/blocks' ] );		% Path to the blockset
addpath( [ rpitdir '/rpit' ] );			% Path to target files
up = userpath;                      % Get userpath
if isempty(up)                      % Check for an empty userpath
  userpath('reset');                % Reset userpath to default
end
up = regexprep(up,';','');          % Remove semicolon at end of userpath
status = savepath( [ up filesep 'pathdef.m' ] );
if status
  rpit_error( 'Could not save the Matlab search path for future sessions.' );
  clear;
  return;
else
  disp( '  > Saving Matlab search path in user path for future sessions.' );
  if exist( [ up filesep 'startup.m' ], 'file' ) == 2
    startup_content = fileread( [ up filesep 'startup.m' ] );
    if contains( startup_content, 'path(pathdef);' )
      disp( '  > ''startup.m'' already has a ''path(pathdef);'' command. Skipping.' );
    else
      rpit_warning( '''startup.m'' found in your userpath. Please add manually ''path(pathdef);'' to ''startup.m''.' );
    end
  else
    fid = fopen( [ up filesep 'startup.m' ], 'wt' );
    if fid ~= -1
      fprintf( fid, 'path(pathdef);\n' );
      fclose(fid);
    else
       rpit_warning( 'Unable to create ''startup.m''. Please add manually ''path(pathdef);'' to ''startup.m''.' );
    end
  end
end
path(pathdef);

% Check for compatible matlab version
mvernum = version( '-release' );
if strcmp( mvernum, '2018b' )
  disp( '  > Supported Matlab version detected.' );
else
  disp( '  > Supported releases: 2018b.' );
  rpit_error( 'This release of Matlab is currently not supported.' );
  clear;
  return;
end

% Compile s-functions mex files to generate executables for the host
disp( '  > Checking mex configuration.' );
cc = mex.getCompilerConfigurations( 'C', 'Selected' );
if isempty(cc)
  rpit_error( {
    'mex not configured.';...
    'Please run ''mex -setup'' before running this setup again.'} );
  clear;
  return;
else
  disp( '  > mex is already configured for C language.' );
end

disp( '  > Compiling S-functions mex files (you can safely ignore warnings).' );
cd( [ rpitdir '/blocks' ] );
mex -silent rpi_sfun_time.c rpi_sfun_time_wrapper.c;
mex -silent rpi_sfun_ev3.c rpi_sfun_ev3_wrapper.c;
mex -silent rpi_sfun_imu.c rpi_sfun_imu_wrapper.c;
mex -silent rpi_sfun_mpu9150.c rpi_sfun_mpu9150_wrapper.c;
mex -silent rpi_sfun_nxt.c rpi_sfun_nxt_wrapper.c;
mex -silent rpi_sfun_cpu.c rpi_sfun_cpu_wrapper.c;
mex -silent rpi_sfun_ev314.c rpi_sfun_ev314_wrapper.c;
mex -silent rpi_sfun_polaris.c rpi_sfun_polaris_wrapper.c;
mex -silent rpi_sfun_iosocket.c rpi_sfun_iosocket_wrapper.c;
mex -silent rpi_sfun_dxl.c rpi_sfun_dxl_wrapper.c;
mex -silent rpi_sfun_js.c rpi_sfun_js_wrapper.c;
mex -silent rpi_sfun_trex.c rpi_sfun_trex_wrapper.c;
mex -silent rpi_sfun_rgpio.c rpi_sfun_rgpio_wrapper.c;
mex -silent rpi_sfun_teensyshot.c rpi_sfun_teensyshot_wrapper.c;

%
% PC platform specific configuration
%
if ispc

  % Configure a passwordless putty ssh connection
  
  cd( [ rpitdir '/tools' ] );
  disp( '  > Generating a public/private RSA pair for ssh level 2.' );

  while 1
    [ ~, out ] = system( 'dir' );
    if contains( out, 'public_key' )
      if contains( out, 'private_key.ppk' )
        disp( '  > ''public_key'' and ''private_key.ppk'' already generated. Skipping.' );
        break;
      end
    end
    disp( '  > Save the public key in the current directory as ''public_key''.' );
    disp( '  > Save the private key passwordless in the current directory as ''private_key''.' );
    disp( '  > Exit the key generator.' );
    pause( 1 );
    command = './puttygen';
    [ ~, ~ ] = system( command );
    [ ~, out ] = system( 'dir' );
    if contains( out, 'public_key' ) 
      if contains( out, 'private_key.ppk' )
        disp( '  > ''public_key'' and ''private_key.ppk'' successfully generated.' );
        break;
      else
        disp( '  > ''private_key.ppk'' not found in current directory.' );
        disp( '  > Please try again.' );
      end
    else
      disp( '  > ''public_key'' not found in current directory.' );
      disp( '  > Please try again.' );
    end
  end

  piip = input( '  > Enter IP address of the target : ', 's' );
  disp( '  > Enter password of the ''pi'' account on the target: ' );
  while 1
    pipwd = passwordUI();
    if ~isempty(pipwd)
      break;
    else
      disp( '  > Please enter a non-empty password: ' );
    end
  end

  % Check if the target is responding

  disp( '  > Answer ''y'' if asked for storing the key in the cache in the cmd window.' );
  command = sprintf( 'start /WAIT plink -pw %s pi@%s pwd', pipwd, piip );
  %command = sprintf( 'plink -batch -pw %s pi@%s pwd', pipwd, piip );
  [ status, ~ ] = system( command );
  if status
    rpit_error( 'Unable to connect to the target passwordlessly.' );
    clear;
    return;
  end
  
  % Create .ssh folder and define its permission

  command = sprintf( 'plink -pw %s pi@%s mkdir .ssh', pipwd, piip );
  [ ~, ~ ] = system( command );
  command = sprintf( 'plink -pw %s pi@%s chmod 700 .ssh', pipwd, piip );
  [ ~, ~ ] = system( command );
  
  % Upload public key to the target
  
  disp( '  > Uploading public key to the target.' );
  command = sprintf( 'pscp -pw %s public_key pi@%s:.', pipwd, piip );
  [ ~, ~ ] = system( command );
  disp( '  > Verifying the authorized keys list.' );
  command = sprintf( 'plink -pw %s pi@%s "cat .ssh/authorized_keys"', pipwd, piip );
  [ ~, out ] = system( command );
  command = sprintf( 'plink -pw %s pi@%s "ssh-keygen -i -f public_key"', pipwd, piip );
  [ ~, public_key ] = system( command );
  if contains( out, public_key )
    disp( '  > Public key already known by the target.' );
  else
    disp( '  > Adding publig key to the authorized keys list.' );
    command = sprintf( 'plink -pw %s pi@%s "ssh-keygen -i -f public_key >> .ssh/authorized_keys"', pipwd, piip );
    [ ~, ~ ] = system( command );
    command = sprintf( 'plink -pw %s pi@%s chmod 640 .ssh/authorized_keys', pipwd, piip );
    [ ~, ~ ] = system( command );
  end
  command = sprintf( 'plink -pw %s pi@%s "rm public_key"', pipwd, piip );
  [ ~, ~ ] = system( command );
  
  % Checking passwordless connection
  
  disp( '  > Checking the passworless connection [CRTL-C if hanging]...' );
  command = sprintf( 'plink -i private_key.ppk pi@%s pwd', piip );
  [ ~, out ] = system( command );
  if contains( out, '/home/pi' )
    disp( '  > Passwordless configuration successfully operating.' );
  else
    disp( '  > Unable to connect to the target passwordlessly. Try to configure manually:' );
    disp( '   - on the host: run puttygen.exe with default settings.' );
    disp( '   - on the host: save public key in putty.exe folder and name it ''public_key''.' );
    disp( '   - on the host: save private key without passphrase in putty.exe folder and name it ''private_key.ppk''.' );
    disp( '   - on the target: import public key and run ''ssh-keygen -i -f public_key >> ~/.ssh/authorized_keys''' );
    rpit_error( 'Unable to connect to the target passwordlessly.' );
    clear;
    return;
  end
  
  % Defining ssh commands
  
  ssh_command = 'plink -i private_key.ppk';
  scp_command = 'pscp -i private_key.ppk';
  
end % End of PC configuration

%
% UNIX platform specific configuration
%
if isunix
  
  % Configure a passwordless ssh connection
  
  cd( [ rpitdir '/tools' ] );
  disp( '  > Generating a public/private RSA pair for ssh level 2.' );
  
  [ ~, out ] = system( 'ls' );
  if  ~contains( out, 'key' )  ||  ~contains( out, 'key.pub' ) 
    [ status, ~ ] = system( 'LD_LIBRARY_PATH=;ssh-keygen -t rsa -N '''' -f key' );
    if ( status )
      rpit_error( 'Unable to generate private and public key. Check your ssh installation.' );
      clear;
      return;
    end
  else
    disp( '  > ''key'' and ''key.pub'' already generated. Skipping.' );
  end
  [ ~, ~ ] = system( 'chmod 600 key' );

  piip = input( '  > Enter IP address of the target : ', 's' );
  disp( '  > Enter password of the pi account on your target when prompted.' );
  command = sprintf( 'LD_LIBRARY_PATH=;scp -q key.pub install_key.sh pi@%s:.', piip );
  system( command );
  disp( '  > Enter password of the pi account on your target again please.' );
  command = sprintf( 'LD_LIBRARY_PATH=;ssh pi@%s "bash install_key.sh"', piip );
  system( command );
  
  % Checking passwordless connection
  
  disp( '  > Checking passwordless ssh connection [CRTL-C if hanging]...' );
  command = sprintf( 'LD_LIBRARY_PATH=;ssh  -i key pi@%s "pwd"', piip );
  [ ~, out ] = system( command );
  if contains( out, '/home/pi' )
    disp( '  > Passwordless connection to the target established.' );
  else
    disp( '  > Unable to connect to the target. Troubleshout the cmd.' );
    disp( ['  > Command: ' command ] );
    disp( '  > And try setup again.' );
    rpit_error( 'Unable to connect to the target.' );
    clear;
    return;
  end
  
  % Defining ssh commands
  
  ssh_command = 'LD_LIBRARY_PATH=;ssh -i key';
  scp_command = 'LD_LIBRARY_PATH=;scp -i key';
  
end % End of UNIX configuration
  
% Check if the target is a RPI

command = sprintf( '%s pi@%s sudo cat /etc/os-release', ssh_command, piip );
[ ~, out ] = system( command );
if  ~contains( out, 'debian' )
  rpit_error( 'Distant target is not Debian-based. Aborting.' );
  clear;
  return;
else
  if  ~contains( out, 'raspbian' ) 
    disp( '  > Distant target is Debian-based but not a RPI.' );
    disp( '  > Checking if distant target is ARM or x86.' );
    command = sprintf( '%s pi@%s sudo uname -m', ssh_command, piip );
    [ ~, out ] = system( command );
    if  ~contains( out, 'arm' ) 
      if  ~contains( out, 'x86' ) 
        rpit_warning( 'Unrecognized platform. Defaulting to ARM setup.' );
        copyfile('../res/rpi_callback_handler_arm.m','../rpit/rpi_callback_handler.m');
        copyfile('../res/ert_rpi_2014a_arm.tmf','../rpit/ert_rpi.tmf');
      else
        disp( '  > Updating the TMF for x86 gcc optimizations.' );
        copyfile('../res/rpi_callback_handler_x86.m','../rpit/rpi_callback_handler.m');
        copyfile('../res/ert_rpi_2014a_x86.tmf','../rpit/ert_rpi.tmf');
      end
    else
      disp( '  > Updating the TMF for ARM gcc optimizations.' );
      copyfile('../res/rpi_callback_handler_arm.m','../rpit/rpi_callback_handler.m');
      copyfile('../res/ert_rpi_2014a_arm.tmf','../rpit/ert_rpi.tmf');
    end
    target_is_rpi = 0;
  else
    command = sprintf( '%s pi@%s cat /proc/device-tree/model', ssh_command, piip );
    [ ~, out ] = system( command );
    disp( ['  > Distant target is a ', out, '.'] );
    if  contains( out, 'Raspberry Pi 3' )
      disp( '  > Optimizing compiler flags for a RPI 3.' );
      copyfile('../res/rpi_callback_handler_arm_pi3.m','../rpit/rpi_callback_handler.m');
      copyfile('../res/ert_rpi_2014a_arm_pi3.tmf','../rpit/ert_rpi.tmf');
    elseif contains( out, 'Raspberry Pi 4' )
      disp( '  > Optimizing compiler flags for a RPI 4.' );
      copyfile('../res/rpi_callback_handler_arm_pi4.m','../rpit/rpi_callback_handler.m');
      copyfile('../res/ert_rpi_2014a_arm_pi4.tmf','../rpit/ert_rpi.tmf');
    else
      disp( '  > Optimizing compiler flags for a RPI 1 and 2.' );
      copyfile('../res/rpi_callback_handler_arm.m','../rpit/rpi_callback_handler.m');
      copyfile('../res/ert_rpi_2014a_arm.tmf','../rpit/ert_rpi.tmf');
    end
    target_is_rpi = 1;
  end
end

% Synchronizing timezones

disp( '  > Synchronizing timezones.' );
time_data = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z');
local_tz = time_data.TimeZone;
command = sprintf( '%s pi@%s "sudo timedatectl set-timezone %s"', ssh_command, piip, local_tz );
[ ~, ~ ] = system( command );

% Copy MATLAB files to the target

disp( '  > Copying MATLAB files (may take a while).' );
command = sprintf( '%s pi@%s "rm -r MATLAB;mkdir MATLAB;mkdir MATLAB/simulink;mkdir -p MATLAB/toolbox/coder/rtiostream/src"', ssh_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB', scp_command, [ '"' matlabroot '/extern' '"' ], piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB', scp_command, [ '"' matlabroot '/rtw' '"' ], piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB/simulink', scp_command, [ '"' matlabroot '/simulink/include' '"' ], piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB/simulink', scp_command, [ '"' matlabroot '/simulink/src' '"' ], piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB/toolbox/coder/rtiostream/src', scp_command, [ '"' matlabroot '/toolbox/coder/rtiostream/src/utils' '"' ], piip );
[ ~, ~ ] = system( command );

% Copy S-function sources to the target

disp( '  > Uploading S-Function files.' );
command = sprintf( '%s -r ../blocks pi@%s:./MATLAB', scp_command, piip );
[ ~, ~ ] = system( command );

% Create the target working directory

disp( '  > Creating a working directory ''RTW'' in user ''pi'' home dir.' );
command = sprintf( '%s pi@%s "rm -r RTW;mkdir RTW"', ssh_command, piip );
[ ~, ~ ] = system( command );

% Copy Polaris rom file to the target into the RTW dir

disp( '  > Uploading Polaris rom file.' );
command = sprintf( '%s -r ../res/polaris.rom pi@%s:./RTW', scp_command, piip );
[ ~, ~ ] = system( command );

% Check internet access
disp( '  > Checking internet access on the target.' );
command = sprintf( '%s pi@%s wget -q --spider http://google.com', ssh_command, piip );
[ status, ~ ] = system( command );
if ( status )
  rpit_error( 'Target is offline. Connect it to internet and rerun setup.' );
  clear;
  return;
else
  disp( '  > Target is online. Installing addtional packages' );
end

% Install some additional packages on the target

disp( '  > Installing the ''screen'' utility on the target.' );
command = sprintf( '%s pi@%s sudo apt-get -y install screen', ssh_command, piip );
[ status, ~ ] = system( command );
if ( status )
  rpit_warning( 'Installation of ''screen'' returned an error.' );
  disp( '  > Is your target connected to internet ?' );
  disp( '  > Please install it manually: sudo apt-get install screen' );
end
disp( '  > Installing the ''libusb-1.0-0-dev'' package on the target.' );
command = sprintf( '%s pi@%s sudo apt-get -y install libusb-1.0-0-dev', ssh_command, piip );
[ status, ~ ] = system( command );
if ( status )
  rpit_warning( 'Installation of ''libusb-1.0-0-dev'' returned an error.' );
  disp( '  > Is your target connected to internet ?' );
  disp( '  > Please install it manually: sudo apt-get install libusb-1.0-0-dev' );
end

% Enable user access to serial port

disp( '  > Enabling user serial port access by adding pi to the dialout group.' );
command = sprintf( '%s pi@%s sudo usermod -a -G dialout $USER', ssh_command, piip );
[ ~, ~ ] = system( command );

% Enable user access to the EV3 USB port on the target

disp( '  > Enabling user access to the EV3 USB port.' );
command = sprintf( '%s ../res/99-EV3.rules pi@%s:.', scp_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo mv ./99-EV3.rules /etc/udev/rules.d"', ssh_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo chown root.root /etc/udev/rules.d/99-EV3.rules"', ssh_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo chmod 644 /etc/udev/rules.d/99-EV3.rules"', ssh_command, piip );
[ status, ~ ] = system( command );
if ( status )
  disp( '  > Creation of /etc/udev/rules.d/99-EV3.rules returned an error.' );
  disp( '  > Please install this manually: ' );
  disp( command );
end

% Enable user access to the NXT USB port on the target

disp( '  > Enabling user access to the NXT USB port.' );
command = sprintf( '%s ../res/99-NXT.rules pi@%s:.', scp_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo mv ./99-NXT.rules /etc/udev/rules.d"', ssh_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo chown root.root /etc/udev/rules.d/99-NXT.rules"', ssh_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo chmod 644 /etc/udev/rules.d/99-NXT.rules"', ssh_command, piip );
[ status, ~ ] = system( command );
if ( status )
  rpit_warning( 'Creation of /etc/udev/rules.d/99-NXT.rules returned an error.' );
  disp( '  > Please install this manually: ' );
  disp( command );
end

% Reduce latency on FTDI chipsets

disp( '  > Reduce latency of FTDI chipsets.' );
command = sprintf( '%s ../res/99-dynamixelsdk-usb.rules pi@%s:.', scp_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo mv ./99-dynamixelsdk-usb.rules /etc/udev/rules.d"', ssh_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo chown root.root /etc/udev/rules.d/99-dynamixelsdk-usb.rules"', ssh_command, piip );
[ ~, ~ ] = system( command );
command = sprintf( '%s pi@%s "sudo chmod 644 /etc/udev/rules.d/99-dynamixelsdk-usb.rules"', ssh_command, piip );
[ status, ~ ] = system( command );
if ( status )
  rpit_warning( 'Creation of /etc/udev/rules.d/99-dynamixelsdk-usb.rules returned an error.' );
  disp( '  > Please install this manually: ' );
  disp( command );
end

if ( target_is_rpi )

  % Setup i2c subsystem on the RPI at full speed

  disp( '  > Installing the i2c utilities on the RPI.' );
  command = sprintf( '%s pi@%s sudo apt-get -y install i2c-tools libi2c-dev', ssh_command, piip );
  [ status, ~ ] = system( command );
  if ( status )
    rpit_warning( 'Installation of i2c utilities returned an error.' );
    disp( '  > Is your RPI connected to internet ?' );
    disp( '  > Please install it manually: sudo apt-get install i2c-tools libi2c-dev' );
  end
  disp( '  > Configuring i2c at 400kHz.' );
  command = sprintf( '%s pi@%s "sudo cat /boot/config.txt"', ssh_command, piip );
  [ ~, out ] = system( command );
  if contains( out, 'dtparam=i2c_arm_baudrate=400000' )
    disp( '  > /boot/config.txt already up to date.' );
  else
    disp( '  > Adding ''dtparam=i2c_arm_baudrate=400000'' at the end of  /boot/config.txt.' );
    command = sprintf( '%s pi@%s "sudo bash -c ''echo dtparam=i2c_arm_baudrate=400000 >> /boot/config.txt''"', ssh_command, piip );
    [ ~, ~ ] = system( command );
  end
  disp( '  > NOTE: if needed, use raspi-config to enable i2c.' );
  disp( '  > Adding user pi to the i2c group.' );
  command = sprintf( '%s pi@%s "sudo usermod -a -G i2c pi"', ssh_command, piip );
  [ ~, ~ ] = system( command );
  
  % Enabling non-root access to GPIO
  
  disp( '  > Adding user pi to the gpio group.' );
  command = sprintf( '%s pi@%s "sudo usermod -a -G gpio pi"', ssh_command, piip );
  [ ~, ~ ] = system( command );

  disp( '  > Checking cpu governor configuration.' );
  command = sprintf( '%s pi@%s "sudo cat /etc/rc.local"', ssh_command, piip );
  [ status, out ] = system( command );
  if contains( out, '# Change governor to performance' )
    disp( '  > rc.local already set cpu governor to performance mode.' );
  else
    disp( '  > Force cpu governor to performance mode in rc.local.' );
    command = sprintf( '%s pi@%s %s', ssh_command, piip, '"sudo sed -i -e ''s/^exit 0/# Change governor to performance\nfor cpucore in \/sys\/devices\/system\/cpu\/cpu?; do echo performance | sudo tee `echo ''$cpucore''`\/cpufreq\/scaling_governor > \/dev\/null; done\n\nexit 0/g'' /etc/rc.local"' );
    [ status, out ] = system( command );
  end
end
  
disp( '  > Configuration of RPIt successfully completed.' );
disp( '  > NOTE: please reboot your target for changes to take effect.' );

% Restore initial path and cleanup variables

cd( rpitdir );
clear;

function rpit_error( msg )
  global rpitdir;
  
  cd( rpitdir );
  waitfor(errordlg( msg, 'RPIt error' ));
end

function rpit_warning( msg )
  waitfor(warndlg( msg, 'RPIt warning' ));
end

function rpit_message( msg )
  waitfor(msgbox( msg, 'RPIt message' ));
end