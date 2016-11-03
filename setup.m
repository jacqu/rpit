% Run this script to install the RPI target
% Author : jacques.gangloff@unistra.fr, June 18th 2014

clear;
clc;

rpitdir = pwd;

disp( '*** Configuration of RPIt.' );
disp( '# NOTES:' );
disp( '# In what follows, ''target'' means the distant Linux system which you want' );
disp( '# to prepare for using with RPIt. The system type is automatically detected.' );
disp( '#  - If the target is a Raspberry Pi, the setup should go without a problem.' );
disp( '#    In this case, some specific i2c configurations will be done.' );
disp( '#  - If the target is a Debian-based distribution (Debian, Ubuntu, Mint), ' );
disp( '#    you should first add a user ''pi'' to the system and add it to the' );
disp( '#    passwordless sudoers (sudo without password prompt with user ''pi'').' );
disp( '#    To do so, create an account ''pi'' with ''sudo adduser pi''' );
disp( '#    and issue the following command as root: ' );
disp( '#    ''echo "pi ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/00-RPIt;chmod 0440 /etc/sudoers.d/00-RPIt''' );
disp( '#    You should also install some basic additional packages by running:' );
disp( '#    ''sudo apt-get install build-essential ssh''' );
disp( '#    If you have warnings about libcrypto.so.1.0.0, replace simply Matlab''s' );
disp( '#    version of libcrypto.so.1.0.0 by your system''s version.' );
disp( '#  - If the target is not Debian-based, you cannot use it with RPIt.' );
cont_quest = input( '  > Continue with the setup (''y'' to continue or ''n'' to abort) ? ', 's' );

if strcmp( cont_quest, 'y' )
  disp( '  > Starting RPIt setup.' );
else
  if strcmp( cont_quest, 'n' )
    disp( '  > Aborting RPIt setup.' );
    return;
  else
    disp( '  > Unrecognized answer. Aborting RPIt setup.' );
    return;
  end
end

% Check for compatible matlab version
mvernum = version( '-release' );
if strcmp( mvernum, '2014a' ) || strcmp( mvernum, '2015a' ) || strcmp( mvernum, '2015b' ) || strcmp( mvernum, '2016a' ) || strcmp( mvernum, '2016b' )
  disp( '  > Configuring TLC files for release 2014a, 2015a, 2015b, 2016a and 2016b.' );
  copyfile('res/rpi_mrmain_2014a.tlc','rpit/rpi_mrmain.tlc');
  copyfile('res/rpi_srmain_2014a.tlc','rpit/rpi_srmain.tlc');
  copyfile('res/slblocks_2014a.m','blocks/slblocks.m');
else
  if strcmp( mvernum, '2012a' )
    disp( '  > Configuring TLC files for release 2012a.' );
    copyfile('res/rpi_mrmain_2012a.tlc','rpit/rpi_mrmain.tlc');
    copyfile('res/rpi_srmain_2012a.tlc','rpit/rpi_srmain.tlc');
    copyfile('res/slblocks_2010b.m','blocks/slblocks.m');
  else
    if strcmp( mvernum, '2010b' )
      disp( '  > Configuring TLC files for release 2010b.' );
      copyfile('res/rpi_mrmain_2010b.tlc','rpit/rpi_mrmain.tlc');
      copyfile('res/rpi_srmain_2010b.tlc','rpit/rpi_srmain.tlc');
      copyfile('res/slblocks_2010b.m','blocks/slblocks.m');
    else
      if strcmp( mvernum, '2011a' )
        disp( '  > Configuring TLC files for release 2011a.' );
        copyfile('res/rpi_mrmain_2010b.tlc','rpit/rpi_mrmain.tlc');
        copyfile('res/rpi_srmain_2010b.tlc','rpit/rpi_srmain.tlc');
        copyfile('res/slblocks_2010b.m','blocks/slblocks.m');
      else
        disp( '  > This release of Matlab is currently not supported.' );
        disp( '  > Supported releases: 2010b, 2010b SP1, 2010b SP2, 2011a, 2012a, 2014a, 2015a, 2015b, 2016a, 2016b.' );
        return;
      end
    end
  end
end

% Compile s-functions mex files to generate executables for the host
cd( [ rpitdir '/blocks' ] );
disp( '  > Compiling S-functions mex files (you can safely ignore warnings).' );
mex rpi_sfun_time.c rpi_sfun_time_wrapper.c
mex rpi_sfun_ev3.c rpi_sfun_ev3_wrapper.c
mex rpi_sfun_imu.c rpi_sfun_imu_wrapper.c
mex rpi_sfun_mpu9150.c rpi_sfun_mpu9150_wrapper.c
mex rpi_sfun_nxt.c rpi_sfun_nxt_wrapper.c
mex rpi_sfun_cpu.c rpi_sfun_cpu_wrapper.c 
mex rpi_sfun_ev314.c rpi_sfun_ev314_wrapper.c
mex rpi_sfun_polaris.c rpi_sfun_polaris_wrapper.c
mex rpi_sfun_socket.c rpi_sfun_socket_wrapper.c
mex rpi_sfun_dxl.c rpi_sfun_dxl_wrapper.c
mex rpi_sfun_js.c rpi_sfun_js_wrapper.c

% Add target paths to matlab paths
addpath( [ rpitdir '/blocks' ] );		% Path to the blockset
addpath( [ rpitdir '/rpit' ] );			% Path to target files
up = userpath;                      % Get userpath
up = regexprep(up,';','');          % Remove semicolon at end of userpath
status = savepath( [ up filesep 'pathdef.m' ] );
if status ~= 0,
  disp( '  > Could not save the Matlab search path for future sessions.' );
  cd( rpitdir );
  clear;
  return;
else
  disp( '  > Saving Matlab search path in user path for future sessions.' );
  if strcmp( mvernum, '2015a' ) || strcmp( mvernum, '2015b' )
    disp( '  > WARNING: your Matlab release needs to add ''path(pathdef);'' in ''startup.m'' in your userpath.' );
  end;
end;
path(pathdef);

%
% PC platform specific configuration
%
if ispc

  % Configure a passwordless putty ssh connection
  
  cd( [ rpitdir '/tools' ] );
  disp( '  > Generating a public/private RSA pair for ssh level 2.' );

  while 1;
    [ status, out ] = system( 'dir' );
    if strfind( out, 'public_key' );
      if strfind( out, 'private_key.ppk' );
        disp( '  > ''public_key'' and ''private_key.ppk'' already generated. Skipping.' );
        break;
      end
    end
    disp( '  > Save the public key in the current directory as ''public_key''.' );
    disp( '  > Save the private key passwordless in the current directory as ''private_key''.' );
    disp( '  > Exit the key generator.' );
    pause( 1 );
    command = 'puttygen';
    [ status, out ] = system( command );
    [ status, out ] = system( 'dir' );
    if strfind( out, 'public_key' ) 
      if strfind( out, 'private_key.ppk' );
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
  pipwd = input( '  > Enter password of the ''pi'' account on the target : ', 's' );

  % Clear command window and history to hide password

  clc;
  com.mathworks.mlservices.MLCommandHistoryServices.removeAll;

  % Check if the target is responding

  disp( '  > Answer ''y'' if asked for storing the key in the cache in the cmd window.' );
  command = sprintf( 'start /WAIT plink -pw %s pi@%s pwd', pipwd, piip );
  [ status, out ] = system( command );
  
  % Create .ssh folder and define its permission

  command = sprintf( 'plink -pw %s pi@%s mkdir .ssh', pipwd, piip );
  [ status, out ] = system( command );
  command = sprintf( 'plink -pw %s pi@%s chmod 700 .ssh', pipwd, piip );
  [ status, out ] = system( command );
  
  % Upload public key to the target
  
  disp( '  > Uploading public key to the target.' );
  command = sprintf( 'pscp -pw %s public_key pi@%s:.', pipwd, piip );
  [ status, out ] = system( command );
  disp( '  > Verifying the authorized keys list.' );
  command = sprintf( 'plink -pw %s pi@%s "cat .ssh/authorized_keys"', pipwd, piip );
  [ status, out ] = system( command );
  command = sprintf( 'plink -pw %s pi@%s "ssh-keygen -i -f public_key"', pipwd, piip );
  [ status, public_key ] = system( command );
  if strfind( out, public_key );
    disp( '  > Public key already known by the target.' );
  else
    disp( '  > Adding publig key to the authorized keys list.' );
    command = sprintf( 'plink -pw %s pi@%s "ssh-keygen -i -f public_key >> .ssh/authorized_keys"', pipwd, piip );
    [ status, out ] = system( command );
    command = sprintf( 'plink -pw %s pi@%s chmod 640 .ssh/authorized_keys', pipwd, piip );
    [ status, out ] = system( command );
  end
  command = sprintf( 'plink -pw %s pi@%s "rm public_key"', pipwd, piip );
  [ status, out ] = system( command );
  
  % Checking passwordless connection
  
  disp( '  > Checking the passworless connection [CRTL-C if hanging]...' );
  command = sprintf( 'plink -i private_key.ppk pi@%s pwd', piip );
  [ status, out ] = system( command );
  if strfind( out, '/home/pi' );
    disp( '  > Passwordless configuration successfully operating.' );
  else
    disp( '  > Unable to connect to the target passwordlessly. Try to configure manually:' );
    disp( '   - on the host: run puttygen.exe with default settings.' );
    disp( '   - on the host: save public key in putty.exe folder and name it ''public_key''.' );
    disp( '   - on the host: save private key without passphrase in putty.exe folder and name it ''private_key.ppk''.' );
    disp( '   - on the target: import public key and run ''ssh-keygen -i -f public_key >> ~/.ssh/authorized_keys''' );
    cd( rpitdir );
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
  
  [ status, out ] = system( 'ls' );
  if isempty( strfind( out, 'key' ) ) || isempty( strfind( out, 'key.pub' ) );
    [ status, out ] = system( 'ssh-keygen -t rsa -N '''' -f key' );
    if ( status )
      disp( '  > Unable to generate private and public key. Check your ssh installation.' );
      cd( rpitdir );
      clear;
      return;
    end
  else
    disp( '  > ''key'' and ''key.pub'' already generated. Skipping.' );
  end

  piip = input( '  > Enter IP address of the target : ', 's' );
  disp( '  > Enter password of the pi account on your target when prompted.' );
  command = sprintf( 'LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libcrypto.so.1.0.0 scp -q key.pub install_key.sh pi@%s:.', piip );
  system( command );
  disp( '  > Enter password of the pi account on your target again please.' );
  command = sprintf( 'LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libcrypto.so.1.0.0 ssh pi@%s "bash install_key.sh"', piip );
  system( command );
  
  % Checking passwordless connection
  
  disp( '  > Checking passwordless ssh connection [CRTL-C if hanging]...' );
  command = sprintf( 'LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libcrypto.so.1.0.0 ssh  -i key pi@%s "pwd"', piip );
  [ status, out ] = system( command );
  if strfind( out, '/home/pi' );
    disp( '  > Passwordless connection to the target established.' );
  else
    disp( '  > Unable to connect to the target. Troubleshout the cmd.' );
    disp( ['  > Command: ' command ] );
    disp( '  > And try setup again.' );
    cd( rpitdir );
    clear;
    return;
  end
  
  % Defining ssh commands
  
  ssh_command = 'LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libcrypto.so.1.0.0 ssh -i key';
  scp_command = 'LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libcrypto.so.1.0.0 scp -i key';
  
end % End of UNIX configuration
  
% Check if the target is a RPI

command = sprintf( '%s pi@%s sudo cat /etc/os-release', ssh_command, piip );
[ status, out ] = system( command );
if isempty( strfind( out, 'debian' ) );
  disp( '  > Distant target is not Debian-based. Aborting.' );
  cd( rpitdir );
  clear;
  return;
else
  if isempty( strfind( out, 'raspbian' ) );
    disp( '  > Distant target is Debian-based but not a RPI.' );
    disp( '  > Checking if distant target is ARM or x86.' );
    command = sprintf( '%s pi@%s sudo uname -m', ssh_command, piip );
    [ status, out ] = system( command );
    if isempty( strfind( out, 'arm' ) )
      if isempty( strfind( out, 'x86' ) )
        disp( '  > Warning: unrecognized platform. Defaulting to ARM setup.' );
        copyfile('../res/rpi_callback_handler_arm.m','../rpit/rpi_callback_handler.m');
        if strcmp( mvernum, '2014a' ) || strcmp( mvernum, '2015a' ) || strcmp( mvernum, '2015b' ) || strcmp( mvernum, '2016a' ) || strcmp( mvernum, '2016b' )
          copyfile('../res/ert_rpi_2014a_arm.tmf','../rpit/ert_rpi.tmf');
        else
          copyfile('../res/ert_rpi_2010b_arm.tmf','../rpit/ert_rpi.tmf');
        end
      else
        disp( '  > Updating the TMF for x86 gcc optimizations.' );
        copyfile('../res/rpi_callback_handler_x86.m','../rpit/rpi_callback_handler.m');
        if strcmp( mvernum, '2014a' ) || strcmp( mvernum, '2015a' ) || strcmp( mvernum, '2015b' ) || strcmp( mvernum, '2016a' ) || strcmp( mvernum, '2016b' )
          copyfile('../res/ert_rpi_2014a_x86.tmf','../rpit/ert_rpi.tmf');
        else
          copyfile('../res/ert_rpi_2010b_x86.tmf','../rpit/ert_rpi.tmf');
        end
      end
    else
      disp( '  > Updating the TMF for ARM gcc optimizations.' );
      copyfile('../res/rpi_callback_handler_arm.m','../rpit/rpi_callback_handler.m');
      if strcmp( mvernum, '2014a' ) || strcmp( mvernum, '2015a' ) || strcmp( mvernum, '2015b' ) || strcmp( mvernum, '2016a' ) || strcmp( mvernum, '2016b' )
        copyfile('../res/ert_rpi_2014a_arm.tmf','../rpit/ert_rpi.tmf');
      else
        copyfile('../res/ert_rpi_2010b_arm.tmf','../rpit/ert_rpi.tmf');
      end
    end
    target_is_rpi = 0;
  else
    disp( '  > Distant target is a RPI.' );
    copyfile('../res/rpi_callback_handler_arm.m','../rpit/rpi_callback_handler.m');
    if strcmp( mvernum, '2014a' ) || strcmp( mvernum, '2015a' ) || strcmp( mvernum, '2015b' ) || strcmp( mvernum, '2016a' ) || strcmp( mvernum, '2016b' )
      copyfile('../res/ert_rpi_2014a_arm.tmf','../rpit/ert_rpi.tmf');
    else
      copyfile('../res/ert_rpi_2010b_arm.tmf','../rpit/ert_rpi.tmf');
    end
    target_is_rpi = 1;
  end
end

% Copy MATLAB files to the target

disp( '  > Copying MATLAB files (may take a while).' );
command = sprintf( '%s pi@%s "rm -r MATLAB;mkdir MATLAB;mkdir MATLAB/simulink;mkdir -p MATLAB/toolbox/coder/rtiostream/src"', ssh_command, piip );
[ status, out ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB', scp_command, [ '"' matlabroot '/extern' '"' ], piip );
[ status, out ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB', scp_command, [ '"' matlabroot '/rtw' '"' ], piip );
[ status, out ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB/simulink', scp_command, [ '"' matlabroot '/simulink/include' '"' ], piip );
[ status, out ] = system( command );
command = sprintf( '%s -r %s pi@%s:./MATLAB/simulink', scp_command, [ '"' matlabroot '/simulink/src' '"' ], piip );
[ status, out ] = system( command );
if strcmp( mvernum, '2014a' ) || strcmp( mvernum, '2015a' ) || strcmp( mvernum, '2015b' ) || strcmp( mvernum, '2016a' ) || strcmp( mvernum, '2016b' )
  command = sprintf( '%s -r %s pi@%s:./MATLAB/toolbox/coder/rtiostream/src', scp_command, [ '"' matlabroot '/toolbox/coder/rtiostream/src/utils' '"' ], piip );
  [ status, out ] = system( command );
end

% Apply 2012a char16_t bug workaround

if strcmp( mvernum, '2012a' )
  disp( '  > Apply 2012a char16_t bug workaround.' );
  command = sprintf( '%s -r ../res/tmwtypes_2012a.h pi@%s:./MATLAB/extern/include/tmwtypes.h', scp_command, piip );
  [ status, out ] = system( command );
end

% Copy S-function sources to the target

disp( '  > Uploading S-Function files.' );
command = sprintf( '%s -r ../blocks pi@%s:./MATLAB', scp_command, piip );
[ status, out ] = system( command );

% Create the target working directory

disp( '  > Creating a working directory ''RTW'' in user ''pi'' home dir.' );
command = sprintf( '%s pi@%s "rm -r RTW;mkdir RTW"', ssh_command, piip );
[ status, out ] = system( command );

% Copy Polaris rom file to the target into the RTW dir

disp( '  > Uploading Polaris rom file.' );
command = sprintf( '%s -r ../res/polaris.rom pi@%s:./RTW', scp_command, piip );
[ status, out ] = system( command );

% Install some additional packages on the target

disp( '  > Installing the ''screen'' utility on the target.' );
command = sprintf( '%s pi@%s sudo apt-get -y install screen', ssh_command, piip );
[ status, out ] = system( command );
if ( status )
  disp( '  > Installation of ''screen'' returned an error.' );
  disp( '  > Is your target connected to internet ?' );
  disp( '  > Please install it manually: sudo apt-get install screen' );
end
disp( '  > Installing the ''libusb-1.0-0-dev'' package on the target.' );
command = sprintf( '%s pi@%s sudo apt-get -y install libusb-1.0-0-dev', ssh_command, piip );
[ status, out ] = system( command );
if ( status )
  disp( '  > Installation of ''libusb-1.0-0-dev'' returned an error.' );
  disp( '  > Is your target connected to internet ?' );
  disp( '  > Please install it manually: sudo apt-get install libusb-1.0-0-dev' );
end

% Enable user access to the EV3 USB port on the target

disp( '  > Enabling user access to the EV3 USB port.' );
command = sprintf( '%s ../res/99-EV3.rules pi@%s:.', scp_command, piip );
[ status, out ] = system( command );
command = sprintf( '%s pi@%s "sudo mv ./99-EV3.rules /etc/udev/rules.d"', ssh_command, piip );
[ status, out ] = system( command );
command = sprintf( '%s pi@%s "sudo chown root.root /etc/udev/rules.d/99-EV3.rules"', ssh_command, piip );
[ status, out ] = system( command );
command = sprintf( '%s pi@%s "sudo chmod 644 /etc/udev/rules.d/99-EV3.rules"', ssh_command, piip );
[ status, out ] = system( command );
if ( status )
  disp( '  > Creation of /etc/udev/rules.d/99-EV3.rules returned an error.' );
  disp( '  > Please install this manually: ' );
  disp( command );
end

% Enable user access to the NXT USB port on the target

disp( '  > Enabling user access to the NXT USB port.' );
command = sprintf( '%s ../res/99-NXT.rules pi@%s:.', scp_command, piip );
[ status, out ] = system( command );
command = sprintf( '%s pi@%s "sudo mv ./99-NXT.rules /etc/udev/rules.d"', ssh_command, piip );
[ status, out ] = system( command );
command = sprintf( '%s pi@%s "sudo chown root.root /etc/udev/rules.d/99-NXT.rules"', ssh_command, piip );
[ status, out ] = system( command );
command = sprintf( '%s pi@%s "sudo chmod 644 /etc/udev/rules.d/99-NXT.rules"', ssh_command, piip );
[ status, out ] = system( command );
if ( status )
  disp( '  > Creation of /etc/udev/rules.d/99-NXT.rules returned an error.' );
  disp( '  > Please install this manually: ' );
  disp( command );
end

if ( target_is_rpi )

  % Setup i2c subsystem on the RPI at full speed

  disp( '  > Installing the i2c utilities on the RPI.' );
  command = sprintf( '%s pi@%s sudo apt-get install i2c-tools libi2c-dev', ssh_command, piip );
  [ status, out ] = system( command );
  if ( status )
    disp( '  > Installation of i2c utilities returned an error.' );
    disp( '  > Is your RPI connected to internet ?' );
    disp( '  > Please install it manually: sudo apt-get install i2c-tools libi2c-dev' );
  end
  disp( '  > Verifying /etc/modules.' );
  command = sprintf( '%s pi@%s "sudo cat /etc/modules"', ssh_command, piip );
  [ status, out ] = system( command );
  if strfind( out, 'i2c-dev' );
    if strfind( out, 'i2c-bcm270' );
      disp( '  > /etc/modules already up to date.' );
    else
      disp( '  > Adding i2c-bcm2708 to /etc/modules.' );
      command = sprintf( '%s pi@%s "sudo bash -c ''echo i2c-bcm2708 >> /etc/modules''"', ssh_command, piip );
      [ status, out ] = system( command );
    end
  else
     disp( '  > Adding i2c-dev to /etc/modules.' );
     command = sprintf( '%s pi@%s "sudo bash -c ''echo i2c-dev >> /etc/modules''"', ssh_command, piip );
     [ status, out ] = system( command );
  end
% Not needed anymore on recent systems
%   disp( '  > Verifying /etc/modprobe.d/raspi-blacklist.conf.' );
%   command = sprintf( '%s pi@%s "sudo cat /etc/modprobe.d/raspi-blacklist.conf"', ssh_command, piip );
%   [ status, out ] = system( command );
%   if strfind( out, '#blacklist i2c-bcm2708' );
%     disp( '  > /etc/modprobe.d/raspi-blacklist.conf already up to date.' );
%   else
%     disp( '  > Commenting out ''blacklist i2c-bcm2708'' in  /etc/modprobe.d/raspi-blacklist.conf.' );
%     command = sprintf( '%s pi@%s "sudo sed ''s/blacklist i2c-bcm2708/#blacklist i2c-bcm2708/'' < /etc/modprobe.d/raspi-blacklist.conf > ~/raspi-blacklist.conf"', ssh_command, piip );
%     [ status, out ] = system( command );
%     command = sprintf( '%s pi@%s "sudo mv ~/raspi-blacklist.conf /etc/modprobe.d/"', ssh_command, piip );
%     [ status, out ] = system( command );
%   end
  disp( '  > Configuring i2c at 400kHz.' );
  command = sprintf( '%s pi@%s "sudo bash -c ''echo options i2c_bcm2708 baudrate=400000 > /etc/modprobe.d/i2c.conf''"', ssh_command, piip );
  [ status, out ] = system( command );
  command = sprintf( '%s pi@%s "sudo rmmod i2c_bcm2708;sudo modprobe i2c_bcm2708;sudo modprobe i2c_dev"', ssh_command, piip );
  [ status, out ] = system( command );
    disp( '  > Verifying /boot/config.txt.' );
  command = sprintf( '%s pi@%s "sudo cat /boot/config.txt"', ssh_command, piip );
  [ status, out ] = system( command );
  if strfind( out, 'device_tree=' );
    disp( '  > /boot/config.txt already up to date.' );
  else
    disp( '  > Adding ''device_tree='' at the end of  /boot/config.txt.' );
    command = sprintf( '%s pi@%s "sudo bash -c ''echo device_tree= >> /boot/config.txt''"', ssh_command, piip );
    [ status, out ] = system( command );
  end
  disp( '  > Adding user pi to the i2c group.' );
  command = sprintf( '%s pi@%s "sudo usermod -a -G i2c pi"', ssh_command, piip );
  [ status, out ] = system( command );
  disp( '  > Checking cpu governor configuration.' );
  command = sprintf( '%s pi@%s "sudo cat /etc/rc.local"', ssh_command, piip );
  [ status, out ] = system( command );
  strfind( out, '# Change governor to performance' );
  if strfind( out, '# Change governor to performance' );
    disp( '  > rc.local already set cpu governor to performance mode.' );
  else
    disp( '  > Force cpu governor to performance mode in rc.local.' );
    command = sprintf( '%s pi@%s %s', ssh_command, piip, '"sudo sed -i -e ''s/^exit 0/# Change governor to performance\nfor cpucore in \/sys\/devices\/system\/cpu\/cpu?; do echo performance | sudo tee $cpucore\/cpufreq\/scaling_governor > \/dev\/null; done\n\nexit 0/g'' /etc/rc.local"' );
    [ status, out ] = system( command );
  end;
end

disp( '# NOTE: please reboot your target for changes to take effect.' );
disp( '*** Configuration of RPIt successfully completed.' );

% Restore initial path and cleanup variables

cd( rpitdir );
clear;
