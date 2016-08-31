% This script will reflash a NXT brick with John Hansen's
% Enhanced NXT firmware version 1.28 and then copy a NXTOSEK
% RPIt driver to the NXT filesystem.
%
% jacques.gangloff@unistra.fr, June 24th 2014

disp( '# This script will reflash a NXT brick with John Hansen''s' );
disp( '# Enhanced NXT firmware version 1.28 and then copy a NXTOSEK' );
disp( '# RPIt driver called "nxtusb" to the NXT filesystem.' );
disp( '# To use the NXT with RPIt, define the ID of your NXT in the Simulink' );
disp( '# block, connect the NXT with USB to the target and run the "nxtusb[ID]"' );
disp( '# program stored on the NXT filesystem where [ID] is the number you gave' );
disp( '# to the NXT brick.' );
disp( '# Note that currently, this script works only on a Windows system.' );
cont_quest = input( '  > Continue with this setup (''y'' to continue or ''n'' to abort) ? ', 's' );

if strcmp( cont_quest, 'y' )
  disp( '  > Starting NXT setup.' );
else
  if strcmp( cont_quest, 'n' )
    disp( '  > Aborting NXT setup.' );
    return;
  else
    disp( '  > Unrecognized answer. Aborting NXT setup.' );
    return;
  end
end

if ispc
  rpitdir = pwd;
  cd( [ rpitdir '/tools/NXT' ] );
  
  % Windows NXT driver installation
  
  disp( '  > Installing Lego''s Phantom driver...' );
  disp( '  > Please close all programs except Matlab of course.' );
  disp( '  > If the dirver is already installed, the installer will skip the installation.' );
  disp( '  > In this case, simply press the "Cancel" button.' );
  system( 'start /WAIT NXT_USB_Driver_120/SETUP.EXE' );
  disp( '  > Connect the NXT brick with USB. Start the NXT brick by pushing the orange button.' );
  disp( '  > Hit any key when the NXT has been detected by Windows.' );pause;
  disp( '  > Checking the NXT firmware current version...' );
  [ status, out ] = system( 'nexttool /COM=usb -versions' );
  
  % Lego firmware update
  
  if strfind( out, 'Firmware version = 1.28' );
    disp( '  > Firmware is already up to date.' );
  else
    if strfind( out, 'Firmware version =' );
      disp( '  > Firmware msut be updated. Hit any key to start the procedure.' );pause;
      disp( '  > Starting the flashing procedure. Do not power off the NXT.' );
      [ status, out ] = system( 'nexttool /COM=usb -firmware=lms_arm_nbcnxc_128.rfw' );
      [ status, out ] = system( 'nexttool /COM=usb -versions' );
      if strfind( out, 'Firmware version = 1.28' );
        disp( '  > Firmware update successfully completed.' );
      else
        disp( '  > The firmware did not complete successfully. Please restart the whole procedure.' );
        cd( rpitdir );
        return;
      end
    else
      disp( '  > The NXT does not respond. Check your connections and restart the setup.' );
      cd( rpitdir );
      return;
    end
  end
  
  % Copy of the RPIt driver
  
  while 1
    driver_id = input( '  > Please enter the ID you want to give to this NXT (1, 2, 3 or 4): ', 's' );
    if ( (driver_id == '1') || (driver_id == '2') || (driver_id == '3') || (driver_id == '4') )
      break;
    end
  end
  
  disp( '  > Downloading RPIt driver to the NXT brick.' );
  command = sprintf( 'nexttool /COM=usb -download=nxtusb%s.rxe', driver_id );
  [ status, out ] = system( command );
  
  disp( '  > Verifying the download...' );
  [ status, out ] = system( 'nexttool /COM=usb -listfiles' );
  check_file = sprintf( 'nxtusb%s.rxe', driver_id );
  if strfind( out, check_file );
    disp( [ '  > Download successfull. You can now launch the program nxtusb' driver_id ' on the NXT.' ] );
    disp( '  > After the NXTOSEK splash screen, simply press the right button ("RUN>").' );
  else
    disp( '  > Download failed. Please restart the whole procedure.' );
  end 
  cd( rpitdir );
else
  disp( '  > This script works only on Windows. Aborting.' );
end