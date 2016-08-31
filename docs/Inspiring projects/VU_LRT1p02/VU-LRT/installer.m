function installer(cmd,rootInstallDir,downloadDir)

%INSTALLER Installs the VU-LEGO Real Time toolbox and associated third party tools
%
%   INSTALLER (with no arguments) displays a menu of installation options.
%   The tools are installed underneath the default root: C:\RTtargets
%
%   INSTALLER(n) directly executes the nth option of the installation menu:
%       1. Download, Install, and compile Mex files;
%       2. Download tools (only);
%       3. Install tools (only);
%       4. Compile Mex files (only);
%       5. Check Installed tools;
%       6. Download NXT firmware to NXT brick;
%       7. Rename the NXT brick;
%       8. Exit;
%
%   INSTALLER(n,rootInstallDir,downloadDir) enables the default download
%   and installation root directories to be overridden with pathnames
%   specified by the user.  However, these pathnames must NOT CONTAIN SPACES
%
% Signature
%   Author: James Peyton Jones
%   Date: 28 Dec 2010
%   Copyright: 2010 Villanova University

% Configuration Section
if nargin<2, rootInstallDir= 'C:\RTtools'; end;     % No spaces allowed in path!!!
if nargin<3, downloadDir= fullfile(rootInstallDir,'Downloads'); end;

if strcmp(computer,'PCWIN'),
    config(1) = struct('name','Win7zip',...                                     % For unzipping Toppers_sg
                       'url','http://kent.dl.sourceforge.net/project/sevenzip/7-Zip/9.20/7z920.exe',...
                       'downloadFile',fullfile(downloadDir,'7z920.exe'),...
                       'destDir',fullfile(rootInstallDir,'Win7zip'));
elseif strcmp(computer,'PCWIN64'),
    config(1) = struct('name','Win7zip',...                                     % For unzipping Toppers_sg
                       'url','http://kent.dl.sourceforge.net/project/sevenzip/7-Zip/9.20/7z920-x64.msi',...
                       'downloadFile',fullfile(downloadDir,'7z920-x64.msi'),...
                       'destDir',fullfile(rootInstallDir,'Win7zip'));
else
    disp('This toolbox is only for Windows platforms. Exiting...');
    return;
end;

config(2) = struct('name','Cygwin',...
                   'url','http://www.cygwin.com/setup.exe',...
                   'downloadFile',fullfile(downloadDir,'Cygwin_packages','setup.exe'),... 
                   'destDir',fullfile(rootInstallDir,'Cygwin'));           % No spaces allowed in path

config(3) = struct('name','GNU ARM Compiler',...
                   'url','http://www.gnuarm.com/bu-2.16.1_gcc-4.0.2-c-c++_nl-1.14.0_gi-6.4.exe',...
                   'downloadFile',fullfile(downloadDir,'bu-2.16.1_gcc-4.0.2-c-c++_nl-1.14.0_gi-6.4.exe'),...
                   'destDir',fullfile(rootInstallDir,'GNU_ARM_Compiler'));  % No spaces allowed in path

config(4) = struct('name','NXT OSEK',...
                   'url','http://kent.dl.sourceforge.net/project/lejos-osek/nxtOSEK_beta/nxtOSEK_v213b0.zip',...
                   'downloadFile',fullfile(downloadDir,'nxtOSEK_v213b0.zip'),...
                   'destDir',rootInstallDir);

config(5) = struct('name','Toppers sg',...
                   'url','http://www.toppers.jp/download.cgi/osek_os-1.1.lzh',...
                   'downloadFile',fullfile(downloadDir,'osek_os-1.1.lzh'),...
                   'destDir',fullfile(rootInstallDir,'nxtOSEK'));

config(6) = struct('name','NXT Tools',...
                   'url','http://bricxcc.sourceforge.net/nexttool.zip',...
                   'downloadFile',fullfile(downloadDir,'nexttool.zip'),... 
                   'destDir',fullfile(rootInstallDir,'NXT_Tools'));

config(7) = struct('name','NXT Enhanced Firmware',...
                   'url','http://bricxcc.sourceforge.net/lms_arm_jch.zip',...
                   'downloadFile',fullfile(downloadDir,'lms_arm_jch.zip'),... 
                   'destDir',fullfile(rootInstallDir,'NXT_Tools'));

config(8) = struct('name','Fantom USB Driver',...
                   'url','http://cache.lego.com/upload/contentTemplating/Mindstorms2SupportFilesDownloads/otherfiles/downloadEC6CCAA9A232D445C9FFACBB917C537F.zip',...
                   'downloadFile',fullfile(downloadDir,'downloadEC6CCAA9A232D445C9FFACBB917C537F.zip'),... 
                   'destDir',fullfile(rootInstallDir,'NXT_Tools','Fantom_USB_Driver'));

cygwinBinPath= [];
               
% Menu / Action Section               
if (nargin<1), 
    disp(' ');
    disp('***********************************************');
    disp('*                                             *');
    disp('*  Villanova University Lego Real Time Target *');
    disp('*  Installer, ver 1.02                        *');
    disp('*                                             *');
    disp('***********************************************');
    cmd = getCmd(config); 
end;
while 1, %loop until terminating 'break'
    switch cmd
        case 1, downloadTools(config); installTools(config);  mexall;
        case 2, downloadTools(config);
        case 3, installTools(config);
        case 4, mexall;
        case 5, checkTools(config);
        case 6, downloadFirmware;
        case 7, renameBrick;
        case 8, ; 
        otherwise, 
            disp(['Unrecognized option:' cmd]);
    end; %switch
    if (nargin>=1) || (cmd==8),  break; 
    else cmd = getCmd(config);
    end; %if
end; %while


function userCmd=getCmd(config)
    disp(' ');
    disp('Options:')
    disp('   1. Download, Install and Compile Mex files');
    disp('   2. Download tools (only)');
    disp('   3. Install tools (only)');
    disp('   4. Compile Mex files (only)');
    disp('   5. Check Installed tools');
    disp('   6. Download NXT firmware to NXT brick');
    disp('   7. Rename NXT brick');
    disp('   8. Exit');
    disp(' ');
    userCmd= input('Enter choice: ');
    if isempty(userCmd), userCmd = 0; end;
end; %getCmd


function checkTools(config)
    disp(' ');
    disp('TOOL STATUS:');
    disp('Checking only for file locations below the Root Install Directory:' );
    disp(['...' rootInstallDir ' (as defined in installer.m)'])
    if isempty(cygwinBinPath),
        cygwinBinPath = which('make.exe');  %assumes cygwin\bin is on the Matlab path
        setenv('PATH', [fileparts(cygwinBinPath) ';' getenv('PATH')]);
    end;
    for i=1:length(config),
        disp(['  ' config(i).name]);
        if exist(config(i).downloadFile, 'file')==2, disp('  ...Downloaded [can be deleted after install]'); end;
        switch config(i).name
            case 'Win7zip'
                 cmdstr = ['"' fullfile(config(i).destDir,'7z.exe') '"' ];
            case 'Cygwin'
                 exeFile = 'make.exe';
                 cmdstr = ['"' fullfile(config(i).destDir,'bin','make.exe') '"' ' --version'];
            case 'GNU ARM Compiler'
                 exeFile = 'arm-elf-gcc.exe';
                 cmdstr = ['"' fullfile(config(i).destDir,'bin','arm-elf-gcc.exe') '"' ' -dumpversion'];
            case 'NXT Tools'
                 exeFile = 'NeXTTool.exe';
                 cmdstr = ['"' fullfile(config(i).destDir,'NeXTTool.exe') '"' ' -help'];
            case 'NXT Enhanced Firmware'
                 cmdstr = ['dir ' fullfile(config(i).destDir,'lms_arm_nbcnxc_*.rfw') ];
            case 'NXT OSEK'
                 exeFile = 'tool_gcc.mak';
                 cmdstr = ['dir ' fullfile(config(i).destDir,'nxtOSEK','ecrobot','tool_gcc.mak') ];
            case 'Toppers sg'
                 cmdstr = ['dir ' fullfile(config(i).destDir,'toppers_osek','sg','sg.exe') ];
            case 'Fantom USB Driver'
                 cmdstr = ['dir ' fullfile(getenv('SYSTEMROOT'), 'system32', 'fantom.dll') ];
        end; %switch
        [status,result] = system(cmdstr);
        if status==0, 
            disp('  ...Installed OK');
            switch config(i).name
                case {'Cygwin','GNU ARM Compiler','NXT Tools','NXT OSEK'}
                    matlabPathExeFile = which(exeFile);
                    if isempty(matlabPathExeFile), disp(['Warning: ' exeFile ' is not on the Matlab path']); end;
            end;
        else disp('  ...Does not appear to be installed');
        end;
    end; %for
    disp(['  ' 'Mex files for VU-LRT Blockset']);
    cmdstr = ['dir ' fullfile(fileparts(which('sfun_OSEK_soundvoltone.c')), '*.mex*') ];
    [status,result] = system(cmdstr);
    if status==0,
        disp('  ...Compiled OK');
    else
        disp('  ...Do not appear to have been compiled');
    end;
end %checkTools


function downloadTools(config)
    for i=1:length(config),
        disp(['Downloading ' config(i).name]);
        filename = config(i).downloadFile;
        if ~isdir(fileparts(filename)), mkdir(fileparts(filename)); end;
        if exist(filename, 'file')==2,
           disp(['...Downloaded file already exists.']);
        else % do download...
           try 
                urlwrite(config(i).url, filename);
                disp('...Done');
            catch
                disp(['Problem downloading ' config(i).name ' from ' config(i).url]);
                disp(lasterror);
            end;
            if strcmp(config(i).name,'Cygwin'),         % Special case:  We have to run cygwinSetup 
                disp('Downloading Cygwin Packages');    % ...to complete download of cygwin packages
                packageURL = 'http://www.gtlib.gatech.edu/pub/cygwin';
                cmdstr = ['"' filename '" --download --no-shortcuts --categories Base --packages make,libintl3 --quiet-mode' ...
                          ' --site ' '"' packageURL '"' ' --root ' '"' fileparts(filename) '"' ' --local-package-dir ' '"' fileparts(filename) '"'];
                [status,result] = system(cmdstr);
                if status==0, disp('...Done'),
                else disp('...Problem downloading Cygwin Packages.');
                end;
            end; %if
        end; %if
    end; %for
end %downloadTools
               

% Install function
function installTools(config)
    response = lower(deblank(input(['\nHave you read and accepted all the licenses associated' ...
                                    '\nwith each third party tool listed in the documentation? [y/n]: '],'s')));
    if response(1)~='y', return; end;
    for i=1:length(config),
        disp(['Installing ' config(i).name]);
        setupFile= config(i).downloadFile;
        destDir= config(i).destDir;
        if ~isdir(fileparts(destDir)), mkdir(fileparts(destDir)); end;
        switch config(i).name
            case 'Win7zip'
                if strcmp(computer,'PCWIN'),
                    cmdstr = ['"' setupFile '" /S /D=' destDir];
                elseif strcmp(computer,'PCWIN64'),
                    cmdstr = ['msiexec /i "' setupFile '" /qn /norestart INSTALLDIR="' destDir '"' ];
                    %/q INSTALLDIR="C:\Program Files\7-Zip"
                end;
            case 'Cygwin'
                cmdstr = ['"' setupFile '" --local-install --no-shortcuts --categories Base --packages make,libintl3 --quiet-mode' ...
                           ' --root ' '"' config(i).destDir '"' ' --local-package-dir ' '"' fileparts(setupFile) '"'];
                if ~isempty(strfind(destDir,' ')),
                    disp('Cygwin install path cannot contain spaces');
                    cmdstr = '';
                end;
            case 'GNU ARM Compiler'
                componentStr = 'l\o,l\n,l\i,l\t,l\t\o,l\t\i'; %little endian components without floating point support
                cmdstr =  ['"' setupFile '" /silent /norestart /lang=default /dir="' destDir '"' ...
                           ' /noicons /components="' componentStr '" /tasks="!cygwindlls" '];
                if ~isempty(strfind(destDir,' ')),
                    disp('GNU ARM Compiler install path cannot contain spaces');
                    cmdstr = '';
                end;
            case {'NXT OSEK','Toppers sg','NXT Tools','NXT Enhanced Firmware'}
                 win7zipExe = fullfile(config(1).destDir,'7z.exe');
                 cmdstr = ['"' win7zipExe '"' ' x ' '"' setupFile '"' ' -o' '"' destDir '"' ' -r -y'];
            case 'Fantom USB Driver'
                 unzip(setupFile,destDir);
                 setupExe = fullfile(destDir,'setup.exe');
                 cmdstr = ['"' setupExe '"' ' /q /AcceptLicenses yes /r:n'];
        end; %switch
        [status,result] = system(cmdstr);
        if status==0, disp('...Done'),
        else disp('...Did not appear to install correctly.');
        end;
    end; %for
    
    disp('Setting Matlab search path');
    addpath(pwd);               % Path to VU-LRT
    addpath(config(6).destDir); % Path to NeXTool.exe
    addpath(fullfile(config(2).destDir,'bin')); % Path to make
    addpath(fullfile(config(3).destDir,'bin')); % Path to gcc
    addpath(fullfile(config(4).destDir,'nxtOSEK','ecrobot')); % Path to tool_gcc.mak
    status = savepath;
    if status==0,
        disp('...Done');
    else,
        disp('Could not save the Matlab search path for future sessions');
        disp('This may require administrator privileges.  On Windows 7 platforms');
        disp('it may also be necessary to be running Matlab in Administrator mode.');
        disp('See http://www.mathworks.com/support/solutions/en/data/1-9574H9/index.html?solution=1-9574H9');
    end; %if
end %installTools


function mexall()
    disp('Compiling Mex Files');
    mexCompiler = mex.getCompilerConfigurations('C','Installed');
    if isempty(mexCompiler),
        disp('...WARNING:  A C compiler is not installed => Mex files cannot be compiled');
        disp('   Please install the C compiler, and then re-run installer(4), or select option 4 of the installer menu, to compile the mex files');
        disp('   See http://www.mathworks.com/support/compilers/R2010b/win64.html');
        disp('   and http://www.mathworks.com/help/techdoc/matlab_external/f23674.html');
    else,
        try 
            mexCompiler = mex.getCompilerConfigurations('C','Selected');
            vulrtDir= fullfile(fileparts(which('installer.m')));
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_adc.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_btinterface_r.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_btinterface_t.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_enterbutton.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_GAI.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_GAI_vernier.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_getbattvoltage.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_getms.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_HT_accel.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_HT_compass.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_HT_gyroscope.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_i2c_recieve.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_lightsensor.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_noisesensor.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_PWM.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_runbutton.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_servomotor.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_servorev.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_servosimple.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_sonarsensor.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_soundvoltone.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_touchsensor.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_usbinterface_r.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_usbinterface_t.c')]);
            eval(['mex -outdir ' vulrtDir ' ' fullfile(vulrtDir,'sfun_OSEK_GAI_vernier.c')]);
            w32serialDir= fullfile(vulrtDir,'@w32serial','private');
            eval(['mex -outdir ' w32serialDir ' ' fullfile(w32serialDir,'communicate.c')...
                                              ' ' fullfile(w32serialDir,'commutil.c')]);
            nxtusbDir= fullfile(vulrtDir,'@nxtusb','private');
            [status,result] = system(['copy /Y ' fullfile(nxtusbDir,'mexusb.c') ' ' fullfile(nxtusbDir,'mexusb.mexw32')]);
            disp('...Done');
        catch
            disp(' ');
            disp('...WARNING:  A C compiler is installed, but has not been selected => Mex files cannot be compiled');
            disp('   You may need to run mex - setup to select one of the installed C compilers');
            disp('   Then re-run installer(4), or select option 4 of the installer menu, to compile the mex files');
        end;
    end; %if
end; %mexall

    
function downloadFirmware()
    disp(' '); disp('Prepare for downloading firmware:')
    disp(' '); disp('1. Select Enhanced NXT firmware file to be flashed to the NXT');
      nextToolDir= fullfile(fileparts(which('NeXTTool.exe')));
      [filename,dirName] = uigetfile(fullfile(nextToolDir,'*.rfw'),'Select Firmware File to Download...');
      if filename==0, return; end;
    disp(' '); 
    disp('2. Power up the NXT and press the recessed reset button (below the USB port)');
    disp('   Note:  The brick should make quiet ticking noises in this mode');
    disp('3. Connect the NXT to the PC using the USB cable');
    disp(' '); 
    response = lower(deblank(input('Ready to download/flash firmware? [y/n]: ','s')));
    while response=='y',
        disp(['Flashing ' filename ' to NXT.  Please wait...']); pause(0.1);
        cmdstr= ['"' fullfile(nextToolDir,'NeXTTool.exe') '" /COM=usb -firmware=' fullfile(nextToolDir,filename)];
        [status,result] = system(cmdstr);
        if status==0, disp('...Done'),
        else disp('...Did not appear to download correctly.');
        end;
        disp(' ');
        disp('4. Remove the NXT battery or cycle the power off and on again to reboot the NXT');
        disp(' ');
        response = lower(deblank(input('Download same firmware to another brick that is ready? [y/n]: ','s')));
    end;
end %downloadFirmware

function renameBrick()
    nextToolDir= fullfile(fileparts(which('NeXTTool.exe')));
    response = 'y';
    while response=='y',
        disp(' '); 
        newName = deblank(input('Enter new brick name (<8 characters): ','s'));
        cmdstr= ['"' fullfile(nextToolDir,'NeXTTool.exe') '" /COM=usb -setname=' newName];
        [status,result] = system(cmdstr);
        cmdstr= ['"' fullfile(nextToolDir,'NeXTTool.exe') '" /COM=usb -getname'];
        [status,result] = system(cmdstr);
        if strcmp(newName,result(1:end-1)), disp('...Done'),
        else disp(['...Did not appear to rename correctly: ' result]);
        end;
        disp(' ');
        response = lower(deblank(input('Rename another NXT brick? [y/n]: ','s')));
    end; %while
end %renameBrick
    
end %installer

    
