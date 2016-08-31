function [result,status] = nexttool(actionStr,optionsStr)
% Calls John Hansen's NeXTTool program to communicate with LEGO NXT brick 
%
% Syntax
%   result = nexttool(actionStr,optionsStr)
%   [result,status] = nexttool(actionStr,optionsStr) returns status=-1 if
%   the brick does not respond to an initial poll of the brick name
%   
% Options:
%    /COM=port: specify port name (COMn, usb, resource string, or alias)
%    /BT[=name]: use bluetooth (selects the first brick found or the named brick)
%    /HEX: use hexidecimal for numeric output
%    /Duration=<n>: specify the tone duration for the playtone action
%    /Inbox=<n>: use inbox number n when sending or reading a message
%    /Loop: loop when playing sound files
%    /Relative: reset output position relative
%    /Empty: empty mailbox when reading
%    /Bin[=filename]: dump data output as binary to a file (nxt.bin)
% Actions:
%    -init : initialize nxt.dat file
%    -listbricks : list resource names of all found NXT bricks
%    -clear : erase all items on the brick
%    -battery : return the battery level
%    -input=<N> : read input N (0-3)
%    -output=<N> : read the status of output N (0-2)
%    -mute : stop playing sounds
%    -playtone=<frequency> : play a tone for the specified duration
%    -run=<filename> : run the specified program
%    -runningprogram : return the name of the currently running program
%    -stop : stop the currently running program
%    -playfile=<filename> : play the specified sound file
%    -firmware=<filename> : download firmware
%    -download=<filename> : download the specified file to the NXT
%    -upload[=<pattern>] : upload the specified file(s) from the NXT (or *.*)
%    -listfiles[=<pattern>] : list the files matching the pattern (or *.*)
%    -listmodules[=<pattern>] : list the modules matching the pattern (or *.*)
%    -delete=<filename> : delete the specified file from the NXT
%    -datalog | -datalog_full: upload datalog (_full == verbose)
%    -eeprom=<n> | -eeprom_full: upload eeprom block (_full == all blocks)
%    -memory=<n> | -memory_full: upload 128 bytes of memory (_full == all memory)
%    -map: upload memory map
%    -keepalive : return the current sleep time limit
%    -sleep=<timeout> : set NXT sleep timeout (in minutes)
%    -msg=<string> : send the specified message to the NXT
%    -readmsg=<box> : read the message from the specified box
%    -resetoutputposition=<port> : reset the position for the specified port
%    -resetinputsv=<port> : reset the input scaled value for the specified port
%    -setname=<new_name> : set the name of the NXT
%    -getname : return the name of the NXT
%    -versions : return the NXT firmware and protocol versions
%    -deviceinfo : return all NXT device information
%    -freemem : return the amount of free memory
%    -lsstatus=<port> : return the low speed status for the specified port
%    -boot : reset the NXT into SAMBA mode (usb only)
%    -btreset : reset the NXT bluetooth to factory settings (usb only)
%    -defrag : defragment the NXT filesystem
% General:
%    -help : display command line options
%
% Signature
%   Author: J.C. Peyton Jones
%   Date:  1 Oct 2010
%   Copyright 2010 Villanova University

% Set default arguments
if (nargin<2) || isempty(optionsStr), optionsStr= '/COM=usb'; end;
if (nargin<1) || isempty(actionStr), actionStr= '-versions -getname'; end;

% Find location of NeXTTool.exe which must be on the Matlab search path
nxtToolExe = which('NeXTTool.exe');

% Poll brick name to check for presence of brick
[status,result] = system(['"' nxtToolExe '" ' optionsStr ' -getname']);
if status==1, error(['Error executing system command: ' cmdstr]); end;
if isempty(result), 
    status=-1;
    result= 'NXT brick is not responding. Check it is connected and powered on';
    disp(result);
else, %execute specified actionStr
    [status,result] = system(['"' nxtToolExe '" ' optionsStr ' ' actionStr]);
end; %if
    


