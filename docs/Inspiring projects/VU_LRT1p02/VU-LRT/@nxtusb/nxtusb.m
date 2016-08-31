function obj = nxtusb(varargin)
%NXTUSB Constructor
%   OBJ = NXTUSB creates a NXTUSB object if MindStormsNXT has been connected
% via USB and the initialization of USB device hase succeeded.
%
%   See also NXTUSB/OPEN, NXTUSB/CLOSE, NXTUSB/READ, NXTUSB/WRITE, NXTUSB/DELETE

if ~ispc
  error('This command is for the PC(Windows) version only.')
end

switch nargin
    case 0
        try
            % call the private MEX function
            [nxterr id name] = mexusb('usbinit');
        catch
            rethrow(lasterror)
        end
    otherwise
        error('Too many input arguments specified.')
end

% update the properties
obj.CurrentState =nxterr;
obj.FileID = id;
obj.Name = name;
obj = class(obj,'nxtusb');
