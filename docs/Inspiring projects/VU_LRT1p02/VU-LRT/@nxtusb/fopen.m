function obj = fopen(obj)
%OPEN Open USB device.
%
%   The NXTUSB object OBJ can connect to the USB device without using
%   OPEN function. OPEN function can be used after the connection is closed
%   by CLOSE function.
%
%   OBJ = OPEN(OBJ) opens the USB device associated with OBJ.
%
%   See also NXTUSB/CLOSE, NXTUSB/READ, NXTUSB/WRITE, NXTUSB/DELETE

% Error checking.
if ~isa(obj,'nxtusb')
    error('First input must be an NXTUSB object.')
end

if length(obj) > 1
    error('First input must be a 1-by-1 interface object.');
end

if isempty(inputname(1))
    % do not allow the syntax like fopen(nxtusb)
    error('No "NXTUSB" object found in this work space')
end

if obj.Id == -1
    error('This "NXTUSB" object is not available')
else
    try
        % call the private MEX function
        nxterr = mexusb('usbopen', obj.Id);
    catch
        rethrow(lasterror)
    end
end

% update the properties
obj.CurrentState = nxterr;
% transfer the updated object to the caller workspace
assignin('caller', inputname(1), obj);
