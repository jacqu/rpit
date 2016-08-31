function obj = delete(obj)
%DELETE Delete USB device.
%   OBJ = DELETE(OBJ) deletes the USB device associated with OBJ.
%
%   See also NXTUSB/OPEN, NXTUSB/CLOSE, NXTUSB/READ, NXTUSB/WRITE

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
    error('Unable to delete USB device of "NXTUSB" object')
end

% Parse the input.
if nargin ~= 1
    error('One input argument must be specified.')
end

try
    % call the private MEX function
    nxterr = mexusb('usbdelete', obj.Id);
catch ME
    rethrow(ME);
end

% update the properties
obj.CurrentState = nxterr;
obj.Id = -1;
% transfer the updated object to the caller workspace
assignin('caller', inputname(1), obj);
