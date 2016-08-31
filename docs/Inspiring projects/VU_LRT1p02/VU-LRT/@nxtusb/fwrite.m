function len = fwrite(obj, varargin)
%FWRITE Write binary data to USB device.
%
%   LEN = FWRITE(OBJ, A) writes the data A to the USB device
%   indicated by OBJ. The data A will be translated to the precision 'uchar'.
%   LEN returns the number of bytes that have written.
%
%   LEN = FWRITE(OBJ,A,'PRECISION') writes the data while translating MATLAB
%   values to the specified precision, PRECISION. The supported
%   PRECISION strings are defined below. By default the 'uchar' 
%   PRECISION is used. 
%
%   The supported PRECISION strings are defined below. By default the 'uchar'
%   PRECISION is used.
%
%      MATLAB           Description
%      'uchar'          unsigned character,  8 bits.
%      'schar'          signed character,    8 bits.
%      'int8'           integer,             8 bits.
%      'int16'          integer,             16 bits.
%      'int32'          integer,             32 bits.
%      'uint8'          unsigned integer,    8 bits.
%      'uint16'         unsigned integer,    16 bits.
%      'uint32'         unsigned integer,    32 bits.
%      'single'         floating point,      32 bits.
%      'float32'        floating point,      32 bits.
%      'double'         floating point,      64 bits.
%      'float64'        floating point,      64 bits.
%      'char'           character,           8 bits (signed or unsigned).
%      'short'          integer,             16 bits.
%      'int'            integer,             32 bits.
%      'long'           integer,             32 or 64 bits.
%      'ushort'         unsigned integer,    16 bits.
%      'uint'           unsigned integer,    32 bits.
%      'ulong'          unsigned integer,    32 bits or 64 bits.
%      'float'          floating point,      32 bits.
%
%   Example:
%       ret = nxtusb;
%       open(ret);
%       len = write(ret, [1.23, 4.56], 'float', 2);
%       close(ret);
%       delete(ret);
%
%   See also NXTUSB/OPEN, NXTUSB/CLOSE, NXTUSB/READ, NXTUSB/DELETE
%

% Error checking.
if ~isa(obj, 'nxtusb')
    error('First input must be a NXTUSB object.')
end

if length(obj) > 1
    error('First input must be a 1-by-1 interface object.')
end

if isempty(inputname(1))
    % do not allow the syntax like close(nxtusb)
    error('No "NXTUSB" object found in this work space')
end

% do not work if the device is not opened
%% switch obj.CurrentState
%%    case {5, 10, 11, 12, 13}
%%    otherwise
%%        error('USB device is not opened')
%%end

% Parse the input.
switch nargin
case 1
   error('The input argument must be specified.')
case 2
   data = deal(varargin{1});
   data= data(:);
   precision = 'uchar';
case 3
   [data, precision] = deal(varargin{1:2});
   data= data(:);
otherwise
   error('Too many input arguments to NXTUSB/WRITE.')
end   

% Error checking.
if ~(isnumeric(data) || ischar(data))
	error('The input argument BUF must be numeric or a string.')
end

% Convert the data to the requested precision.
switch (precision)
case {'uchar', 'char'}
    data = uint8(data);
case {'schar'}
    data = int8(data);
case {'int8'}
    data = int8(data);
case {'int16', 'short'}
    data = int16(data);
case {'int32', 'int', 'long'}
    data = int32(data);
case {'uint8'}
    data = uint8(data);
case {'uint16', 'ushort'}
    data = uint16(data);
case {'uint32', 'uint', 'ulong'}
    data = uint32(data);
case {'single', 'float32', 'float'}
    data = single(data);
case {'double' ,'float64'}
    data = double(data);
otherwise
    error('Invalid PRECISION specified.')
end
count = length(data);

try
    len = mexusb('usbwrite', obj.FileID, data, count);
catch ME
    rethrow(ME);
end

% obj.CurrentState = nxterr;
% assignin('caller', inputname(1), obj);
