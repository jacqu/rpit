function wsize = fwrite(obj, varargin)
%FWRITE Write binary data to device.
%   FWRITE(OBJ, A) writes the data A, to the device connected to serial port object, OBJ.
%
%   The serial port object must be connected to the device with the 
%   FOPEN function before any data can be written to the device 
%   otherwise an error will be returned. A connected serial port
%   object has a Status property value of open.
%
%   FWRITE(OBJ,A,'PRECISION') writes binary data translating MATLAB
%   values to the specified precision, PRECISION. The supported
%   PRECISION strings are defined below. By default the 'uchar' 
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
%       s = serial('COM1');
%       fopen(s);
%       fwrite(s, [0 5 5 0 5 5 0]);
%       fclose(s);
%
%   See also W32SERIAL/FOPEN, W32SERIAL/FCLOSE, W32SERIAL/FREAD
%

% Error checking.
if ~isa(obj, 'w32serial')
    error('First input must be a W32SERIAL object.')
end

if length(obj) > 1
    error('First input must be a 1-by-1 interface object.')
end

% Parse the input.
switch nargin
case 1
   error('The input argument A must be specified.')
case 2
   cmd = deal(varargin{1});
   cmd = cmd(:);
   precision = 'uchar';
case 3
   [cmd, precision] = deal(varargin{1:2});
   cmd = cmd(:);
otherwise
   error('Too many input arguments to W32SERIAL/FWRITE.')
end   

if ~(isa(precision, 'char') || isa(precision, 'double'))
   error('The third input argument must be a string.')
end

% Error checking.
if ~(isnumeric(cmd) || ischar(cmd))
	error('The input argument A must be numeric or a string.')
end
if ~isa(precision, 'char')
	error('The input argument PRECISION must be a string.');
end

% Convert the data to the requested precision.
switch (precision)
case {'uchar', 'char'}
    cmd = uint8(cmd);
case {'schar'}
    cmd = int8(cmd);
case {'int8'}
    cmd = int8(cmd);
case {'int16', 'short'}
    cmd = int16(cmd);
case {'int32', 'int', 'long'}
    cmd = int32(cmd);
case {'uint8'}
    cmd = uint8(cmd);
case {'uint16', 'ushort'}
    cmd = uint16(cmd);
case {'uint32', 'uint', 'ulong'}
    cmd = uint32(cmd);
case {'single', 'float32', 'float'}
    cmd = single(cmd);
case {'double' ,'float64'}
    cmd = double(cmd);
otherwise
    error('Invalid PRECISION specified.')
end
count = length(cmd);

wsize = communicate('comwrite', obj.FileID, cmd, count);
