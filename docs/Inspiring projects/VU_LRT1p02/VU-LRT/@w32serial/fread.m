function varargout = fread(obj, varargin)
%FREAD Read binary data from device.
% 
%   A=FREAD(OBJ) reads at most the number of values specified by
%   serial port object, OBJ's, InputBufferSize property from the
%   instrument connected to OBJ and returns to A.
%
%   A=FREAD(OBJ,SIZE) reads at most the specified number of values, 
%   SIZE, from the device connected to serial port object, OBJ, and 
%   returns to A.  
%
%   FREAD blocks until one of the following occurs:
%       1. InputBufferSize values have been received
%       2. SIZE values have been received
%       3. A timeout occurs as specified by the Timeout property
%
%   The serial port object must be connected to the device with the 
%   FOPEN function before any data can be read from the device otherwise
%   an error is returned. A connected serial port object has a Status
%   property value of open.
%
%   Available options for SIZE include:
%
%      N      read at most N values into a column vector.
%      [M,N]  read at most M * N values filling an M-by-N
%             matrix, in column order.
%
%   SIZE cannot be set to INF. If SIZE is greater than the OBJ's 
%   InputBufferSize property value an error will be returned. Note
%   that SIZE is specified in values while the InputBufferSize is
%   specified in bytes.
%
%   A=FREAD(OBJ,SIZE,'PRECISION') reads binary data with the specified 
%   precision, PRECISION. The precision argument controls the number 
%   of bits read for each value and the interpretation of those bits
%   as character, integer or floating point values. The supported
%   PRECISION strings are defined below. By default the 'uchar' 
%   PRECISION is used. By default, numeric values are returned in 
%   double precision arrays.
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
%   [A,COUNT]=FREAD(OBJ,...) returns the number of values read to 
%   COUNT.
%
%   [A,COUNT,MSG]=FREAD(OBJ,...) returns a message, MSG, if FREAD 
%   did not complete successfully. If MSG is not specified a warning  
%   is displayed to the command line. 
%
%   The byte order of the device can be specified with OBJ's ByteOrder
%   property. 
%
%   OBJ's ValuesReceived property will be updated by the number of 
%   values read from the device.
% 
%   If OBJ's RecordStatus property is configured to on with the RECORD
%   function, the data received will be recorded in the file specified
%   by OBJ's RecordName property value.
%
%   Example:
%       s = serial('COM1');
%       fopen(s);
%       fprintf(s, 'Curve?');
%       data = fread(s, 512);
%       fclose(s);
%
%   See also W32SERIAL/FOPEN, W32SERIAL/FCLOSE, W32SERIAL/FWRITE
%

% Error checking.
if nargout > 3
   error('Too many output arguments.')
end  

if ~isa(obj, 'w32serial')
    error('First input must be a W32SERIAL object.')
end

if length(obj)>1
    error('First input must be a 1-by-1 interface object.')
end

% Initialize variables.
warnMsg = 'The specified amount of data was not returned within the Timeout period.';

% Parse the input.
switch nargin
    case 1
       size = get(obj, 'InputBufferSize');   
       precision = 'uchar';
    case 2
       size = varargin{1};
       precision = 'uchar';
    case 3
       [size, precision] = deal(varargin{:});
    otherwise
       error('Too many input arguments to W32SERIAL/FREAD.')
end

% Error checking.
if ~isa(precision, 'char')
	error('PRECISION must be a string.')
elseif ~isa(size, 'double')
    error('SIZE must be a double.')
elseif (size <= 0)
    error('SIZE must be greater than 0.')
elseif (any(isinf(size)))
   error('SIZE cannot be set to INF.')
elseif (any(isnan(size)))
   error('SIZE cannot be set to NaN.')
end

% Floor the size.
% Note: The call to floor must be done after the error checking
% since floor on a string converts the string to its ascii value.
size = floor(size);  

% Determine the total number of elements to read.
switch (length(size))
case 1
    totalSize = size;
    size = [size 1];
case 2
    totalSize = size(1) * size(2);
otherwise
    error('Invalid SIZE specified. Type ''help w32serial/fread'' for more information.')
end

%Determine the total number of bytes to read:   added by JCPJ 
switch precision
case {'uint8', 'int8', 'uchar', 'schar'}
case {'uint16', 'int16', 'ushort', 'short'}
    totalSize = totalSize*2;
case {'uint', 'int', 'uint32', 'int32', 'single', 'float'}
    totalSize = totalSize*4;
case {'double', 'float64', 'uint64', 'int64'}
    totalSize = totalSize*4;
otherwise
    error('Unrecognized PRECISION')
end;

% Read the data
[data numRead status] = communicate('comread', obj.FileID, totalSize);

% If the specified number of values were not available
% - pad the data, but leave numRead unchanged:  modified by JCPJ
if (numRead < totalSize)
    data= [data; zeros(totalSize-numRead,1,'uint8')];
end

% Construct the output.
varargout = cell(1,3);
try	
    data= typecast(data,precision);  %return correct data type :added by JCPJ
    varargout{1} = reshape(data, size(1), size(2));
    varargout{2} = numRead;
    varargout{3} = status;
catch
    % An error occurred while reshaping. Return the data as an array.
	varargout{1} = data;
    varargout{2} = numRead;
    varargout{3} = status;
end


