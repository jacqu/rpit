function obj = set(obj,varargin)
%SET Set properties of W32SERIAL objects.
%
% OBJ = SET(OBJ,'PropertyName',VALUE) sets the property 'PropertyName' of
% the W32SERIAL object OBJ to the value VALUE.  
%
% OBJ = SET(OBJ,'PropertyName',VALUE,'PropertyName',VALUE,..) sets multiple
% property values of the W32SERIAL object OBJ with a single statement.
%
%   Note: This function is a helper function for SUBSASGN and not intended
%   for users.  Structure notation should be used to set property values of
%   W32SERIAL objects.  For example:
%
%       obj.BaudRate = value;
%

if nargout ~=1
  error('You must use the syntax obj=set(obj,....).');
end

if rem(length(varargin),2)
  error('The property/value inputs must always occur as pairs.');
end

numPairs = length(varargin)/2;
  
paramNames =  {'baudrate', 'parity', 'databits', 'stopbits', 'inputbuffersize'};
paramSetFcns = {'setBaudRate', 'setParity', 'setDataBits', 'setStopBits', 'setInputBufferSize'};

params = varargin(1:2:end);
values = varargin(2:2:end);

for i = 1:numPairs
  match = strmatch(lower(params{i}),paramNames);
  switch length(match)
   case 1
    obj = feval(paramSetFcns{match},obj,values{i});
   case 0
    error(sprintf('Unrecognized parameter name ''%s''.', params{i}));
   otherwise % more than one match
    msg = sprintf('Ambiguous parameter name ''%s''.', params{i});
    error(msg);
  end
end
return;

% ------------------------------------------------------------------------
function obj = setBaudRate(obj, value)
% Set BaudRate property.
% Value must be one of 110/300/600/1200/2400/4800/9600/14400/
%                      19200/38400/57600/115200/128000/256000/
baudRates = {110, 300, 600, 1200, 2400, 4800, 9600, 14400, ...
    19200, 38400, 57600, 115200, 128000, 256000};
switch value
    case baudRates
        obj.BaudRate = value;
    otherwise
        s = sprintf('%d/', baudRates{:});
        error(['BaudRate must be one of ', s])
end
return;

% ------------------------------------------------------------------------
function obj = setParity(obj, value)
% Set Parity property.
% Value must be one of
% EVENPARITY/MARKPARITY/NOPARITY/ODDPARITY/SPACEPARITY/
scheme = {'EVENPARITY', 'MARKPARITY', 'NOPARITY', 'ODDPARITY', 'SPACEPARITY'};
switch value
    case scheme
        obj.Parity = value;
    otherwise
        s = sprintf('%s/', scheme{:});
        error(['Parity must be one of ', s])
end
return;

% ------------------------------------------------------------------------
function obj = setDataBits(obj, value)
% Set DataBits property.
% Value must be one of 4/5/6/7/8/
byteSize = {4, 5, 6, 7, 8};
switch value
    case byteSize
        obj.DataBits = value;
    otherwise
        s = sprintf('%d/', byteSize{:});
        error(['DataBits must be one of ', s])
end
return;

% ------------------------------------------------------------------------
function obj = setStopBits(obj, value)
% Set StopBits property.
% Value must be one of 1/1.5/2/
stopBits = {1, 1.5, 2};
switch value
    case stopBits
        obj.StopBits = value;
    otherwise
        error('StopBits must be one of 1/1.5/2/')
end
return;

% ------------------------------------------------------------------------
function obj = setInputBufferSize(obj, value)
% Set InputBufferSize property.
%
if isa(value, 'double') & isscalar(value)
    obj.InputBufferSize = value;
else
    error('InputBufferSize must be scalar double')
end
return;
