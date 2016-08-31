function obj = fopen(obj)
%OPEN Open COM port.
%   OBJ = CLOSE(OBJ) begins reading/writing the COM port associated with
%   OBJ, which is a W32SERIAL object obtained from W32SERIAL.
%
%   See also W32SERIAL/W32SERIAL, W32SERIAL/FCLOSE, , W32SERIAL/FREAD, , W32SERIAL/FWRITE.

if ~isa(obj,'w32serial')
    error('First input must be an W32SERIAL object.')
end

if length(obj) > 1
    error('First input must be a 1-by-1 interface object.');
end

if isempty(inputname(1))
    % do not allow the syntax like fopen(w32serial('COM1'))
    error('No "W32SERIAL" object found in this work space')
end

if strcmp(obj.CurrentState, 'Closed')
    c.PortName = obj.PortName;
    c.BaudRate = obj.BaudRate;
    c.Parity = obj.Parity;
    c.DataBits = obj.DataBits;
    c.StopBits = obj.StopBits;
    hid = communicate('comopen', c);
    obj.FileID = hid;
    obj.CurrentState = 'Opened';
    assignin('caller', inputname(1), obj);
end
