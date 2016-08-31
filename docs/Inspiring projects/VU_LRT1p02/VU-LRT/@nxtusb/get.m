function value = get(obj,param)
%GET Query W32SERIAL properties
%
%   VALUE = GET(OBJ,'PropertyName') returns the value of the specified
%   property of the W32SERIAL object OBJ.
%
%   Note: This function is a helper for SUBSREF and is not intended for
%   users.  To retrieve property values of the W32SERIAL object, use
%   structure notation.
%
%   Example:
%       value = obj.BaudRate;
%

% Error checking.
switch lower(param)
    case 'currentstate'
        value = obj.CurrentState;
    case 'fileid'
        value = obj.FileID;
    case 'name'
        value = obj.Name;
    otherwise
        error('Unrecognized parameter name ''%s''.',param)
end
