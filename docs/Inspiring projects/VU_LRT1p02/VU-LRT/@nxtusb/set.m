function obj = set(obj,varargin)
%SET Set properties of NXTUSB objects.
%
%   OBJ = SET(OBJ, 'PropertyName', VALUE) sets the property 'PropertyName'
%   of the NXTUSB object OBJ to the value VALUE.
%
%   OBJ = SET(OBJ, 'PropertyName', VALUE, 'PropertyName', VALUE, ...)
%   sets multiple property values of the NXTUSB object OBJ with
%   a single statement.
%
%   Note: This function is a helper function for SUBSASGN and not intended
%   for users.  Structure notation should be used to set property values of
%   NXTUSB objects.
%
%   Example:
%       obj.BaudRate = value;
%

% Error checking.
if nargout ~= 1
    error('You must use the syntax obj=set(obj,....).')
end

if rem(length(varargin), 2)
    error('The property/value inputs must always occur as pairs.')
end

numPairs = length(varargin) / 2;

paramNames =  {'currentstate', 'id', 'name'};
params = varargin(1:2:end);

for i = 1 : numPairs
    match = strmatch(lower(params{i}),paramNames, 'exact');
    switch length(match)
        case 1
            error('An object''s ''%s'' property is read only.', params{i})
        case 0
            error('Unrecognized parameter name ''%s''.', params{i})
        otherwise
            % more than one match
            error('Ambiguous parameter name ''%s''.', params{i})
    end
end
