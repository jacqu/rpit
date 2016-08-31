function obj=subsasgn(obj,subscript,value)
%SUBSASGN subsasgn for a W32SERIAL object

if length(subscript) > 1
    error('W32SERIAL objects only support one level of subscripting.')
end

switch subscript.type
    case '.'
        param = subscript.subs;
        if strcmp(lower(param), 'portname')
            error('An object''s ''PortName'' property is read only.')
        elseif strcmp(lower(param), 'currentstate')
            error('An object''s ''CurrentState'' property is read only.')
        elseif strcmp(lower(param), 'fileid')
            error('An object''s ''FileID'' property is read only.')
        end
        obj = set(obj, param, value);
    otherwise
     error('W32SERIAL objects only support structure subscripting.')
end
