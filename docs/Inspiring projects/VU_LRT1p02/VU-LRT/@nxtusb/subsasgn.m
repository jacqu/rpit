function obj=subsasgn(obj,subscript,value)
%SUBSASGN subsasgn for a NXTUSB object

if length(subscript) > 1
    error('NXTUSB objects only support one level of subscripting.')
end

switch subscript.type
    case '.'
        param = subscript.subs;
        if strcmpi(param, 'currentstate')
            error('An object''s ''CurrentState'' property is read only.')
        elseif strcmpi(param, 'id')
            error('An object''s ''Id'' property is read only.')
        elseif strcmpi(param, 'name')
            error('An object''s ''name'' property is read only.')
        end
        obj = set(obj, param, value);
    otherwise
     error('NXTUSB objects only support structure subscripting.')
end
