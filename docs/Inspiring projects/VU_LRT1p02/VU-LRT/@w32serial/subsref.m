function value=subsref(obj,subscript)
%SUBSREF subsref for a W32SERIAL object

if length(subscript) > 1
  error('W32SERIAL objects only support one level of subscripting.');
end

switch subscript.type
 case '.'
  param = subscript.subs;    
  value = get(obj, param);
 otherwise
  error('W32SERIAL objects only support structure subscripting.')
end
