function value=subsref(obj,subscript)
%SUBSREF subsref for a NXTUSB object

if length(subscript) > 1
  error('NXTUSB objects only support one level of subscripting.');
end

switch subscript.type
 case '.'
  param = subscript.subs;    
  value = get(obj, param);
 otherwise
  error('NXTUSB objects only support structure subscripting.')
end
