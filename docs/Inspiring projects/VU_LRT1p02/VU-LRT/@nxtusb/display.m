function display(obj)
%DISPLAY Displays the NXTUSB object

disp(' ')
disp(obj)
disp('NXTUSB information')
disp(' ')

switch obj.CurrentState
    case 0
        c.CurrentState = 'NXT_NOT_FOUND';
    case 1
        c.CurrentState = 'NXT_OPEN_FAILED';
    case 2
        c.CurrentState = 'NXT_GET_NAME_FAILED';
    case 3
        c.CurrentState = 'NXT_CONNECT';
    case 4
        c.CurrentState = 'NXT_CONNECT_FAILED';
    case 5
        c.CurrentState = 'NXT_CLOSE';
    otherwise
        c.CurrentState = 'UNKNOWN';
end
c.FileID = obj.FileID;
c.Name = obj.Name;

disp(c)
