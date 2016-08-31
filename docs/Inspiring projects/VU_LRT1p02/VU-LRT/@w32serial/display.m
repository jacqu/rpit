function display(obj)
%DISPLAY Displays the W32SERIAL object

disp(' ')
disp(obj)
disp('control setting for a serial communications device')
disp(' ')

c.PortName = obj.PortName;
c.BaudRate = obj.BaudRate;
c.Parity = obj.Parity;
c.DataBits = obj.DataBits;
c.StopBits = obj.StopBits;
c.FileID = obj.FileID;
c.InputBufferSize = obj.InputBufferSize;
c.CurrentState = obj.CurrentState;

disp(c)
