function h = openNXT(port)

%Establishes a USB connection between the host and NXT brick
%Returns the created usb object

if nargin<1, port='USB'; end;

switch upper(port(1:3)),
    case 'USB', h=openNXTusb;
    case 'COM', h=openNXTbt(port);
    otherwise
        error(['Unrecognized port: ' port]);
end;
pause(0.2); %delay to allow NXT to complete its end
end

function h1=openNXTusb
h1 = nxtusb;
while h1.CurrentState ~= 3,
    switch h1.CurrentState
        case 0, CurrentState = 'NXT_NOT_FOUND';
        case 1, CurrentState = 'NXT_OPEN_FAILED';
        case 2, CurrentState = 'NXT_GET_NAME_FAILED';
        case 3, CurrentState = 'NXT_CONNECTED';
        case 4, CurrentState = 'NXT_CONNECT_FAILED';
        case 5, CurrentState = 'NXT_CLOSED';
        otherwise, CurrentState = 'UNKNOWN';
    end; %switch
    disp(' ');
    disp(['Could not connect to runtime model over USB;  State= ' CurrentState]);
    disp('  a) Turn on the NXT with the USB cable connected');
    disp('  b) Start/Run the target program on the NXT brick');
    response = lower(deblank(input('Try again, or exit, [y/n]: ','s')));
    if response(1)=='y',
        if h1.FileID ~= -1, delete(h1); end;
        h1 = nxtusb;
    else
        return; 
    end;
end; %while
disp(['Connected to ID#: ', num2str(h1.FileID) ', Name: ' h1.Name]);
end

function h1=openNXTbt(port)
h1 = w32serial(port, 'BaudRate', 128000);
while h1.FileID==-1,
    try
        h1= fopen(h1);
    catch err
        disp(err.message(1:end-2));
        if strncmp(err.message,'Access is denied',16),
            disp(['Close any other handles that may be accessing ' port]);
            response = lower(deblank(input('Force closure of ALL BT w32serial ports and try again, [y/n]: ','s')));
            if response(1)=='y',
                fclose(h1,'all');
            end;
        else
            disp(['Could not connect to NXT over Bluetooth;']);
            disp(['  a) Ensure port ' port ' has been paired with the NXT at the operating system level'])
            disp( '  b) Turn on the NXT and wait a couple of seconds for connectivity');
            disp( '     to be established at the operating system level');
            response = lower(deblank(input('Try again, or exit, [y/n]: ','s')));
        end;
        if response(1)~='y',
            return;
        end;
    end;
end; %while
disp(['Connected to ID#: ', num2str(h1.FileID) ', on port: ' h1.PortName ', State: ' h1.currentState]);
end