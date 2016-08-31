MEX serial interface(w32serial)

1. Back Ground

MATLAB provides standard serial interface SERIAL based on Java,
which does not depend on the platform.

Unfortunately, this command is not stable at least using on the Windows platform.
For example, if we can hardly connect up to the ECRobot Mindstorms NXT in one shot.
After restarting the ECRobot, we can create a connetion.

To solve this cproblem, we developed an original Win32 based MEX serial
interface named W32SERIAL.

This will remote-control the ECRobot efficiently, and
will be help to work with data acquisition.

2. Over View
W32SERIAL constructs a MATLAB class.
Each of the processes for serial communication are implemented as class methods.
Actually each method calls Win32 API through MEX.

3. Command Specification

The usage of W32SERIAL is similar to that of SERIAL command.

<Properties>

PortName: COM port number.(Read Only)
BaudRate: Baud rate. One of the following can be specified.
          110/300/600/1200/2400/4800/9600/14400/19200/38400/57600/115200/128000/256000/
Parity:   Parity. One of the following can be specified.
          EVENPARITY/MARKPARITY/NOPARITY/ODDPARITY/SPACEPARITY/
DataBits: Data bit. One of the following can be specified.
          4/5/6/7/8/

StopBits: Stop bit. One of the following can be specified.
          1/1.5/2/
InputBufferSize: Input buffer size,
CurrentState: Current status(Read Only)
              Opened/Closed
FileID:   File handle. (Read Only)

<Methods>

w32serial: constructor
     properties can be specified by input arguments.
fopen: Open the port.
fclose: Close the port.
fread: Read the data.
        n = fread(obj, size, presision)
fwrite: Write the data.
        n = fwrite(obj, cmd, presision, count)
