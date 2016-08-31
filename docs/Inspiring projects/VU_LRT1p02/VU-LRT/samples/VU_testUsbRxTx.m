function VU_testUsbRxTx1()

% VU_testUSBRxTx demonstrates USB communication with the VU_testUsbTxRx.mdl Simulink model
%    The model transmits frequency and volume values to the PC
%    which displays them on the screen, and transmits them back to the NXT
%    The model then uses these values to drive the NXT's loudspeaker.
%
%    To run the demo:-
%    a) connect the usb cable
%    b) Start+Run the model on the brick BEFORE calling this function.
%    c) call this function with no arguments. (The default openNXT port is 'USB')
%    d) the function will terminate when the user ends the program on the NXT
%
% Ver 1.0 9/20/2011 JCPJ

h = openNXT('USB');               %establish USB connectivity with the target.
if h.FileID==-1, return; end;     %quit if unsuccessful

%Define the packet:...           
packetLength = 5;                 % uint32(freq)= 4Bytes    | => 7 bytes
bytesRead = packetLength;         % + uint8(vol)= 1Byte     /

while bytesRead==packetLength,    %while 5-byte packages are being received...
	[packet,bytesRead] = fread(h, packetLength, 'uint8');   %read the data
    freq = typecast(packet(1:4),'uint32');                  %unpack the freq value
    vol = typecast(packet(5),'uint8');                      %unpack the vol value
    clc, packet, freq, vol                                  %display the results
    fwrite(h, packet, 'uint8');                             %echo the data back to the NXT   
end; %while

disp('NXT is not transmitting expected packets:'); 
disp(['Received packet length=[' num2str(bytesRead) '].  Exiting...']);
closeNXT(h);

