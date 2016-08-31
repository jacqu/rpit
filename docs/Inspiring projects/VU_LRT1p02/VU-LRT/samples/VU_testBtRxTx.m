function VU_testBtRxTx(port)

% VU_testBtRxTx demonstrates BT communication with the VU_testBtTxRx.mdl Simulink model
%    The model transmits frequency and volume values to the PC
%    which displays them on the screen, and transmits them back to the NXT
%    The model then uses these values to drive the NXT's loudspeaker.
%
%    To run the demo:-
%    a) first pair the NXT with the PC at the operating system level.  This
%       is a one-off operation.
%    b) determine the COM Port associated with the paired NXT device
%    c) Start+Run the model on the brick BEFORE calling this function.
%    d) call this function with the appropriate com port: eg. VU_testBtRxTx('COM5')
%    e) the function will terminate when the user ends the program on the NXT
%
% Ver 1.0 9/20/2011 JCPJ

h = openNXT(port);                %establish BT connectivity with the target.
if h.FileID==-1, return; end;     %quit if unsuccessful

%Define the packet:...            % uint16(BTheader)= 2Bytes \
packetLength = 7;                 % + uint32(freq)= 4Bytes    | => 7 bytes
bytesRead = packetLength;         % + uint8(vol)= 1Byte      /

while bytesRead==packetLength,    %while 7-byte packages are being received...
	[packet,bytesRead] = fread(h, packetLength, 'uint8');    %read the data
    header = typecast(packet(1:2),'uint16');                 %unpack the BT header (=len of packet)
    freq = typecast(packet(3:6),'uint32');                   %unpack the freq value
    vol = typecast(packet(7),'uint8');                       %unpack the vol value
    clc, header, packet', freq, vol                          %display the results
    fwrite(h, packet, 'uint8');                              %echo the data back to the NXT   
end; %while

disp('NXT is not transmitting expected packets:'); 
disp(['Received packet length=[' num2str(bytesRead) '].  Exiting...']);
closeNXT(h);



