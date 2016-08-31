function [data numRead status] = readBytes(obj, n)
%READBYTES Reads a byte-stream of data from a device
%
% SYNTAX: [data numRead status] = readBytes(obj, n) 
%         Reads N bytes of data from a device OBJ into a column vector, DATA.
%         NUMREAD indicates the actual number of bytes successfully read
%         but the DATA is zero-padded if NUMREAD<N.
% 
%

% Error checking.
if nargout>3,  error('Too many output arguments.'); end  
if length(obj)>1,  error('First input must be a 1-by-1 interface object.'); end
if ~isa(obj,'w32serial'),  error('First input must be a W32SERIAL object.'); end
if ~isnumeric(n) || (length(n)~=1) || (n<=0) || isnan(n),
    error('Number of bytes to read, n, must be a positive scalar integer');
end;

% Read the byte stream
[data numRead status] = communicate('comread', obj.FileID, n);

% Pad the data if too few bytes read, but leave numRead, status unchanged.
if (numRead < n), data= [data; zeros(n-numRead,1,'uint8')]; end



