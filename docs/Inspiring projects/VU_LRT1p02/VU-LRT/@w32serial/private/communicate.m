function varargout = communicate(varargin)
%COMMUNICATE access to COM port
%   ID = COMMUNICATE('open', S) initializes the interface to the W32SERIAL routines
%   		    and returns ID, a unique integer ID corresponding to the open com port
%               configurated by S. The MATLAB M-file W32SERIAL/FOPEN should be used to call this routine.
%
%   COMMUNICATE('write',FRAME,BITMAPINFO,FRAMENUM,FPS,QUALITY,ID, STREAMNAME) adds FRAME 
%   		    number FRAMENUM to the stream in the AVI file represented by ID.  The 
%   		    BITMAPINFO is the bitmapheader structure of the AVIFILE object.  
%   		    The FPS (frames per second) and QUALITY parameters ar required by the
%   		    AVIFILE routines. STREAMNAME is a string describing the video stream.
%                   The MATLAB M-file W32SERIAL/ADDFRAME should be used to call this routine.
%   
%   COMMUNICATE('comclose',ID) finishes writing the COM port represented by ID. This will
%   		    close the stream and file handles. This routine should be called by
%   		    the MATLAB M-file W32SERIAL/CLOSE.
%

error(sprintf('Missing MEX-file %s.',mfilename))
