#include "communicate.h"

/* 
 * communicate.c
 *
 * This MEX-file is an interface to the WIN32 SERIAL APIs.
 * There are three modes of operation for this MEX-file; 'open',
 * 'comopen' and 'comclose'.  The syntax for calling this MEX-file is as follows:
 *
 * ID = COMMUNICATE('comopen',S) initializes the interface to the COM port
 *		and returns ID, a unique integer ID corresponding to the open com port
 *		configurated by S. The MATLAB M-file W32SERIAL/FOPEN should be used to call this routine.
 *
 *  COMMUNICATE('comclose',ID) finishes writing the COM port represented by ID.
 *  This will close the stream and file handles. This routine should be called by
 *  the MATLAB M-file W32SERIAL/FCLOSE.
 *
 *	commutil.c contains all the routines called by AVI.  
 */

static int mexAtExitIsSet = 0;

void mexFunction(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[])
{


    char *mode;
    void (*opMode)(int nlhs,
                   mxArray *plhs[],
                   int nrhs,
                   const mxArray *prhs[]);

    if( (nrhs < 2) && (nrhs > 8) )
        mexErrMsgTxt("Invalid number of input arguments.");

    if(!mexAtExitIsSet)
        {
            InitNode();
            mexAtExit(exitMex);
            mexAtExitIsSet = 1;
        }

    if ( mxIsChar(prhs[0]) ) 
        mode = mxArrayToString(prhs[0]);
    else
        mexErrMsgTxt("First input to COMMUNICATE must be a string.");

    if (mode == NULL)
        mexErrMsgTxt("Memory allocation failed.");
	
    if(!strcmp(mode,"comopen"))
    	opMode = comopen;
	else if(!strcmp(mode,"comclose"))
    	opMode = comclose;
    else if(!strcmp(mode,"comwrite"))
    	opMode = comwrite;
    else if(!strcmp(mode,"comread"))
    	opMode = comread;
    else if(!strcmp(mode,"comcloseall"))
    	opMode = comcloseall;
	else
		mexErrMsgTxt("Unrecognized mode for COMMUNICATE.");

	mxFree(mode);
		
	(*opMode)(nlhs,
                 plhs,
                 nrhs,
                 prhs);

}

/*
 * COMOPEN opens the COM port and returns a unique file ID in plhs
*/

void comopen(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[])
{
    mxArray *m;
	DCB dcb;
    COMMTIMEOUTS CommTimeouts;
	HANDLE hComm;
    char *portname, *fname;
    const char *prefix = "\\\\.\\";
	int id;
	double databits, stopbits, baudrate;
	/* Validate inputs */
	if (nrhs != 2)
		mexErrMsgTxt("Invalid number of inputs to COMOPEN.");
    m = mxGetField(prhs[1], 0, "PortName");
	if ( !mxIsChar(m) )
		mexErrMsgTxt("The first input to COMOPEN must be the com port name.");
	portname = mxArrayToString(m);
	if( portname == NULL )
		mexErrMsgTxt("Memory allocation failed.");
    fname = (char *)mxMalloc(strlen(prefix) + strlen(portname) + 1);
    strcpy(fname, prefix);
    strcat(fname, portname);
    //Create Connection
	hComm = CreateFile((LPCTSTR)fname,
						GENERIC_READ | GENERIC_WRITE,
						0,
						0,
						OPEN_EXISTING,
						FILE_ATTRIBUTE_NORMAL,
						NULL
						);
	if(hComm == INVALID_HANDLE_VALUE)
	{
		LPVOID lpMessageBuffer;
		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER |
			FORMAT_MESSAGE_FROM_SYSTEM,
			NULL,
			GetLastError(),
			MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US), 
			(LPTSTR) &lpMessageBuffer,
			0,
			NULL );
        mxFree(fname);
        mxFree(portname);
        mexErrMsgIdAndTxt("Communicate:COMopenFail", TEXT((LPCTSTR)lpMessageBuffer));
		LocalFree( lpMessageBuffer );
	}
    
    
	GetCommState(hComm, &dcb);
    // Set BaudRate
    // Disable the flow controls
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    m = mxGetField(prhs[1], 0, "BaudRate");
    if (m != NULL)
    {
        baudrate = mxGetScalar(m);
        dcb.BaudRate = (DWORD)baudrate;
    }
    // Set Parity
    m = mxGetField(prhs[1], 0, "Parity");
    if (m != NULL)
    {
        char *parity = mxArrayToString(m);
        if(strcmp(parity, "EVENPARITY") == 0)
            dcb.Parity = EVENPARITY;
        else if(strcmp(parity, "MARKPARITY") == 0)
            dcb.Parity = MARKPARITY;
        else if(strcmp(parity, "NOPARITY") == 0){
            dcb.Parity = NOPARITY;}
        else if(strcmp(parity, "ODDPARITY") == 0)
            dcb.Parity = ODDPARITY;
        else if(strcmp(parity, "SPACEPARITY") == 0)
            dcb.Parity = SPACEPARITY;
    }
    // Set StopBits
    m = mxGetField(prhs[1], 0, "StopBits");
    if (m != NULL)
    {
        stopbits = mxGetScalar(m);
        if(stopbits == 1)
            dcb.StopBits = ONESTOPBIT;
        else if(stopbits == 1.5)
            dcb.StopBits = ONE5STOPBITS;
        else if(stopbits == 2)
            dcb.StopBits = TWOSTOPBITS;
    }
    // Set ByteSize
    m = mxGetField(prhs[1], 0, "DataBits");
    if (m != NULL)
    {
        databits = mxGetScalar(m);
        dcb.ByteSize = (BYTE)databits;
    }
	if(!SetCommState(hComm, &dcb))
	{
        mxFree(fname);
        mxFree(portname);
		CloseHandle(hComm);
        mexErrMsgTxt("Error Setting Comm State.");
	}

    // Retrieve the timeout parameters for all read and write operations on the port. 
    GetCommTimeouts (hComm, &CommTimeouts);

    // Change the COMMTIMEOUTS structure settings.
    CommTimeouts.ReadIntervalTimeout = 65535; //MAXDWORD;  
    CommTimeouts.ReadTotalTimeoutMultiplier = 5;  
    CommTimeouts.ReadTotalTimeoutConstant = 200;    
    CommTimeouts.WriteTotalTimeoutMultiplier = 10;  
    CommTimeouts.WriteTotalTimeoutConstant = 500;    

    // Set the timeout parameters for all read and write operations on the port. 
    if (!SetCommTimeouts (hComm, &CommTimeouts))
    {
        mxFree(fname);
        mxFree(portname);
		CloseHandle(hComm);
        mexErrMsgTxt("Error Setting Comm Timeouts.");
    }
    
	/* Keep track of stream handles */
	id = addNodetoList(hComm);

	/* Return a unique number */
	plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
	*((double *) mxGetPr(plhs[0])) = id;
    mxFree(fname);
    mxFree(portname);
}


/* 
 * COMCLOSE close the stream and the file then remove this information from
 *	the list.
 */

void comclose(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[])
{
	int identifier;
	NodeType *handle;

	/*Need to find the stream handle for this particular file. */
	identifier = (int) *mxGetPr(prhs[1]);
	handle = FindNodeInList(identifier);
    if(handle == NULL)
		mexErrMsgTxt("Cannot close the port.");
    else if(handle->hComm == NULL)
		mexErrMsgTxt("Cannot close the port.");
    CloseHandle(handle->hComm);
	deleteNodeFromList(identifier);	     
}

/* 
 * COMCLOSEALL close all streams and files and clean the list.
 */

void comcloseall(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[])
{
    cleanList();	     
}


/* 
 * COMWRITE write the data to the com port
 *
 */

void comwrite(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[])
{
    //mxClassID e;
    DWORD nobtw, size;
    void *buf;
    int cnt;
    int identifier;
    NodeType *handle;
    //double dsize;
	if (nrhs != 4)
		mexErrMsgTxt("Invalid number of inputs to COMWRITE.");
    cnt = (int) *mxGetPr(prhs[3]);
    //e = mxGetClassID(prhs[2]);
    switch(mxGetClassID(prhs[2])) {
        case mxDOUBLE_CLASS:
            nobtw = cnt * sizeof(double);
            break;
        case mxSINGLE_CLASS:
            nobtw = cnt * sizeof(float);
            break;
        case mxINT8_CLASS:
            nobtw = cnt * sizeof(char);
            break;
        case mxUINT8_CLASS:
            nobtw = cnt * sizeof(unsigned char);
            break;
        case mxINT16_CLASS:
            nobtw = cnt * sizeof(short);
            break;
        case mxUINT16_CLASS:
            nobtw = cnt * sizeof(unsigned short);
            break;
        case mxINT32_CLASS:
            nobtw = cnt * sizeof(long);
            break;
        case mxUINT32_CLASS:
            nobtw = cnt* sizeof(unsigned long);
            break;
    /*  case mxCHAR_CLASS:
            break;
        case mxUNKNOWN_CLASS:
            break;
        case mxCELL_CLASS:
            break;
        case mxSTRUCT_CLASS:
            break;
        case mxLOGICAL_CLASS:
            break;
        case mxINT64_CLASS:
            break;
        case mxUINT64_CLASS:
            break; */
        default:
            mexErrMsgTxt("Invalid data type to COMWRITE");
            break;
    }
	identifier = (int) *mxGetPr(prhs[1]);
	handle = FindNodeInList(identifier);
    if(handle == NULL) {
		mexErrMsgTxt("Cannot write the port.");
    }
    else if(handle->hComm == NULL) {
		mexErrMsgTxt("Cannot write the port.");
    }
    buf = mxGetData(prhs[2]);
    WriteFile(handle->hComm, buf, nobtw, &size, NULL);
	/* Return a number of bytes written */
	plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
	*((double *) mxGetPr(plhs[0])) = (double)size;
}


/* 
 * COMREAD read the data to the com port
 *
 */

void comread(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[])
{
    DWORD nobtw,tsize, rsize;
    mwSize dims[]={0,0};
    void *buf;
    int cnt;
    int identifier;
    NodeType *handle;
    //double dsize;
    int ret;
	if (nrhs != 3)
		mexErrMsgTxt("Invalid number of inputs to COMREAD.");
	identifier = (int) *mxGetPr(prhs[1]);
    tsize = (DWORD) *mxGetPr(prhs[2]);
	handle = FindNodeInList(identifier);
    if(handle == NULL) {
		mexErrMsgTxt("Cannot read the port.");
    }
    else if(handle->hComm == NULL) {
		mexErrMsgTxt("Cannot read the port.");
    }
    buf = mxCalloc(tsize, sizeof(unsigned char));
    if(ReadFile(handle->hComm, buf, tsize, &rsize, NULL) != 0) {
        dims[0]= (int)rsize; dims[1]= 1;
        plhs[0] = mxCreateNumericArray(2,dims,mxUINT8_CLASS,mxREAL);
        for(cnt = 0; cnt < (mwSize)rsize; cnt++) {
            *((unsigned char *)mxGetPr(plhs[0]) + cnt) = *((unsigned char *)buf + cnt);
        }
        plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
        *((double *)mxGetPr(plhs[1])) = (double)rsize;
        plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL);
        *((double *)mxGetPr(plhs[2])) = (double)ret;
    }
    else {
        dims[0]= 0; dims[1]= 0;
        plhs[0] = mxCreateNumericArray(2,dims,mxUINT8_CLASS,mxREAL);
        plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
        *((double *)mxGetPr(plhs[1])) = (double)rsize;
        plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL);
        *((double *)mxGetPr(plhs[2])) = (double)ret;
    }
    mxFree(buf);
}
