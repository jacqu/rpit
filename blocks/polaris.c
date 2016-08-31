/*
 * 	polaris.c:	API for accessing the NDI Polaris localizer.
 * 
 * 	JG, 25.01.15
 */

#include "polaris.h"

/*
 * 	SERIAL COMMUNICATION ROUTINES
 */

void Comm32Port_construct( )
{
	pCOMPort = malloc(sizeof(Comm32Port));

	pCOMPort->m_bRtsCts = true;
	pCOMPort->m_ulSerBreakDelay = 4000;  /* 4000 milliseconds wait after polaris RESET */
	pCOMPort->m_nNumBytes = 0;
	pCOMPort->m_nIndexBuffer = 0;
	pCOMPort->m_fdCommPort = -1;

	memset(pCOMPort->m_gruchInputBuffer, '\0', sizeof(pCOMPort->m_gruchInputBuffer));
}

void Comm32Port_destruct( )
{
	Comm32Port_SerialClose();

	free(pCOMPort);
}

/*****************************************************************

Routine:	SerialClose

Inputs:
	None.

Returns:
	None.

Description:
	This routine closes the COM Port handle and thus the COM Port.
	It releases the hold it has on it so other programs can use it.
	Always close the COM Port once you are finished.
*****************************************************************/
void Comm32Port_SerialClose( )
{
	if( pCOMPort->m_fdCommPort != -1 )
	{
		close( pCOMPort->m_fdCommPort );
		pCOMPort->m_fdCommPort = -1;
	} /* if */

} /* SerialClose */

/*****************************************************************

Routine:	SerialOpen

Inputs:		
	unsigned uPort - the port number to open ( 0 - 8, 0 = COM1 )
	unsigned long ulBaudRate - the baud rate for the COM Port
	unsigned uFormat - the format to open the COM port with COMM_7E1, COMM_7O1, COMM_8N1
	boolean bRtsCts - whether or not to include hardware handshaking

Returns:
	int - COMM_OK (1) if successful

Description:
	This routine opens up the selected COM port to be used for all
	calls to the hardware and responses from the hardware
*****************************************************************/
int Comm32Port_SerialOpen( 	const char* portId, BaudRate nBaudRate, unsigned uFormat,
														bool bRtsCts )
{
	struct termios dcb;

	pCOMPort->m_bRtsCts = bRtsCts;

	/* Open port */
	if ( ( pCOMPort->m_fdCommPort = open( portId,  O_RDWR | O_NOCTTY | O_NDELAY |O_NONBLOCK ) ) < 0 )	{
    if (DEB) printf( "Unable to open serial port.\n" );
		pCOMPort->m_fdCommPort = -1;
		return 0;
  }

	/* Retrieve serial port config */
	if( tcgetattr( pCOMPort->m_fdCommPort, &dcb ) < 0 )	{
		if (DEB) printf( "Unable to read serial port configuration.\n" );
		return 0;
  }

	/* Input config */
	dcb.c_iflag &=~(ICRNL| INPCK | ISTRIP | IXON | BRKINT);
	/* 	No CR to NL conversion
			No parity check
			No removal of the 8th bit
			No software flow control
			BREAK char read as other characters */

	/* Output config */
	dcb.c_oflag &= ~OPOST; 	/* No output processing */

	/* Local mode */
	dcb.c_lflag &= ~(ICANON | IEXTEN | ISIG | ECHO);
	/*  Non canonical mode
			No echo */

	/* Control mode */
	dcb.c_cflag |= (CLOCAL | CREAD);
	/*  
			Ignore control signals
			Valid reception */

	/* Control char : instant return. Number of available char is returned */
	dcb.c_cc[VMIN]  = 1;
	dcb.c_cc[VTIME] = 0;

	/* Apply configuration */
  if ( tcsetattr( pCOMPort->m_fdCommPort, TCSANOW, &dcb ) )	{
		if (DEB) printf( "Unable to configure serial port.\n" );
		return 0;
  }

	/* Discards all characters from the output or input buffer */
  tcflush(pCOMPort->m_fdCommPort, TCIFLUSH);
 	tcflush(pCOMPort->m_fdCommPort, TCOFLUSH);

  Comm32Port_SerialSetBaud( nBaudRate, uFormat, pCOMPort->m_bRtsCts );

	return 1;

} /* SerialOpenComm */

/*****************************************************************

Routine:	SerialSetBaud

Inputs:
	unsigned long ulBaudRate - the Baud Rate setting for the COM Port
	unsigned uFormat - the format to set for the COM Port ( stop bits, data bits and parity )
	boolean bRtsCts - hardware handshaking ( on or off )

Returns:
	int 1 if successful

Description:
	This routine sets the COM Port settings after the COM Port has
	been open, usually the COM Port is opened with default settings
	(ie BAUD_RATE = 9600) and after it has been open the other settings
	are set.
*****************************************************************/
int Comm32Port_SerialSetBaud( BaudRate nBaudRate, unsigned uFormat, bool bRtsCts )
{
	struct termios dcb;
	speed_t nBaudRatePosix = 0;

	if( pCOMPort->m_fdCommPort == -1 )
		return 0;

	pCOMPort->m_bRtsCts = bRtsCts;

	/* Retrieve serial port config */
	if( tcgetattr( pCOMPort->m_fdCommPort, &dcb ) < 0 )	{
		pCOMPort->m_fdCommPort = -1;
		return 0;
	}

	switch ( nBaudRate )
		{
			case  BR9600   : nBaudRatePosix = B9600;break;
			case  BR14400  : nBaudRatePosix = B9600;break; /* B14400 doesn't exist */
			case  BR19200  : nBaudRatePosix = B19200;break;
			case  BR38400  : nBaudRatePosix = B38400;break;
			case  BR57600  : nBaudRatePosix = B57600;break;
			case  BR115200 : nBaudRatePosix = B115200;break;
			default        : nBaudRatePosix = B9600;
		}

	cfsetispeed( &dcb, nBaudRatePosix );
	cfsetospeed( &dcb, nBaudRatePosix );

	if( uFormat == (unsigned)COMM_8N1 )
	{
		dcb.c_cflag &= ~PARENB;
		dcb.c_cflag &= ~CSTOPB;
		dcb.c_cflag |= CS8;
	} /* if */
	else if( uFormat == (unsigned)COMM_8N2 )
	{
		dcb.c_cflag &= ~PARENB;
		dcb.c_cflag |= CSTOPB;
		dcb.c_cflag |= CS8;
	} /* else if */
	else if( uFormat == (unsigned)COMM_8O1 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag |= PARODD;
		dcb.c_cflag &= ~CSTOPB;
		dcb.c_cflag |= CS8;
	} /* else if */
	else if( uFormat == (unsigned)COMM_8O2 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag |= PARODD;
		dcb.c_cflag |= CSTOPB;
		dcb.c_cflag |= CS8;
	} /* else if */
	else if( uFormat == (unsigned)COMM_8E1 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag &= ~PARODD;
		dcb.c_cflag &= ~CSTOPB;
		dcb.c_cflag |= CS8;
	} /* else if */
	else if( uFormat == (unsigned)COMM_8E2 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag &= ~PARODD;
		dcb.c_cflag |= CSTOPB;
		dcb.c_cflag |= CS8;
	} /* else if */
	else if( uFormat == (unsigned)COMM_7N1 )
	{
		dcb.c_cflag &= ~PARENB;
		dcb.c_cflag &= ~CSTOPB;
		dcb.c_cflag |= CS7;
	} /* else if */
	else if( uFormat == (unsigned)COMM_7N2 )
	{
		dcb.c_cflag &= ~PARENB;
		dcb.c_cflag |= CSTOPB;
		dcb.c_cflag |= CS7;
	} /* else if */
	else if( uFormat == (unsigned)COMM_7O1 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag |= PARODD;
		dcb.c_cflag &= ~CSTOPB;
		dcb.c_cflag |= CS7;
	} /* else if */
	else if( uFormat == (unsigned)COMM_7O2 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag |= PARODD;
		dcb.c_cflag |= CSTOPB;
		dcb.c_cflag |= CS7;
	} /* else if */
	else if( uFormat == (unsigned)COMM_7E1 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag &= ~PARODD;
		dcb.c_cflag &= ~CSTOPB;
		dcb.c_cflag |= CS7;
	} /* else if */
	else if( uFormat == (unsigned)COMM_7E2 )
	{
		dcb.c_cflag |= PARENB;
		dcb.c_cflag &= ~PARODD;
		dcb.c_cflag |= CSTOPB;
		dcb.c_cflag |= CS7;
	} /* else if */
	else
	{
		return 0;
	}

	/* Hardware transmission control */
	if( pCOMPort->m_bRtsCts ) dcb.c_cflag |= CRTSCTS;
	else dcb.c_cflag &= ~CRTSCTS;

	/* Applying termios config for serial port */
  if ( tcsetattr( pCOMPort->m_fdCommPort, TCSANOW, &dcb ) )	{
		return 0;
  }

	return COMM_OK;

} /* SerialSetBaudRate */

/*****************************************************************

Routine:	SerialBreak()

Inputs:
	None.

Returns:
	int COMM_OK if successful

Description:
	This routine resets the COM PORT by sending a Serial Break to
	it.
*****************************************************************/
int Comm32Port_SerialBreak( )
{
	if( pCOMPort->m_fdCommPort == -1 )
		return COMM_NOT_INITIALIZED;

  if ( tcsendbreak( pCOMPort->m_fdCommPort, 0 ) ) {
		return  COMM_ERROR_SETBREAK;
	}

  m_sleep( pCOMPort->m_ulSerBreakDelay );
	
	return COMM_OK;

} /* SerialBreak */


/*****************************************************************

Routine:	SerialPutChar

Inputs:
	unsigned char uch - the char to be placed in the COM Port
	out buffer

Returns:
	int

Description:
	This routine takes a single character and places it in the output
	buffer to be sent ot the attached hardware.
*****************************************************************/
int Comm32Port_SerialPutChar( unsigned char uch )
{
	int		nRes = 0;

	unsigned char 	szString[1];

	szString[0] = uch;

	nRes = Comm32Port_SerialPutString( szString, 1 );

	if ( nRes < 0 )
		return nRes;

	else
		return COMM_OK;

} /* SerialPutChar */

/*****************************************************************

Routine:	SerialPutString

Inputs:
	unsigned char *pszString - the string to be place in the out buffer
	of the serial port
	unsigned long ulLen - the length of the buffer to be placed in the
	out buffer of the serial port

Returns:
	int

Description:
	this routine takes the string and size to be placed in the out buffer
	of the serial port and then writes that string to the hardware.
*****************************************************************/
int Comm32Port_SerialPutString( unsigned char *pszString, unsigned long ulLen )
{

	unsigned long 	ulBytesWritten = 0;

	if( pCOMPort->m_fdCommPort == -1 )
		return COMM_NOT_INITIALIZED;

	if( pszString == NULL )
		return -1;

	if( ulLen <= 0 )
		return -1;

	ulBytesWritten = write(pCOMPort->m_fdCommPort, pszString, ulLen);

	if ( ulBytesWritten != ulLen ) {
		printf( "Fail to write Command %s on COM port \n", pszString );
		return -1 ;
  }

	return ulBytesWritten;

} /* SerialPutString */

/*****************************************************************

Routine:	SerialGetChar

Inputs:
	None.

Returns:
	int

Description:
	This routine gets a single char from the input buffer of the serial
	port.  It can only get what is there so it is a good idea to check
	if there is data in the input buffer using SerialCharsAvailable
*****************************************************************/
int Comm32Port_SerialGetChar( )
{
	int nRetChar = COMM_NO_DATA;

	if( !pCOMPort->m_nNumBytes )
	{
		pCOMPort->m_nNumBytes = Comm32Port_SerialGetString( pCOMPort->m_gruchInputBuffer, READ_BUFFER_SIZE );

		if (pCOMPort->m_nNumBytes <= -1)
			pCOMPort->m_nNumBytes = 0;

	}

	if( pCOMPort->m_nNumBytes > 0 )
	{
		nRetChar = pCOMPort->m_gruchInputBuffer[pCOMPort->m_nIndexBuffer];
		pCOMPort->m_nIndexBuffer++;
		pCOMPort->m_nNumBytes--;
	}

	return nRetChar;
} /* SerialGetChar */

/*****************************************************************

Routine:	SerialCharsAvailable

Inputs:
	None.

Returns:
	int

Description:
	This routine check to see if there are characters in the input buffer
	to be read.  It is a good idea to call this routine before actually
	reading the buffer.
*****************************************************************/
int Comm32Port_SerialCharsAvailable( )
{
	if( !pCOMPort->m_nNumBytes )
	{
		pCOMPort->m_nNumBytes = Comm32Port_SerialGetString( pCOMPort->m_gruchInputBuffer, READ_BUFFER_SIZE );

		if (pCOMPort->m_nNumBytes <= -1)
			pCOMPort->m_nNumBytes = 0;

		pCOMPort->m_nIndexBuffer = 0;
	}
	return pCOMPort->m_nNumBytes;
} /* SerialCharsAvailable */

/*****************************************************************

Routine:	SerialGetString

Inputs:
	unsigned char *pszString -  the buffer to hold the string that was read
	unsigned long ulMaxLen - the maximum length the input string can be

Returns:
	int

Description:
	This routine reads in a string value from the input buffer of the
	COM Port.  It will read in a buffer up to ulMaxLen or the number of
	bytes recieved which ever is smaller.
*****************************************************************/
int Comm32Port_SerialGetString( unsigned char *pszString, unsigned long ulMaxLen )
{

	struct timeval tv;

  fd_set rset;

	int	nRet = 0;
	int ret;

	if( pCOMPort->m_fdCommPort == -1 )
		return COMM_NOT_INITIALIZED;


 	if ( true ) /* Use select with timeout, else wait until something happens */
 	{
		tv.tv_sec = 0;
    tv.tv_usec = 1000; /* 1ms time for select to avoid CPUvore active waiting */

 		for (;;)
 		{
 			FD_ZERO( &rset );
 			FD_SET( pCOMPort->m_fdCommPort, &rset );

 			if ( (ret = select( pCOMPort->m_fdCommPort + 1, &rset, NULL, NULL, &tv ) ) > 0 )
 			{
 				if ( FD_ISSET( pCOMPort->m_fdCommPort, &rset ) )
					break;
 			}

 			if ( ret == 0 )
 			{
 				/* Timeout */
 				return  COMM_ERROR_READTIMEOUT;
 			}

 			if ( ( ret < 0 )  &&  ( errno != EINTR ) )
 				return  -1;
 		}

 	} else {
 		/* Wait forever (if necessary) */
 		while( 1 )
 		{
 			FD_ZERO( &rset );
 			FD_SET( pCOMPort->m_fdCommPort, &rset );

 			if ( (ret = select( pCOMPort->m_fdCommPort + 1, &rset, NULL, NULL, NULL ) ) > 0 )
 			{
 				if ( FD_ISSET(pCOMPort->m_fdCommPort, &rset ) )
					break;
 			}

 			if ( ( ret < 0 )  &&  ( errno != EINTR ) )
 				return  -1;

 		}
  }

  nRet = read( pCOMPort->m_fdCommPort, pszString, ulMaxLen );
  
  return nRet;

} /* SerialGetString */


void m_sleep( unsigned long ms )
{
	usleep( ms * 1000 );
}

/*
 *	CRC ROUTINES
 */

static unsigned int CrcTable[256];
static int 					bFirst=1;

/***************************************************************************
Name:	uASCIIToHex

Input Values:
	char
	szStr[]:	String of ASCII characters that make up a hex
						integer value to be converted.
						Note that the string may NOT be null-terminated.
	int
	nLen:			Length of input string.

Output Values:
	None.

Returned Value:
	unsigned int:
						Integer equivalent of input array.

Description:
	This routine translates a character ASCII array which is
	hex to its equivalent integer value.

***************************************************************************/
unsigned int uASCIIToHex( char szStr[], int nLen )
{
	char
		chTemp;
	unsigned int
		uVal;
	int
		nCnt;

	uVal = 0;
	for ( nCnt = 0; nCnt < nLen; ++nCnt )
	{
		chTemp = szStr[nCnt];
		/*
		 * Convert hex ASCII digits to values to add to total value.
		 */
		if ( (chTemp >= '0') && (chTemp <= '9') )
		{
			chTemp -= '0';
		} /* if */
		else if ( (chTemp >= 'A') && (chTemp <= 'F') )
		{
			chTemp = 0x0000000a + (chTemp - 'A');
		} /* else if */
		else if ( (chTemp >= 'a') && (chTemp <= 'f') )
		{
			chTemp = 0x0000000a + (chTemp - 'a');
		} /* else if */
		else
		{
			/*
			 * We've hit a non-hex character.
			 */
			return( 0 );
		} /* else */

		/*
		 * Shift result into position of total value.
		 */
		uVal |= (chTemp << (4 * (nLen - 1 - nCnt)));
	} /* for */

	return( uVal );
} /* uASCIIToHex */

/*****************************************************************
Routine:	bExtractValue

Inputs:
	pszVal - value to be extracted, uLen - length of value
	fDivisor - how to break up information, pfValue - data

Returns:

Description:
	This routine breaks up the transformation into
	there individual components

*****************************************************************/
bool bExtractValue( char *pszVal, unsigned uLen,
					float fDivisor, float *pfValue )
{
    unsigned
        i;
    char
        szBuff[ 10 ];

    *pfValue = BAD_FLOAT;

    /*
     * Make sure that the first character is either a + or -.
     */
    if( *pszVal != '-' && *pszVal != '+' )
		{
      return false;
		} /* if */

    /*
     * Copy the + or - character to the buffer
     */
    szBuff[0] = *pszVal;

    /*
     * Copy the rest of the value.  Make sure that the remainder of
     * the value string contains only digits 0 - 9.
     */
    for( i = 1; i < uLen; i++ )
    {
			if( pszVal[i] < '0' || pszVal[i] > '9' )
			{
				return false;
			} /* if */

      szBuff[i] = pszVal[i];
    } /* for */

    /*
     * Null terminate the string.
     */
    szBuff[i] = '\0';

    *pfValue = (float)(atof( szBuff ) / fDivisor);

    return true;
} /* bExtractValue */

/*****************************************************************
Routine:	nGetHex1

Inputs:
	char *sz - the buffer that contains the 2 hex chars
	of information to be decoded

Returns:
	int u - the decimal value of the decoding

Description:
	Converts two (2) hex characters to its decimal equivialent

*****************************************************************/
int nGetHex1( char *sz )
{
    unsigned int u;

    u=sz[0] & 0xff;

    return u;
} /* nGetHex1 */

/*****************************************************************
Routine:	nGetHex2

Inputs:
	char *sz - the buffer that contains the 2 hex chars
	of information to be decoded

Returns:
	int u - the decimal value of the decoding

Description:
	Converts two (2) hex characters to its decimal equivialent

*****************************************************************/
int nGetHex2( char *sz )
{
	unsigned int u;

  u=sz[0] & 0xff;
  u|=((sz[1] & 0xFF) << 8);

  return u;
} /* nGetHex2 */

/*****************************************************************
Routine:	nGetHex4

Inputs:
	char *sz - the buffer that contains the 4 hex chars
	of information to be decoded

Returns:
	int u - the decimal value of the decoding

Description:
	Converts two (4) hex characters to its decimal equivialent

*****************************************************************/
int nGetHex4( char *sz )
{
	unsigned int u;

	u=sz[0] & 0xff;
	u|=((sz[1] & 0xFF) << 8);
	u|=((sz[2] & 0xFF) << 16);
	u|=((sz[3] & 0xFF) << 24);

	return (int) u;
} /* nGetHex4 */

/*****************************************************************
Routine:	fGetFloat

Inputs:		
	char *sz - the buffer that contains the 2 hex chars
	of information to be decoded

Returns:
	int f - the float value of the decoding

Description:
	Converts two (2) hex characters to its float equivialent

*****************************************************************/
float fGetFloat( char *sz )
{
	float f;
	unsigned int *pu;

	pu =((unsigned int *)&f);

	(*pu)=sz[0] & 0xff;
	(*pu)|=((sz[1] & 0xFF) << 8);
	(*pu)|=((sz[2] & 0xFF) << 16);
	(*pu)|=((sz[3] & 0xFF) << 24);

	return f;
} /* fGetFloat */

/*****************************************************************
Name:	InitCrcTable

Inputs:
		None.

Return Value:
		None.

Description:   
	Sets up CRC table for use with CalcCRC16
	Sets up static global var CrcTable
*****************************************************************/
void CRC_InitCrcTable( )
{
	int i,j;
	long lCrcTable;
	
	/*
	 * Create the CRC lookup table
	 */
	for( i=0; i<256; i++ )
	{
		lCrcTable = i;
		for( j=0; j<8; j++ )
			lCrcTable = ( lCrcTable >> 1 ) ^ (( lCrcTable & 1 ) ? 0xA001L : 0 );

		CrcTable[i] = (unsigned int) lCrcTable & 0xFFFF;
	} /* for */
} /* InitCrcTable */

/*****************************************************************
Name:	CalcCRC16

Inputs:
	int
	data: Data value to add to running CRC16
	unsigned int
	*puCRC16: Ptr. to running CRC16

Return Value:
	None.

Description:
	This routine calcualted a running CRC16 using the polynomial
	X^16 + X^15 + X^2 + 1.

  NOTE:  This routine was taken from the back of the System API.
*****************************************************************/
unsigned int CRC_CalcCrc16( unsigned int crc, int data )
{
	if(bFirst) /* if this is the first time perform this */
	{
		bFirst = 0;
		CRC_InitCrcTable();
	} /* if */
	crc = CrcTable[ (crc ^ data) & 0xFF] ^ (crc >> 8);
	return (crc & 0xFFFF);
} /* CalcCRC16 */

/*****************************************************************
Name:	CalcCRCByLen

Input Values:
	char *pszString : String for which CRC should be computed.
  int  nLen       : Length of string (since string is not ASCII)

Output Values:
  None.

Returned Value:
  unsigned        : CRC for computed for input string.

Description:
  This routine calculates a running CRC16 using the polynomial
  X^16 + X^15 + X^2 + 1.

*****************************************************************/
unsigned CRC_CalcCRCByLen( char *pszString, int nLen )
{
	static unsigned char
		oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1,
											1, 0, 0, 1, 0, 1, 1, 0 };
  unsigned
		data,
		uCrc = 0;
		
  unsigned char
		*puch = (unsigned char *)pszString;
		
  int
		nCnt = 0;

  while ( nCnt < nLen )
	{
		data = (*puch ^ (uCrc & 0xff)) & 0xff;
		uCrc >>= 8;

		if ( oddparity[data & 0x0f] ^ oddparity[data >> 4] )
		{
			uCrc ^= 0xc001;
		} /* if */

		data <<= 6;
		uCrc ^= data;
		data <<= 1;
		uCrc ^= data;
		puch++;
		nCnt++;
	} /* while */

  return uCrc;

} /* CalcCRCByLen */

/*****************************************************************
Name:	SystemCheckCRC

Inputs:
	char * pointer to string to check

Return Value:
	int - 0 if CRC failure
				1 if CRC successful

Description:
	 This command will check the System reply for the correct
	 CRC value.
*****************************************************************/
int CRC_SystemCheckCRC( char *psz )
{

	unsigned int
		uCrc = 0,
		uReplyCrc = 0,
		uReplySize = 0;

	int
		m, n;
		
	/*
	 * calculate CRC
	 */
	uCrc = 0;

	/*
	 * We need to check if the reply is for BX, in binary format.
	 * The start byte shall be 0xA5C4
	 */
	if ( ((psz[0] & 0xff) == 0xc4) &&
		 ((psz[1] & 0xff) == 0xa5) )
	{
		uReplyCrc = (psz[4] & 0xff) | ((psz[5] & 0xff) << 8); /* get the header CRC */

		if (CRC_CalcCRCByLen(psz, 4) == uReplyCrc) /* Check the header CRC */
		{
			/*
			 *  Get the reply size.
			 *  = reply size at [2] and [3] + 6 header bytes + 2 CRC bytes.
			 */
			uReplySize = ((psz[2] & 0xff) | ((psz[3] & 0xff) << 8)) + 8;

			/* Get the body CRC */
			uReplyCrc = (psz[uReplySize-2] & 0xff) | ((psz[uReplySize-1] & 0xff) << 8);

			if (CRC_CalcCRCByLen(&psz[6], (uReplySize-8)) == uReplyCrc)
			{
				return 1; /* CRC check OK */
			}
			else
			{
				return 0; /* Bad CRC */
			}/* else */
		}
		else
		{
			return 0; /* Bad CRC */
		}/* else */
	}
	else
	{
		for( n = 0; (psz[n]!= CARRIAGE_RETURN) && (n< MAX_REPLY_MSG); n++)
		{
			; /* get total number of bytes n */
		}/* for */

		/*
		 * if rolled over the buffer size then something is wrong and
		 * we will say the CRC is bad
		 */
		if(n>=MAX_REPLY_MSG)
			return 0;

		/*
		 * NULL terminate the string to be tidy
		 */
		psz[n+1] = 0;

		/*
		 * determine 16 bit CRC
		 */
		for(m=0;m<(n-4);m++)
				uCrc = CRC_CalcCrc16(uCrc,psz[m]);

		/*
		 * read CRC from message
		 */
		sscanf(&(psz[n-4]),"%04x",&uReplyCrc);

		/*
		 * return the comparison of the calculated and received CRC values
		 */
		return (uCrc == uReplyCrc);

	}/* else */
} /* SystemCheckCrc */


/*****************************************************************
Name:	SystemGetCRC

Inputs:
	char * pointer to string to check

Return Value:
	int - the CRC calculated for the string

Description:
	This command will calc the crc for the inputed reply string
*****************************************************************/
unsigned int CRC_SystemGetCRC( char *psz, int nLength )
{
	unsigned int
		uCrc = 0;
	int
		m = 0;

	for(m=0; m<(nLength);m++)
	{
		uCrc = CRC_CalcCrc16(uCrc, psz[m]);
	}

	return uCrc;
}

/*
 *	COMMAND HANDLING ROUTINES
 */

CommandHandling* pCommandAurora;
 
void p_construct( bool enableLog, int timeout  )
{
	pCommandAurora = malloc(sizeof(CommandHandling));

	pCommandAurora->m_bLog = enableLog;
	pCommandAurora->m_nTimeout = timeout;

	Comm32Port_construct();


} /* CCommandHandling() */


void p_destruct( )
{
	/* clean up */
	if ( pCOMPort )
		Comm32Port_destruct( );

    free(pCommandAurora);
} /* ~CCommandHandling */

/*****************************************************************
Name:	nCloseComPorts

Inputs:
	None.

Return Value:
	int , 0 if fails and 1 is passes

Description:
	This routine closes all open COM ports.
*****************************************************************/
int p_nCloseComPorts( )
{
  Comm32Port_SerialClose( );

	return 0;
} /* nCloseComPorts */

/*****************************************************************
Name:	nOpenComPort

Inputs:
	string Port : port to be open for instance "COM1" for win32
								"/dev/ttyXX" for posix

Return Value:
	int - 1 if successful, 0 otherwise

Description:
	This routine opens the selected com port and sets its settings
	to the default communication parameters
*****************************************************************/
int p_nOpenComPort( const char* port )
{
	if ( pCOMPort != NULL )
	{
		/* Set the parameters to the defaults */
		if ( Comm32Port_SerialOpen( port, BR9600, COMM_8N1, false ) )
		{
			return 1;
		}
	}

	return 0;
} /* nOpenComPort */

/*****************************************************************
Name:	p_nHardWareReset

Inputs:
	None.

Return Value:
	int - 0 if it fails, nCheckResponse if passes

Description:
	This routine sends a serial break to the system, reseting it.
*****************************************************************/
int p_nHardWareReset( )
{
	int nResponse = 0;

	/* Check COM port */
	if( pCOMPort == NULL )
	{
		return 0;
	}/* if */

	/* send serial break */  
  Comm32Port_SerialBreak( );

	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );

	if (!p_nGetResponse( ))
	{
		return 0;
	}
	/* check for the RESET response */
	nResponse = p_nVerifyResponse( pCommandAurora->m_szLastReply, true );

	if ( !p_nCheckResponse( nResponse ) )
	{
		return 0;
	}


	if ( nResponse & REPLY_RESET )
	{
		if (!CRC_SystemCheckCRC( pCommandAurora->m_szLastReply ) )
			return REPLY_BADCRC;
		else
			return nResponse;
	}
	else
	{
		return nResponse;
	}
} /* nHardwareReset */


/*****************************************************************
Name:	nSetSystemComParms

Inputs:
	int nBaudRate - the baud rate to set
	int nDateBits - the data bit setting
	int nParity - the parity setting
	int nStopBits - the stop bit setting
	int nHardware - whether or not to use hardware handshaking

Return Value:
	int - 0 if fails, else nCheckResponse

Description:
	This routine sets the systems com port parameters, remember
	to immediatley set the computers com port settings after this
	routine is called.
*****************************************************************/
int p_nSetSystemComParms( BaudRate nBaudRate, int nDataBits, int nParity, int nStopBits, int nHardware )
{
	int nBaudRateCmd = -1;

	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );

	switch ( nBaudRate )
    {
        case  BR9600   : nBaudRateCmd = 0; break;
        case  BR14400  : nBaudRateCmd = 1; break;
        case  BR19200  : nBaudRateCmd = 2; break;
        case  BR38400  : nBaudRateCmd = 3; break;
        case  BR57600  : nBaudRateCmd = 4; break;
        case  BR115200 : nBaudRateCmd = 5; break;
    }

	assert( nBaudRateCmd != -1 );

	sprintf( pCommandAurora->m_szCommand, "COMM %d%d%d%d%d", nBaudRateCmd,
                nDataBits, nParity, nStopBits, nHardware );

	if (p_nSendMessage( pCommandAurora->m_szCommand, true ))
		if (p_nGetResponse( ))
			return p_nCheckResponse( p_nVerifyResponse(pCommandAurora->m_szLastReply, true) );

	return 0;
} /* nSetSystemComParms */

/*****************************************************************
Name:	nSetCompComParms

Inputs:
	int nBaud - the baud rate to set
	int nDateBits - the data bit setting
	int nParity - the parity setting
	int nStop - the stop bit setting
	int nHardware - whether or not to use hardware handshaking

Return Value:
	int - 0 if fails, else 1

Description:
	This routine sets the computer's com port parameters, remember
	to immediatley set the computer's com port settings after the
	system's com port parameters.
*****************************************************************/
int p_nSetCompCommParms( BaudRate nBaud, int nDataBits, int nParity, int nStop, int nHardware )
{
	unsigned uFormat;

	/* Check COM port */
	if( pCOMPort == NULL )
	{
		return 0;
	}/* if */

	uFormat = (nDataBits * 6) + (nParity * 2) + (nStop);

	if ( Comm32Port_SerialSetBaud( nBaud, uFormat, nHardware ) )
		return 1;

	return 0;
} /* nSetCompCommParms */

/*****************************************************************
Name:	nBeepSystem

Inputs:
	int nBeeps - the number of times the system should beep

Return Value:
	int - 0 if fails, else nCheckResponse

Description:
	This routine sends the beep command to the System, causing
	it to beep.
*****************************************************************/
int p_nBeepSystem( int nBeeps )
{
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "BEEP %d", nBeeps );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if ( p_nGetResponse( ) )
			return p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) );
	} /* if */

	return 0;
} /* nBeepSystem */

/*****************************************************************
Name:	nInitializeSystem

Inputs:
	None.

Return Value:
	int - 0 if fails, else nCheckResponse

Description:
	This routine initializes the System by sending the
	INIT command.  Remember to reset the appropriate
	variables.
*****************************************************************/
int p_nInitializeSystem( )
{
  int i;
	/* clear the handle information */
	for ( i = 0; i < NO_HANDLES; i++ )
	{
		memset( pCommandAurora->m_dtHandleInformation[i].szPhysicalPort, 0, 5 );
		pCommandAurora->m_dtHandleInformation[i].HandleInfo.bInitialized = false;
		pCommandAurora->m_dtHandleInformation[i].HandleInfo.bEnabled = false;
	} /* for */

	/* send the message */
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "INIT " );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if ( p_nGetResponse( ) )
			return p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) );
	} /* if */

	return 0;
} /* nInitializeSystem */


/*****************************************************************
Name:	nSetFiringRate

Inputs:
	FiringRate nFiringRate - 20Hz, 30Hz or 60Hz for Polaris

Return Value:
	int - 0 if fails, 1 if passes

Description:
	This routine gets the System information through
	the VER and SFLIST commands, remember that it is
	not the same calls for the AURORA and POLARIS the
	reply mode numbers are different.
*****************************************************************/
int p_nSetFiringRate( FiringRate nFiringRate )
{
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "IRATE %d", (int)nFiringRate );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if ( p_nGetResponse( ) )
			return p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) );
	}

	return 0;
}

/*****************************************************************
Name:	nGetSystemInfo

Inputs:
	None.

Return Value:
	int - 0 if fails, 1 if passes

Description:
	This routine gets the System information through
	the VER and SFLIST commands, remember that it is
	not the same calls for the AURORA and POLARIS the
	reply mode numbers are different.
*****************************************************************/
int p_nGetSystemInfo( )
{
	int
		nHexResponse = 0;

	/* version Information */
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "VER 4" );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if ( !p_nGetResponse( ) )
		{
			return 0;
		}/* if */

		if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
		{
			return 0;
		}/* if */

		if ( !strncmp( pCommandAurora->m_szLastReply, "POLARIS", 7 )||
			 !strncmp( pCommandAurora->m_szLastReply, "polaris", 7 )||
			 !strncmp( pCommandAurora->m_szLastReply, "Polaris", 7 ) )
		{
			if ( strstr( pCommandAurora->m_szLastReply, "ACCEDO" ) )
			{
				pCommandAurora->m_dtSystemInformation.nTypeofSystem = ACCEDO_SYSTEM;
			}
			else
			{
				pCommandAurora->m_dtSystemInformation.nTypeofSystem = POLARIS_SYSTEM;
			}/* else */
		}
		else if ( !strncmp( pCommandAurora->m_szLastReply, "AURORA", 6 )||
			 !strncmp( pCommandAurora->m_szLastReply, "aurora", 6 )||
			 !strncmp( pCommandAurora->m_szLastReply, "Aurora", 6 ) )
		{
			pCommandAurora->m_dtSystemInformation.nTypeofSystem = AURORA_SYSTEM;
		}
		else
		{
			return 0;
		}/* else */

		sprintf( pCommandAurora->m_dtSystemInformation.szVersionInfo, "%s", pCommandAurora->m_szLastReply );
		/* EC-03-0071 */
		pCommandAurora->m_dtSystemInformation.szVersionInfo[strlen(pCommandAurora->m_dtSystemInformation.szVersionInfo) - 5] = 0;
	} /* if */

	if ( (pCommandAurora->m_dtSystemInformation.nTypeofSystem == POLARIS_SYSTEM) ||
		 (pCommandAurora->m_dtSystemInformation.nTypeofSystem == ACCEDO_SYSTEM) )
	{
		memset(pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand));
		sprintf( pCommandAurora->m_szCommand, "SFLIST 00" );
		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
				return 0;
			if (!p_nCheckResponse(p_nVerifyResponse(pCommandAurora->m_szLastReply, true)))
				return 0;
			/* supported features summary list */
			nHexResponse = uASCIIToHex( pCommandAurora->m_szLastReply, 8 );
			pCommandAurora->m_dtSystemInformation.bActivePortsAvail = (0x01 & nHexResponse ? 1 : 0);
			pCommandAurora->m_dtSystemInformation.bPassivePortsAvail = (0x02 & nHexResponse ? 1 : 0);
			pCommandAurora->m_dtSystemInformation.bMultiVolumeParms = (0x04 & nHexResponse ? 1 : 0);
			pCommandAurora->m_dtSystemInformation.bTIPSensing = (0x08 & nHexResponse ? 1 : 0);
			pCommandAurora->m_dtSystemInformation.bActiveWirelessAvail = (0x10 & nHexResponse ? 1 : 0);
			pCommandAurora->m_dtSystemInformation.bMagneticPortsAvail = (0x8000 & nHexResponse ? 1 : 0);
			pCommandAurora->m_dtSystemInformation.bFieldGeneratorAvail = (0x40000 & nHexResponse ? 1 : 0);
		} /* if */

		sprintf( pCommandAurora->m_szCommand, "SFLIST 01" );
		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
				return 0;
			if (!p_nCheckResponse(p_nVerifyResponse(pCommandAurora->m_szLastReply, true )))
				return 0;
			/* number of active ports */
			pCommandAurora->m_dtSystemInformation.nNoActivePorts = uASCIIToHex( &pCommandAurora->m_szLastReply[0], 1 );
		} /* if */

		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "SFLIST 02" );
		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
				return 0;
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;
			/* number of passive ports */
			pCommandAurora->m_dtSystemInformation.nNoPassivePorts = uASCIIToHex( &pCommandAurora->m_szLastReply[0], 1 );
		} /* if */

		memset(pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand));
		sprintf( pCommandAurora->m_szCommand, "SFLIST 04" );
		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
				return 0;
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;
			/* number of active tool ports supporting TIP detection */
			pCommandAurora->m_dtSystemInformation.nNoActTIPPorts = uASCIIToHex( &pCommandAurora->m_szLastReply[0], 1 );
		} /* if */

		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "SFLIST 05" );

		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
				return 0;
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;
			/* number of active wireless ports */
			pCommandAurora->m_dtSystemInformation.nNoActWirelessPorts = uASCIIToHex( &pCommandAurora->m_szLastReply[0], 1 );
		} /* if */

	} /* if */
	else
	{
		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "SFLIST 10" );
		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
				return 0;
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;
			/* number of magnetic ports */
			pCommandAurora->m_dtSystemInformation.nNoMagneticPorts = uASCIIToHex( &pCommandAurora->m_szLastReply[0], 2 );
		} /* if */

		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "SFLIST 12" );
		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
				return 0;
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;
			/* number of FGs */
			pCommandAurora->m_dtSystemInformation.nNoFGCards = uASCIIToHex( &pCommandAurora->m_szLastReply[0], 1 );
			pCommandAurora->m_dtSystemInformation.nNoFGs = uASCIIToHex( &pCommandAurora->m_szLastReply[1], 1 );
		} /* if */

		/* EC-03-0071
		 * - Add VER 7 and VER 8 commands
		 */
		/* Field Generator Version Information */
		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "VER 7" );

		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
			{
				return 0;
			}/* if */

			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
			{
				return 0;
			}/* if */

			strcat( pCommandAurora->m_dtSystemInformation.szVersionInfo, pCommandAurora->m_szLastReply );
			pCommandAurora->m_dtSystemInformation.szVersionInfo[strlen(pCommandAurora->m_dtSystemInformation.szVersionInfo) - 5] = 0;
		} /* if */

		/* SIU Version Information */
		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "VER 8" );

		if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
		{
			if ( !p_nGetResponse( ) )
			{
				return 0;
			}/* if */

			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
			{
				return 0;
			}/* if */

			strcat( pCommandAurora->m_dtSystemInformation.szVersionInfo, pCommandAurora->m_szLastReply );
			pCommandAurora->m_dtSystemInformation.szVersionInfo[strlen(pCommandAurora->m_dtSystemInformation.szVersionInfo) - 5] = 0;
		} /* if */
	} /* else */

	return 1;
} /* nGetSystemInfo */

/*****************************************************************
Name:	nInitializeAllPorts

Inputs:
	None.

Return Value:
	int - 1 is successful, 0 otherwise

Description:
	This routine intializes all the ports using the
	PINIT call.  It also makes calls to the PVWR routine
	and TTCFG routine to virtual load tool definitions.
*****************************************************************/
int p_nInitializeAllPorts( const char* pszROMFileName[NB_ROM_FILES] )
{
	int
		i = 0,
		nNoHandles = 0,
		nHandle = 0,
		n = 0;
	char
		pszPortID[8],
		szHandleList[MAX_REPLY_MSG],
		szErrorMessage[250];

	for ( i = 0; i < pCommandAurora->m_dtSystemInformation.nNoPassivePorts; i++ )
	{
		if ( strlen(pszROMFileName[i]) )
		{
			sprintf( pszPortID, "0%c", (i + 65 ) );
			if (DEB) printf( "Loading SROM for port : %s \n", pszPortID );

			p_nLoadVirtualSROM( pszROMFileName[i], pszPortID, true );
		}
	}

	do
	{
		n = 0;
		/* get the handles that need to be initialized */
		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "PHSR 02" );

		if (!p_nSendMessage( pCommandAurora->m_szCommand, true ))
			return 0;
        /*if(DEB) printf("BN : Commande %s envoyée.\n", m_szCommand);*/

		if (!p_nGetResponse( ))
			return 0;
        /*if(DEB) printf("BN : Commande %s reçue.\n", m_szLastReply);*/

		if (!p_nCheckResponse(p_nVerifyResponse(pCommandAurora->m_szLastReply, true )))
			return 0;
        /*if(DEB) printf("BN : Commande ok : %s\n", pCommandAurora->m_szCommand);*/

		sprintf( szHandleList, "%s", pCommandAurora->m_szLastReply );
		nNoHandles = uASCIIToHex( &pCommandAurora->m_szLastReply[n], 2 );
		n+=2;
		if ( nNoHandles > 0 )
		{
			for ( i = 0; i < nNoHandles; i++ )
			{
				nHandle = uASCIIToHex( &szHandleList[n], 2 );

				if(DEB) printf( "NoHandle : %d | Port handle : %d \n", nNoHandles, nHandle );

				if ( !p_nGetPortInformation( nHandle ) )
					return 0;
				if ( !pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bInitialized )
				{
					if (!p_nInitializeHandle( nHandle ))
					{
						/* Inform user which port fails on PINIT */
						sprintf( szErrorMessage, "Port %s could not be initialized.\n"
						"Check your SROM image file settings.", pCommandAurora->m_dtHandleInformation[nHandle].szPhysicalPort );
						if(DEB) printf( "%s", szErrorMessage );
						return 0;
					}/* if */
					n+=5;
				} /* if */
			} /* for */
		} /* if */
		/* do this until there are no new handles */
	}while( nNoHandles > 0 );
	return 1;
} /* nInitializeAllPorts */

/*****************************************************************
Name:	nInitializeHandle

Inputs:
	int nHandle - the handle to be intialized

Return Value:
	int - 1 if successful, otherwise 0

Description:
	This routine initializes the specified handle using
	the PINIT command.
*****************************************************************/
int p_nInitializeHandle( int nHandle )
{
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "PINIT %02X", nHandle );
	if ( !p_nSendMessage( pCommandAurora->m_szCommand, true ))
		return 0;
	if ( !p_nGetResponse() )
		return 0;

	if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
	{
		return 0;
	} /* if */
	pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bInitialized = true;

	return 1;
} /* nInitializeHandle */

/*****************************************************************
Name:	nEnableAllPorts

Inputs:
	None.

Return Value:
	int - 1 if successful, 0 otherwise

Description:
	This routine enables all the port handles that need
	to be enabled using the PENA command.
*****************************************************************/
int p_nEnableAllPorts( )
{
	int
		nNoHandles = 0,
		nPortHandle = 0,
		n = 0,
		i = 0;
	char
		szHandleList[MAX_REPLY_MSG];

	pCommandAurora->m_nPortsEnabled = 0;
	/* get all the ports that need to be enabled */
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "PHSR 03" );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if (!p_nGetResponse( ))
			return 0;

		if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
			return 0;

		sprintf( szHandleList, "%s", pCommandAurora->m_szLastReply );
		nNoHandles = uASCIIToHex( &szHandleList[n], 2 );
		n+=2;

		for (i = 0; i < nNoHandles; i++ )
		{
			nPortHandle = uASCIIToHex( &szHandleList[n], 2 );
			memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
			sprintf( pCommandAurora->m_szCommand, "PENA %02X%c", nPortHandle, 'D' );
			n+=5;
			if (!p_nSendMessage( pCommandAurora->m_szCommand, true ))
				return 0;
			if ( !p_nGetResponse() )
				return 0;
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;
			p_nGetPortInformation( nPortHandle );
			pCommandAurora->m_nPortsEnabled++;
		} /* for */
		return 1;
	} /* if */
	return 0;
} /* nEnableAllPorts */


/*****************************************************************
Name:	nActivateAllPorts

Inputs:
	None.

Return Value:
	int - 1 if successful, 0 otherwise

Description:
	This is the routine that activates all ports using

*****************************************************************/
int p_nActivateAllPorts( )
{
	const char* RomFilesName[NB_ROM_FILES];
	int         i;

	for( i = 0; i < NB_ROM_FILES; i++ ) RomFilesName[i] = "";

	RomFilesName[0] = POLARIS_ROM_FILE;

	if (!p_nFreePortHandles( ))
		return 0;
	if (DEB) printf( "Port Handles freed\n" );

	if (!p_nInitializeAllPorts( RomFilesName ))
		return 0;
  if (DEB) printf( "SROM loaded, ports initialized\n" );

	if (!p_nEnableAllPorts( ))
		return 0;
  if (DEB) printf( "Ports enabled\n" );

	return 1;
} /* nActivateAllPorts */

/*****************************************************************
Name:	nLoadVirtualSROM

Inputs:
	char * pszFileName - the file to be loaded
	char * pszPhysicalPortID - the physical port id that is being
	loaded to.
	bool bPassive - is this a passive port or not

Return Value:
	int - 1 if successful, 0 otherwise.

Description:
	This routine virtual loads a SROM file to the specified port.
	It uses the PVWR command to do pCommandAurora->
*****************************************************************/
int p_nLoadVirtualSROM( const char * pszFileName, char * pszPhysicalPortID, bool bPassive )
{
	FILE
	  *pFileHandle = NULL;
	int
		nHandle = 0,
		nBytes = 0,
		nCnt = 0,
		i = 0,
		n = 0;
	static unsigned char
    gruchBuff[1024];
	char
		cMessage[256];


	if ( !*pszFileName )
		return 0;

	if ( bPassive )
	{
		for ( i = 0; i < NO_HANDLES; i++ )
		{
			if ( !strncmp( pCommandAurora->m_dtHandleInformation[i].szPhysicalPort, pszPhysicalPortID, 2 ) )
				return 0;
		}
		/* if passive we need a port handle */
		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "PHRQ ********01%s**", pszPhysicalPortID );
		if (!p_nSendMessage( pCommandAurora->m_szCommand, true ))
			return 0;

		if (!p_nGetResponse( ))
			return 0;
		if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
			return 0;
		nHandle = uASCIIToHex( &pCommandAurora->m_szLastReply[n], 2 );
		if ( pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bInitialized == 1 )
			return 0;
	}/* if */
	else
	{
		/* if active a handle has already been assigned */
		nHandle = p_nGetHandleForPort( pszPhysicalPortID );
		if ( nHandle == 0 || pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bInitialized == 1 )
			return 0;
	}/* else */

	if( !(pFileHandle = fopen( pszFileName, "rb" )) )
	{
		if(DEB) printf( "Impossible d'ouvrir le fichier %s \n", pszFileName );
		return 0;
	} /* if */

	if( (nBytes = fread( gruchBuff, 1, sizeof(gruchBuff), pFileHandle )) < 1 )
	{
		if(DEB) printf( "Unable to read ROM image file %s.", pszFileName );
		fclose(pFileHandle);
		return 0;
	} /* if */

	for( nCnt = 0; nCnt < nBytes; )
	{
	/*
	 * write the data to the tool description section of
	 * the virtual SROM on a per port basis
	 */
		memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
		sprintf( pCommandAurora->m_szCommand, "PVWR:%02X%04X", nHandle, nCnt );

		for( i = 0; i < 64; i++, nCnt++ )
		{
		/* (plus eleven for the PVWR:XX0000 ) */
			sprintf( pCommandAurora->m_szCommand + 11 + 2 * i, "%02X", gruchBuff[nCnt] );
		} /* for */

		if (!p_nSendMessage( pCommandAurora->m_szCommand, true ))
			break;

		if (!p_nGetResponse( ))
		{
			sprintf( cMessage, "Failed writing to virtual port %s.", pszPhysicalPortID );
			if(DEB) printf( "%s", cMessage );
			break;
		} /* if */
		if (!p_nCheckResponse(p_nVerifyResponse(pCommandAurora->m_szLastReply, true)))
		{
			break;
		} /* if */
	} /* for */

	if( !(fclose( pFileHandle ) ) )
		return 0;

	return 1;
} /* nLoadVirtualSROM */

/*****************************************************************
Name:	nGetHandleForPort

Inputs:
	char * pszPortID - the physical port to match a handle to

Return Value:
	int - 0 if fails, port handle if passes

Description:
	This routine takes a physical port location and matches a handle
	to it.  These handles have already been defined with a PHSR
*****************************************************************/
int p_nGetHandleForPort( char * pszPortID )
{
	int
		nPortHandle = 0,
		nNoHandles = 0,
		n = 0,
		i = 0;
	char
		szHandleList[MAX_REPLY_MSG];

	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "PHSR 00" );

	if ( p_nSendMessage( pCommandAurora->m_szCommand, true ) )
	{
		if ( p_nGetResponse() )
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;
	} /* if */

	strcat( szHandleList, pCommandAurora->m_szLastReply );
	nNoHandles = uASCIIToHex( &szHandleList[n], 2 );
	n+=2;

	for (i = 0; i < nNoHandles; i++ )
	{
		/* for all the handles, get their physical location with PHINF */
		nPortHandle = uASCIIToHex( &szHandleList[n], 2 );
		n+=5;
		p_nGetPortInformation( nPortHandle );
		/* if the physical location match pszPortID, return the handle */
		if ( !strncmp( pCommandAurora->m_dtHandleInformation[nPortHandle].szPhysicalPort, pszPortID, 2 ) )
			return nPortHandle;
	} /* for */

	return 0;
} /* nGetHandleForPort */

/*****************************************************************
Name:	nFreePortHandles

Inputs:
	None.

Return Value:
	int - 0 if fails, 1 if passes

Description:
	This routine frees all port handles that need to be freed
	using the PHF command.
*****************************************************************/
int p_nFreePortHandles( )
{
	int
		nNoHandles = 0,
		nHandle = 0,
		n = 0,
		i = 0;
	char
		szHandleList[MAX_REPLY_MSG];

	/* get all the handles that need freeing */
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "PHSR 01" );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if (!p_nGetResponse( ))
			return 0;
		if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
			return 0;

		sprintf( szHandleList, "%s", pCommandAurora->m_szLastReply );
		nNoHandles = uASCIIToHex( &szHandleList[n], 2 );
		n+=2;
		for ( i = 0; i < nNoHandles; i++ )
		{
			nHandle = uASCIIToHex( &szHandleList[n], 2 );
			memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
			sprintf( pCommandAurora->m_szCommand, "PHF %02X", nHandle );
			n+=5;
			if (!p_nSendMessage( pCommandAurora->m_szCommand, true ))
				return 0;
			if ( !p_nGetResponse() )
				return 0;
			if (!p_nCheckResponse(p_nVerifyResponse(pCommandAurora->m_szLastReply, true )))
				return 0;
			pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bInitialized = false;
			pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bEnabled = false;
			/* EC-03-0071 */
			memset( pCommandAurora->m_dtHandleInformation[nHandle].szPhysicalPort, 0, 5 );
		} /* for */
		return 1;
	} /* if */

	return 0;
} /* nFreePortHandles */

/*****************************************************************
Name:	nGetPortInformation

Inputs:
	int nPortHandle - the handle to get information for

Return Value:
	int - 1 if successful, 0 otherwise

Description:
	This routine gets the port handling information for the supplied
	handle.  It uses the PHINF call to get this information.
*****************************************************************/
int p_nGetPortInformation( int nPortHandle )
{
	unsigned int
		uASCIIConv = 0;
	char
		*pszPortInformation = NULL;
    int i;

	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "PHINF %02X0021", nPortHandle );

	if ( p_nSendMessage( pCommandAurora->m_szCommand, true ) )
	{
		if ( p_nGetResponse() )
		{
			if ( !p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
				return 0;

			pszPortInformation = pCommandAurora->m_szLastReply;

			strncpy( pCommandAurora->m_dtHandleInformation[nPortHandle].szToolType, pszPortInformation, 8 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].szToolType[8] = '\0';
			pszPortInformation+=8;
			strncpy( pCommandAurora->m_dtHandleInformation[nPortHandle].szManufact, pszPortInformation, 12 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].szManufact[12] = '\0';
			pszPortInformation+=12;
			strncpy( pCommandAurora->m_dtHandleInformation[nPortHandle].szRev, pszPortInformation, 3 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].szRev[3] = '\0';
			pszPortInformation+=3;
			strncpy( pCommandAurora->m_dtHandleInformation[nPortHandle].szSerialNo, pszPortInformation, 8 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].szSerialNo[8] = '\0';
			pszPortInformation+=8;
			uASCIIConv = uASCIIToHex( pszPortInformation, 2 );
			pszPortInformation+=2;
			pCommandAurora->m_dtHandleInformation[nPortHandle].HandleInfo.bToolInPort = ( uASCIIConv & 0x01 ? 1 : 0 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].HandleInfo.bGPIO1 = ( uASCIIConv & 0x02 ? 1 : 0 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].HandleInfo.bGPIO2 = ( uASCIIConv & 0x04 ? 1 : 0 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].HandleInfo.bGPIO3 = ( uASCIIConv & 0x08 ? 1 : 0 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].HandleInfo.bInitialized = ( uASCIIConv & 0x10 ? 1 : 0 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].HandleInfo.bEnabled = ( uASCIIConv & 0x20 ? 1 : 0 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].HandleInfo.bTIPCurrentSensing = ( uASCIIConv & 0x80 ? 1 : 0 );

			pszPortInformation+=10;
			strncpy( pCommandAurora->m_dtHandleInformation[nPortHandle].szPhysicalPort, pszPortInformation, 2 );
			/* EC-03-0071
			pCommandAurora->m_dtHandleInformation[nPortHandle].szPhysicalPort[2] = '\0';
			 */
			pszPortInformation+=2;
			strncpy( pCommandAurora->m_dtHandleInformation[nPortHandle].szChannel, pszPortInformation, 2 );
			pCommandAurora->m_dtHandleInformation[nPortHandle].szChannel[2] = '\0';
			if ( !strncmp( pCommandAurora->m_dtHandleInformation[nPortHandle].szChannel, "01", 2 ) )
			{
				/* EC-03-0071
				strncat(pCommandAurora->m_dtHandleInformation[nPortHandle].szPhysicalPort, "-b", 2 );
				 */
				strncpy(&pCommandAurora->m_dtHandleInformation[nPortHandle].szPhysicalPort[2], "-b", 2 );
				for ( i = 0; i < NO_HANDLES; i++ )
				{
					if ( strncmp( pCommandAurora->m_dtHandleInformation[i].szPhysicalPort, pCommandAurora->m_dtHandleInformation[nPortHandle].szPhysicalPort, 4 ) &&
						 !strncmp( pCommandAurora->m_dtHandleInformation[i].szPhysicalPort, pCommandAurora->m_dtHandleInformation[nPortHandle].szPhysicalPort, 2 ) )
						/* EC-03-0071
						strncat(pCommandAurora->m_dtHandleInformation[i].szPhysicalPort, "-a", 2 );
						 */
						strncpy( &pCommandAurora->m_dtHandleInformation[i].szPhysicalPort[2], "-a", 2 );
				} /* for */
			} /* if */
		} /* if */
		else
			return 0;
	} /* if */

	return 1;
} /* nGetPortInformation */

/*****************************************************************
Name:	nStartTracking

Inputs:
	None.

Return Value:
	int - 0 if fails, else nCheckResponse

Description:
	This routine starts that System tracking.  It uses
	the TSTART command to do this.
*****************************************************************/
int p_nStartTracking( )
{
	pCommandAurora->m_bDisplayErrorsWhileTracking=true;

	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "TSTART " );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if(!p_nGetResponse()) return 0;
		return p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) );
	}

	return 0;
} /* nStartTracking */


/*****************************************************************
Name:	nGetBXTransforms

Inputs:
	bool bReturnOOV - whether or not to return values outside
					  of the characterized volume.

Return Value:
	int - 1 if successful, 0 otherwise.

Description:
	This routine gets the transformation information using the BX
	command.  Remember that if you want to track outside the
	characterized volume you need to set the flag.
*****************************************************************/
int p_nGetBXTransforms( bool bReturnOOV )
{
	int
		nReplyMode = 0x0001,
		/*nReplySize = 0,*/
		nSpot = 0,
		nNoHandles = 0,
		nHandle = 0,
		i = 0;
	unsigned int
		unSystemStatus = 0,
		uTransStatus = 0,
		unHandleStatus = 0,
		uHeaderCRC = 0,
		uBodyCRC = 0,
		uCalcCRC = 0;
	char
		*pszTransformInfo = NULL;

	/* set reply mode depending on bReturnOOV */
	nReplyMode = bReturnOOV ? 0x0801 : 0x0001;

	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "BX %04X", nReplyMode );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if (!p_nGetBinaryResponse( ))
		{
			return 0;
		}

    if ( !p_nCheckResponse(p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) ) )
			return 0;

		pszTransformInfo = pCommandAurora->m_szLastReply;
		uCalcCRC = CRC_SystemGetCRC( pCommandAurora->m_szLastReply, 4 );

		/* check for preamble ( A5C4 ) */
    while(((pszTransformInfo[0]&0xff)!=0xc4))
		{
			pszTransformInfo++;
		}/* while */

		if ( (pszTransformInfo[0]&0xff)!=0xc4 || (pszTransformInfo[1]&0xff)!=0xa5 )
		{
			return REPLY_INVALID;
		}/* if */

		/* parse the header */
    nSpot+=2;
		/*nReplySize = nGetHex2(&pszTransformInfo[nSpot]);*/
		nSpot+=2;
		uHeaderCRC = nGetHex2( &pszTransformInfo[nSpot] );
    nSpot+=2;

		if ( uCalcCRC != uHeaderCRC )
		{
      p_nCheckResponse( REPLY_BADCRC ); /* display the Bad CRC error message */
			return REPLY_BADCRC;
		} /* if */

		nNoHandles = nGetHex1( &pszTransformInfo[nSpot] );
		nSpot++;

		for ( i = 0; i < nNoHandles; i++ )
		{
			nHandle = nGetHex1( &pszTransformInfo[nSpot] );
			nSpot++;

			uTransStatus = nGetHex1( &pszTransformInfo[nSpot] );
			nSpot++;

			if ( uTransStatus == 1 ) /* one means that the transformation was returned */
			{
				/* parse out the individual components by converting binary to floats */
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.q0 = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.qx = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.qy = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.qz = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.translation.x = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.translation.y = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.translation.z = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.fError = fGetFloat(&pszTransformInfo[nSpot]);
				nSpot+=4;
				unHandleStatus = nGetHex4(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.ulFrameNumber = nGetHex4(&pszTransformInfo[nSpot]);
				nSpot+=4;
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.ulFlags = TRANSFORM_VALID;
			} /* if */

			if ( uTransStatus == 2 || uTransStatus == 4 ) /* 2 means the tool is missing and */
														  /* 4 means DISABLED */
			{
				/*
				 * no transformation information is returned but the port status and time
				 * are return
				 */
				if ( uTransStatus == 2 )
				{
					unHandleStatus = nGetHex4(&pszTransformInfo[nSpot]);
					nSpot+=4;
					pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.ulFrameNumber = nGetHex4(&pszTransformInfo[nSpot]);
					nSpot+=4;
					pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.ulFlags = TRANSFORM_MISSING;
				} /* if */
				else
					pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.ulFlags = TRANSFORM_DISABLED;

				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.q0 =
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.qx =
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.qy =
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.rotation.qz =
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.translation.x =
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.translation.y =
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.translation.z =
				pCommandAurora->m_dtHandleInformation[nHandle].Xfrms.fError = BAD_FLOAT;
			}/* if */

			if ( uTransStatus == 1 || uTransStatus == 2 )
			{
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bToolInPort = ( unHandleStatus & 0x01 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bGPIO1 = ( unHandleStatus & 0x02 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bGPIO2 = ( unHandleStatus & 0x04 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bGPIO3 = ( unHandleStatus & 0x08 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bInitialized = ( unHandleStatus & 0x10 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bEnabled = ( unHandleStatus & 0x20 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bOutOfVolume = ( unHandleStatus & 0x40 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bPartiallyOutOfVolume = ( unHandleStatus & 0x80 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bDisturbanceDet = ( unHandleStatus & 0x200 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bSignalTooSmall = ( unHandleStatus & 0x400 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bSignalTooBig = ( unHandleStatus & 0x800 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bProcessingException = ( unHandleStatus & 0x1000 ? 1 : 0 );
				pCommandAurora->m_dtHandleInformation[nHandle].HandleInfo.bHardwareFailure = ( unHandleStatus & 0x2000 ? 1 : 0 );
			}/* if */
		} /* for */

		unSystemStatus = nGetHex2( &pszTransformInfo[nSpot] );
		nSpot+=2;
		uBodyCRC = nGetHex2(&pszTransformInfo[nSpot]);
		pCommandAurora->m_dtSystemInformation.bCommunicationSyncError = ( unSystemStatus & 0x01 ? 1 : 0 );
		pCommandAurora->m_dtSystemInformation.bTooMuchInterference = ( unSystemStatus & 0x02 ? 1 : 0 );
		pCommandAurora->m_dtSystemInformation.bSystemCRCError = ( unSystemStatus & 0x04 ? 1 : 0 );
		pCommandAurora->m_dtSystemInformation.bRecoverableException = ( unSystemStatus & 0x08 ? 1 : 0 );
		pCommandAurora->m_dtSystemInformation.bHardwareFailure = ( unSystemStatus & 0x10 ? 1 : 0 );
		pCommandAurora->m_dtSystemInformation.bHardwareChange = ( unSystemStatus & 0x20 ? 1 : 0 );
		pCommandAurora->m_dtSystemInformation.bPortOccupied = ( unSystemStatus & 0x40 ? 1 : 0 );
		pCommandAurora->m_dtSystemInformation.bPortUnoccupied = ( unSystemStatus & 0x80 ? 1 : 0 );

		uCalcCRC = CRC_SystemGetCRC(pszTransformInfo+=6, nSpot-6 );
		if ( uCalcCRC != uBodyCRC )
		{
			p_nCheckResponse( REPLY_BADCRC ); /* display the Bad CRC error message */
			return REPLY_BADCRC;
		} /* if */


	} /* if */

	return 1;
} /* nGetBXTransforms */


/*****************************************************************
Name:	nStopTracking

Inputs:
	None.

Return Value:
	int - 0 if fails, nCheckResponse if passes

Description:
	This routine stops the System's tracking by using
	the TSTOP call.
*****************************************************************/
int p_nStopTracking( )
{
	memset( pCommandAurora->m_szCommand, 0, sizeof(pCommandAurora->m_szCommand) );
	sprintf( pCommandAurora->m_szCommand, "TSTOP " );

	if(p_nSendMessage( pCommandAurora->m_szCommand, true ))
	{
		if(!p_nGetResponse()) return 0;
		return p_nCheckResponse( p_nVerifyResponse( pCommandAurora->m_szLastReply, true ) );
	} /* if */

	return 0;
} /* nStopTracking */


/*****************************************************************
Name:	nSendMessage

Inputs:
	char * m_szCommand - the command string, coming in.  The command
	to be sent to the System
	bool bAddCRC - if true, we add the CRC to the command and replace the
	space with the :

Return Value:
	int -  0 if fails, 1 if passes.

Description:
	This command takes in a command string and parses it depending
	on the value of bAddCRC.  If bAddCRC is true, we replace the
	space with a : and calculate and add the CRC to the command.
	We then send the command to the System.
*****************************************************************/
int p_nSendMessage( char *szCommand, bool bAddCRC )
{
	unsigned int i;
	bool bComplete = false;
	struct timeval		t1,t2;

	gettimeofday ( &t1, NULL );
	/* Check COM port */
	if( pCOMPort == NULL )
	{
		return bComplete;
	}/* if */

	/* build the command, by adding a carraige return to it and crc if specified */
	if (!p_nBuildCommand( szCommand, bAddCRC ))
		return bComplete;

	if(strlen(szCommand) >= (MAX_COMMAND_MSG))
		return bComplete;

	for( i = 0; i < strlen(szCommand); i++ )
	{
		if ( Comm32Port_SerialPutChar( szCommand[i] ) == 0 )
		{
			bComplete = false;
			if(DEB) printf( "carac passe pas: %c\n", szCommand[i] );
			break;
		} /* if */
		else if( szCommand[i] == CARRIAGE_RETURN )
		{
			bComplete = true;
			break;
		}/* if */
	} /* for */


	if (pCommandAurora->m_bLog)
	{
		if (DEB_COMM) printf( "SERIAL PORT : SENDING : %s \n", szCommand );
	}

	gettimeofday ( &t2, NULL );
	if (DEB_COMM) printf( "SERIAL PORT : SENDING TIME : %d\n", (int)
        (( t2.tv_sec - t1.tv_sec )*1000 +	(t2.tv_usec - t1.tv_usec )/1000) );
	return bComplete;
} /* nSendMessage */

/*****************************************************************
Name:	nGetResponse

Inputs:
	None.

Return Value:
	int - 1 if passes, else 0

Description:
	This routine gets the response from the system that is to be
	poled through the com port.  The routine gets it start time
	( the time it started polling ) and continues until one of two
	things happens.  First, if the end of response (Carriage Return)
	if found the reply is complete and 1 is returned.  If the time
	it takes to get a response exceeds the timeout value, the system
	assumes no response is coming and timeouts.  The timeout dialog
	is then displayed.  Use this response routine for all calls except
	the BX call.
*****************************************************************/
int p_nGetResponse( )
{
	char
		chChar,
		c;
	struct timeval
		currenttime,
		starttime;
	bool
		bDone = false;
	int
		nCount = 0,
		nRetry = 0;
    double diff;

	/* Check COM port */
	if( pCOMPort == NULL )
  {
		return false;
  }

	gettimeofday ( &starttime, NULL );

  memset( pCommandAurora->m_szLastReply, 0, sizeof( pCommandAurora->m_szLastReply ) );

	do
	{
		while ( (Comm32Port_SerialCharsAvailable( ) > 0) && (!bDone) )
		{
      chChar = Comm32Port_SerialGetChar();
			/* if carriage return, we are done */
			if ( chChar == '\r' )
   			{
				pCommandAurora->m_szLastReply[nCount] = CARRIAGE_RETURN;
				pCommandAurora->m_szLastReply[nCount+1] = '\0';
			   	bDone = true;
			} /* if */
			else
			{
				pCommandAurora->m_szLastReply[nCount] = chChar;
				nCount++;
			} /* else */
		} /* while */

		if ( !bDone )
		{
			/*
			 * Get the current time and compare with start time
			 * if longer "timeout" assume no response and timeout
			 */
			gettimeofday( &currenttime, NULL );
			diff = ((currenttime.tv_sec - starttime.tv_sec)*1000 + (currenttime.tv_usec - starttime.tv_usec)/1000);

			if ( diff >= pCommandAurora->m_nTimeout )
			{
        if (DEB) printf( "diff %f | timeout %d\n", diff, pCommandAurora->m_nTimeout );
				/*
				 * If a COM port timeout is noted, we will try to
				 * send the command again, up to MAX_COMM_ATTEMPTS times.
				 */

        if (DEB) printf( "SERIAL PORT : TIMEOUT for command: %s \n", pCommandAurora->m_szCommand );
				nCount = 0;
				if( nRetry < MAX_COMM_ATTEMPTS )
				{
					nRetry++;
					memset( pCommandAurora->m_szLastReply, 0, sizeof(pCommandAurora->m_szLastReply) );

					/*
					 * Do not clear the m_szCommand at this point, since
					 * we are re-sending the same command.
					 */
					p_nSendMessage( pCommandAurora->m_szCommand, false ); /* Command already has CRC */

					/* Reset the start time. */
          gettimeofday( &starttime, NULL );
				}
				else
				{
					/*
					 * if the user chooses to retry sending the command
					 * handle that here.
					 */
          if (DEB) printf( "SERIAL PORT : TIMEOUT AFTER %d RETRIES for command: %s \n", MAX_COMM_ATTEMPTS, pCommandAurora->m_szCommand );

					printf( "System seems not to respond. Retry ? (o/N)" );
                    c = getchar();

					if ( c=='o'||c=='O' )
					{
						if ( strlen(pCommandAurora->m_szCommand) > 0 )
						{
							nRetry = 1;
							memset( pCommandAurora->m_szLastReply, 0, sizeof(pCommandAurora->m_szLastReply) );

							/*
							 * Do not clear the m_szCommand at this point, since
							 * we are re-sending the same command.
							 */
							p_nSendMessage( pCommandAurora->m_szCommand, false ); /* Command already has CRC */

							/* Reset the start time. */
              gettimeofday( &starttime, NULL );
						}
						else
						{
							Comm32Port_SerialBreak( );
						} /* else */
					}
					else
					{
						return false;
					}/* else */
				}/* else */
			}/* if */
		} /* if */
	} while ( !bDone );


	if (DEB_COMM) printf( "SERIAL PORT : RECEIVED : %s \n", pCommandAurora->m_szLastReply );
	gettimeofday ( &currenttime, NULL );
	if (DEB_COMM) printf( "SERIAL PORT : RECEIVED TIME : %d msec \n",
	(int)((currenttime.tv_sec - starttime.tv_sec)*1000 + (currenttime.tv_usec - starttime.tv_usec)/1000) );


	return 1;
} /* nGetResponse */


/*****************************************************************
Name:	nGetBinaryResponse

Inputs:
	None.

Return Value:
	int - 1 if passes, else 0

Description:
	This routine gets the response from the system that is to be
	poled through the com port.  The routine gets its start time
	( the time it started polling ) and continues until one of two
	things happens.  First, if the end of response ( the number of bytes
	specified in the header is found ) the reply is complete and 1 is
	returned.  If the time it takes to get a response exceeds the timeout
	value, the system assumes no response is coming and timeouts.  The
	timeout dialog 	is then displayed.  Use this response routine for
	all calls except the BX call.
*****************************************************************/
int p_nGetBinaryResponse( )
{
	char
		chChar;
	struct timeval
		currenttime,
		starttime;
	bool
		bDone = false;
	int
		nTotalBinaryLength = -1,
		nCount = 0,
		nRetry = 0;
    double diff;

    memset( pCommandAurora->m_szLastReply, 0, sizeof( pCommandAurora->m_szLastReply ) );


	/*
	 * Get the start time that the call was initialized.
	 */
	gettimeofday( &starttime, NULL );

	do
	{
		/* Check COM port */
		if( pCOMPort == NULL )
		{
			return false;
		}/* if */

		while ( (Comm32Port_SerialCharsAvailable() > 0) && (!bDone) )
		{
			chChar = Comm32Port_SerialGetChar( );

			pCommandAurora->m_szLastReply[nCount] = chChar;

			/*
			 * Get the total length of the buffer
			 */
			if ( nCount == 3 )
			{
				/* + 7 to account for header information */
				nTotalBinaryLength = nGetHex2( &pCommandAurora->m_szLastReply[2] ) + 7 + 1;
			}/* if */

			nCount++;

			if ( nCount == nTotalBinaryLength )
			{
			   	bDone = true;
			}/* if */
		} /* while */

		if ( !bDone )
		{
			/*
			 * Get the current time and compare with start time
			 * if longer "timeout" assume no response and timeout
			 */
			gettimeofday( &currenttime, NULL );
			diff = 10000*((currenttime.tv_sec+currenttime.tv_usec/1000000.0)
                    - (starttime.tv_sec+starttime.tv_usec/1000000.0));
			if ( diff >= pCommandAurora->m_nTimeout )
			{
				/*
				 * If a COM port timeout is noted, we will try to
				 * send the command again, up to 3 times.
				 */
				nCount = 0;
				if( nRetry < 3 )
				{
					nRetry++;
					memset( pCommandAurora->m_szLastReply, 0, sizeof(pCommandAurora->m_szLastReply) );

					/*
					 * Do not clear the m_szCommand at this point, since
					 * we are re-sending the same command.
					 */
					p_nSendMessage( pCommandAurora->m_szCommand, false ); /* Command already has CRC */

					 /* Reset the start time. */
					gettimeofday( &starttime, NULL );
				}
				else
				{
					return false;
				}/* else */
			}/* if */
		} /* if */
	} while ( !bDone );


  if (pCommandAurora->m_bLog)
	{
		if (DEB_COMM) printf( "SERIAL PORT : RECEIVED : %s \n", pCommandAurora->m_szLastReply );
		gettimeofday ( &currenttime, NULL );
		if (DEB_COMM) printf( "SERIAL PORT : RECEIVED TIME : %d msec \n",
        (int)((currenttime.tv_sec - starttime.tv_sec)*1000 + (currenttime.tv_usec - starttime.tv_usec)/1000) );
	}

	return bDone;

} /* nGetBinaryResponse */


/*****************************************************************
Name:	nCheckResponse

Inputs:
	int nReturnedValue - the value returned by nVerifyResponse

Return Value:
	int - 1 if the response is valid, 0 if the response is invalid
	or an error.

Description:
	This routine checks the value from nVerifyResponse.
	The following occurs:
	REPLY_ERROR - the response from the system was an error, we
		beep the system if required and post the error
		message ( ErrorMessage() )
	REPLY_BADCRC - a bad crc was returned with the response
		i.e. the crc returned doesn't match the one
		calculated for the response. Post a message
	REPLY_WARNING - the warning message was recieve from the system
		while intializing a tool (see API for reasons)
		post a message and beep if required.
	REPLY_INVALID - an invalid response was recieved from the system
		post a message
*****************************************************************/
int p_nCheckResponse( int nReturnedValue )
{

	if ( nReturnedValue == REPLY_ERROR )
	{
		p_ErrorMessage( );
    if ( bBeepOnError )
			p_nBeepSystem( nNoErrorBeeps );
		return 0;
	} /* if */

	if ( nReturnedValue == REPLY_BADCRC )
	{
		if(DEB) printf( "Bad CRC\n" );
		return 0;
	} /* if */

	if ( nReturnedValue == REPLY_WARNING )
	{
		if (bBeepOnWarning)
			p_nBeepSystem( nNoWarningBeeps );

		if(DEB) printf( "System warning while port initialization\n" );
		return 1;
	} /* if */

	if ( nReturnedValue == REPLY_INVALID )
	{
		if(DEB) printf( "Invalid response received\n" );
		return 0;
	} /* if */

	return 1;
} /* nCheckResponse */

/*****************************************************************
Name:	nVerifyResponse

Inputs:
	char * pszReply - the reply to verify
	bool bCheckCRC - perform the CRC check on the response

Return Value:
	int - the response that is found, defined in the APIStructures.h

Description:
	This routine checks the given response for the predetermined
	response values and returns the corresponding reply value.
*****************************************************************/
int p_nVerifyResponse( char *pszReply, bool bCheckCRC )
{
	int
		nResponse = 0;

	/* verify the response by comparing it with the possible responses */
	if (!strncmp(pszReply, "RESET",5))
		nResponse = REPLY_RESET;
	else if (!strncmp(pszReply, "OKAY",4))
		nResponse = REPLY_OKAY;
	else if (!strncmp(pszReply, "ERROR",5))
		nResponse = REPLY_ERROR;
	else if (!strncmp(pszReply, "WARNING",7))
		nResponse = REPLY_WARNING;
	else if ( strlen( pszReply ) > 0 )
		nResponse = REPLY_OTHER;
	else
		return REPLY_OTHER;

	if ( (nResponse & REPLY_OKAY  || nResponse & REPLY_OTHER) && bCheckCRC )
	{
		if (!CRC_SystemCheckCRC( pszReply ) )
			return REPLY_BADCRC;
		else
			return nResponse;
	} /* if */
	else
		return nResponse;
} /* nVerifyResponse */

/*****************************************************************
Name:	ErrorMessage

Inputs:
	None.

Return Value:
	None.

Description:
	This routine loads and displays the error that
	corresponds with the reply string from the system.
*****************************************************************/
void p_ErrorMessage( )
{
	char pchErrorMessage[256], pchErrorNumber[8];

	strncpy( pchErrorNumber, pCommandAurora->m_szLastReply, 7 );
	pchErrorNumber[7] = '\0';

	strncpy( pchErrorMessage, pCommandAurora->m_szLastReply, 7 );
	pchErrorNumber[7] = '\0';

	if(DEB) printf( "Erreur : %s \n", pchErrorNumber );

} /* ErrorMessage */


/*****************************************************************

Name:	AddCRCToCommand

Input Values:
	char *pszCommandString - the message to have a CRC added
	remeber, we substitute the space with a :

Output Values:
	char *

Return Value:
	int - 1 if succesful, 0 otherwise

Description:
	This function adds a CRC to the end of a command
	and replaces the space with a :.  The CRC is calc'd
	using the CRC functionality found in the API
*****************************************************************/
int p_nAddCRCToCommand( char * pszCommandString )
{
	unsigned int m,n;
	unsigned int uCrc;

	if(strlen(pszCommandString) >= (MAX_COMMAND_MSG-6))
		return 0;

	n = (unsigned int) strlen(pszCommandString);
	/*
	 * determine 16 bit CRC
	 */
	uCrc = 0;
	for( m=0 ; m<n ; ++m )
	{
		/*
		 * replace space character with : if sending CRC
		 */
		if(pszCommandString[m]==' ')
			pszCommandString[m]=':';
		uCrc = CRC_CalcCrc16( uCrc,pszCommandString[m] );
	} /* for */
	sprintf( &pszCommandString[n],"%04X",uCrc );
	n+=4;

	return 1;
} /* nAddCRCToCommand */

/*****************************************************************

Name:	AddCRToCommand

Input Values:
	char *pszCommandString - the message to have a carriage
	return added to the end
	remeber, all command need a carriage return

Output Values:
	char *

Return Value:
	int - 1 if successful, 0 otherwise

Description:
	This function adds a carriage return  to the end of
	a command
*****************************************************************/
int p_nAddCRToCommand( char * pszCommandString )
{
	unsigned int n;

	if(strlen(pszCommandString) >= (MAX_COMMAND_MSG-1))
		return 0;

	n = (unsigned int) strlen(pszCommandString);
	pszCommandString[n++] = CARRIAGE_RETURN;
	pszCommandString[n++] = '\0';
	return 1;
} /* nAddCRToCommand */

/*****************************************************************

Name:	nBuildCommand

Input Values:
	char *pszCommandString - the message to be built
	bool bAddCRC - whether or not to add the CRC to the command

Output Values:
	char *

Return Value:
	int - 1 if successful, 0 otherwise

Description:
	This routine builds the message.  If bAddCRC is true, replace
	the space with a : and add the commands CRC to the end of it.
*****************************************************************/
int p_nBuildCommand( char *pszCommandString, bool bAddCRC )
{
	int nCnt = 0;

	for ( nCnt = 0; nCnt < MAX_COMMAND_MSG; nCnt++)
	{
		if( pszCommandString[nCnt++] == CARRIAGE_RETURN )
		{
			pszCommandString[nCnt++] = '\0';
			return 1;
		}/* if */
	}/* if */

	if ( bAddCRC )
		if (!p_nAddCRCToCommand( pszCommandString ))
			return 0;

	if (!p_nAddCRToCommand( pszCommandString ))
		return 0;

	return 1;
} /* nBuilCommand */

/*
 *	MAIN USER ROUTINES: START, STOP, UPDATE
 */

unsigned char										polaris_initialized = 0;
unsigned char										polaris_tracking = 0;

int polaris_start( ) {
	 
	/* Polaris initialization */

	p_construct( true, POLARIS_INI_TIMEOUT );
	
	// Open port
	if ( !p_nOpenComPort( POLARIS_SERIAL_PORT ) )
	{
		if (DEB) printf(	"polaris_start: COM Port could not be opened.\n"
											"Check your COM Port settings/access right and\n"
											"make sure you system is turned on.\n" );
		polaris_stop( );
		return -1;
	}
	else
	if (DEB) printf( "polaris_start: serial port opened.\n" );

	// Hardware reset
	if( p_nHardWareReset( ) ) {
		if(DEB) printf( "polaris_start: hardware reset done.\n" );
	}
	else {
		if (DEB) printf( "polaris_start: hardware reset failed.\n" );
		polaris_stop( );
		return -1;
	}
	
	// Set serial port speed
	if ( p_nSetSystemComParms( 	POLARIS_SERIAL_BR, 
															POLARIS_DATA_BITS,
															POLARIS_PARITY,
															POLARIS_STOP_BITS,
															POLARIS_HARD_FLOW ) )  {
		if ( p_nSetCompCommParms( POLARIS_SERIAL_BR,
															POLARIS_DATA_BITS,
															POLARIS_PARITY, 
															POLARIS_STOP_BITS,
															POLARIS_HARD_FLOW ) )	{
			if ( p_nInitializeSystem( ) )	{
				if ( !p_nGetSystemInfo( ) )	{
					if (DEB) printf( "polaris_start: could not determine type of system.\n" );
					polaris_stop( );
					return -1;
				}
				else if ( ( pCommandAurora->m_dtSystemInformation.nTypeofSystem == POLARIS_SYSTEM ) ||
									( pCommandAurora->m_dtSystemInformation.nTypeofSystem == ACCEDO_SYSTEM ) )	{
					if ( p_nSetFiringRate( POLARIS_FIRING_RATE ) )	{
						if (DEB) printf( "polaris_start: Polaris system successfully intialized.\n");
						polaris_initialized = 1;
					}
					else {
						if (DEB) printf( "polaris_start: firing rate could not be set.\n" );
						polaris_stop( );
						return -1;
					}
				}
			}
			else
			{
				if (DEB) printf( "polaris_start: system could not be initialized.\n" );
				polaris_stop( );
				return -1;
			}
		} 
		else
		{
			if (DEB) printf( "polaris_start: unable to configure host serial settings.\n" );
			polaris_stop( );
			return -1;
		}
	}
	else
	{
		if (DEB) printf( "polaris_start: unable to configure system serial settings.\n" );
		polaris_stop( );
		return -1;
	}

	/* Start tracking */
	
	if (DEB) printf( "polaris_start: activating all ports.\n" );
	if ( p_nActivateAllPorts( ) ) {}
	else {
		if (DEB) printf( "polaris_start: Unable to initialize the tool.\n" );
		polaris_stop( );
		return -1;
	}
	
	if (DEB) printf( "polaris_start: all ports activated.\n" );

	if ( p_nStartTracking( ) ) {
		if (DEB) printf( "polaris_start: tracking ON.\n" );
		polaris_tracking = 1;
		return 0;
	} 
	else {
		if (DEB) printf( "polaris_start: cannot start tracking." );
		polaris_stop( );
		return -1;
	}
}

void polaris_stop( ) {
	
	if (DEB) printf( "polaris_stop: closing.\n" );
	
	if ( polaris_tracking )	{
		p_nStopTracking( );
		polaris_tracking = 0;
	}

	if ( polaris_initialized ) {
		p_nSetSystemComParms( POLARIS_SERIAL_BR, 
													POLARIS_DATA_BITS,
													POLARIS_PARITY,
													POLARIS_STOP_BITS,
													POLARIS_HARD_FLOW  );
		p_nSetCompCommParms( 	POLARIS_SERIAL_BR, 
													POLARIS_DATA_BITS,
													POLARIS_PARITY,
													POLARIS_STOP_BITS,
													POLARIS_HARD_FLOW  );
		polaris_initialized = 0;
	}

	p_nCloseComPorts( );

	p_destruct( );
}

int polaris_update( )	{
	
	// Ask for an update
	
	if ( !p_nGetBXTransforms( true ) )
		return -1;
	
	return 0;
}
