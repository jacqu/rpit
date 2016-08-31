/*
 * Copyright 1994-2010 The MathWorks, Inc.
 *
 * File: rtiostream_tcpip.c     $Revision: 1.1.6.13 $
 *
 * Abstract: This source file implements both client-side and server-side TCP/IP
 *  communication. Typically, this driver is used to support host-target
 *  communication where the client-side device driver runs on the host and the
 *  server-side driver runs on the target. For this implementatation, both
 *  client-side and server-side driver code has been combined into a single
 *  file.
 *
 *  If you are using this code as a starting point to implement a TCP/IP driver
 *  for a custom target it is only necessary to include code for the server side
 *  of the connection.
 */

#undef  WIN32

#ifndef _WIN32
/* Required BSD Unix extensions are not available by default on certain Unix
 * distributions */
#define _BSD_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include "rtiostream.h"

#ifdef _WIN32
  /* WINDOWS */
# include <windows.h>

# ifdef __LCC__
#   include <winsock2.h>
#   include <errno.h>
# endif

#define RTIOSTREAM_ECONNRESET WSAECONNRESET

#elif defined(VXWORKS)
 /*VxWorks headers*/
# include <selectLib.h>
# include <sockLib.h>
# include <inetLib.h>
# include <ioLib.h>
# include <taskLib.h>
# include <netinet/tcp.h> 

#define RTIOSTREAM_ECONNRESET ECONNRESET

#else
  /* UNIX */
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/syspage.h>
#include <netdb.h>
#include <netinet/in.h>
//#include <netinet/tcp.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <semaphore.h>

#define RTIOSTREAM_ECONNRESET ECONNRESET
#endif

#ifdef USE_MEXPRINTF
#include "mex.h"
#define printf mexPrintf
#define SERVER_PORT_PRINTF(FORMAT, ARG1) mexPrintf(FORMAT, ARG1)
#else
/* If stdout is redirected to file, it is essential that the port number is 
 * available immediately in the output file. With LCC, printf does not flush 
 * correctly to the redirected output file - use fprintf & fflush instead. */
#define SERVER_PORT_PRINTF(FORMAT, ARG1) fprintf(stdout, FORMAT, ARG1); \
                                         fflush(stdout)
#endif

/***************** DEFINES ****************************************************/

#define SERVER_STREAM_ID (1) /* Allow a single server-side connection */

#define HOSTNAME_MAXLEN (64U)

#define SERVER_PORT_NUM  (17725U)   /* sqrt(pi)*10000 */

#define  TMP_BUF_SIZ (40)

#define DEFAULT_RECV_TIMEOUT_SECS (1)

/* 
 * EXT_BLOCKING  
 *
 * Depending on the implementation of the main program (e.g., grt_main.c,
 * rt_main.c), the EXT_BLOCKING flag must be set to either 0 or 1.
 *
 * rt_main.c (tornado/vxworks): rt_main.c is a real-time, multi-tasking target.
 * The upload and packet servers are called via background (low priority) tasks.
 * In this case, it is o.k. for the transport function to block as the blocked
 * tasks will simply be pre-empted in order to enable the model to run.  It is
 * desirable to block instead of to poll to free up the cpu for any other
 * potential work. 
 */
//#ifdef VXWORKS
#ifdef QNX_OS                                         
# define EXT_BLOCKING (1)  
#else
# define EXT_BLOCKING (0)  
#endif


#ifdef VXWORKS
extern SEM_ID commSem;
#endif
#ifdef QNX_OS
extern sem_t *commSem;
#endif
#ifdef WIN32
  /* WINDOWS */
# define close closesocket
# define SOCK_ERR SOCKET_ERROR
#else
  /* UNIX, VXWORKS */
# define INVALID_SOCKET (-1)
# define SOCK_ERR (-1)

  typedef int SOCKET;
#endif

/***************** TYPEDEFS **************************************************/

typedef struct ServerCommsData_tag {
    int       port;
    SOCKET    listenSock;     /* listening socket to accept incoming connections */
    SOCKET    sock;           /* socket to send/receive packets */
    char      *serverInfoFile; /* the filename that is used to write the server 
                                 port number when dynamic port allocation is used                               */                      
} ServerCommsData;

/**************** LOCAL DATA *************************************************/

static ServerCommsData ServerData = {SERVER_PORT_NUM, INVALID_SOCKET, INVALID_SOCKET, NULL};
static unsigned int Blocking = EXT_BLOCKING;
static unsigned int RecvTimeoutSecs = DEFAULT_RECV_TIMEOUT_SECS;

/************** LOCAL FUNCTION PROTOTYPES ************************************/

static int socketDataSet(
    const SOCKET sock,
    const void *src,
    const size_t size,
    size_t *sizeSent);

static int socketDataGet(
    const SOCKET   sock,
    char          *dst,
    const size_t   size,
    size_t        *sizeRecvd);

static int socketDataPending(
    const SOCKET sock,
    int    *outPending,
    unsigned int timeoutSecs);

static int serverStreamRecv( 
    void * dst,
    size_t size,
    size_t * sizeRecvd);

static int serverStreamOpen(void);

#if (!defined(VXWORKS))
static SOCKET clientStreamOpen(char * hostName, unsigned int portNum);
#endif

static SOCKET serverOpenConnection(void);

static int processArgs(
    const int      argc,
    void         * argv[],
    char        ** hostName, 
    unsigned int * portNum,
    unsigned int * isClient,
    unsigned int * isBlocking,
    unsigned int * recvTimeout);

#if (!defined(VXWORKS))
static struct hostent * nameLookup(char * hostName);
#endif

/*************** LOCAL FUNCTIONS **********************************************/

/* Function: socketDataPending =================================================
 * Abstract:
 *  Returns true, via the 'pending' arg, if data is pending on the comm line.
 *  Returns false otherwise.
 *
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR on failure.
 */
static int socketDataPending(
    const SOCKET sock,
    int    *outPending, 
    unsigned int timeoutSecs)
{
    fd_set          ReadFds;
    int             pending;
    struct timeval  tval;
    int retVal = RTIOSTREAM_NO_ERROR;
    
    FD_ZERO(&ReadFds);
    FD_SET(sock, &ReadFds);

    tval.tv_sec  = timeoutSecs;
    tval.tv_usec = 0;

    /*
     * Casting the first arg to int removes warnings on windows 64-bit
     * platform.  It is safe to cast a SOCKET to an int here because on
     * linux SOCKET is typedef'd to int and on windows the first argument
     * to select is ignored (so it doesn't matter what the value is).
     */
    pending = select((int)(sock + 1), &ReadFds, NULL, NULL, &tval);
    if (pending == SOCK_ERR) {
        retVal = RTIOSTREAM_ERROR;
    }

    *outPending = (pending==1);
    return(retVal);    

} /* end socketDataPending */ 


/* Function: socketDataGet =====================================================
 * Abstract:
 *  Attempts to gets the specified number of bytes from the specified socket.
 *  The number of bytes read is returned via the 'sizeRecvd' parameter.
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR is returned on
 *  failure.
 *
 * NOTES:
 *  o it is not an error for 'sizeRecvd' to be returned as 0
 *  o this function blocks if no data is available
 */
static int socketDataGet(
    const SOCKET   sock,
    char          *dst,
    const size_t   size,
    size_t        *sizeRecvd)
{
    int nRead;
    int retVal;

    nRead = recv(sock, dst, (int)size, 0U);

    if (nRead == SOCK_ERR) {
        retVal = RTIOSTREAM_ERROR;
    } else {
        retVal = RTIOSTREAM_NO_ERROR;
    }

    if (retVal!=RTIOSTREAM_ERROR) {
        *sizeRecvd = (size_t) nRead;
    }

    return retVal;
} /* end socketDataGet */ 


/* Function: socketDataSet =====================================================
 * Abstract:
 *  Utility function to send data via the specified socket
 */
static int socketDataSet(
    const SOCKET sock,
    const void *src,
    const size_t size,
    size_t *sizeSent)
{
    int nSent;    
    int sizeLim;
    int retVal = RTIOSTREAM_NO_ERROR;
    
    /* Ensure size is not out of range for socket API send function */
    if (size > (size_t) INT_MAX) {
        sizeLim = INT_MAX;
    } else {
        sizeLim = (int) size;
    }

#ifndef VXWORKS
    nSent = send(sock, src, sizeLim, 0);
    if (nSent != sizeLim) {
       printf("########### Error sending data to host ###########\n");
       exit(-1);
    }
   
#else
    /*
     * VXWORKS send prototype does not have src as const.  This suppresses
     * the compiler warning.
     */
    nSent = send(sock, (char *)src, sizeLim, 0);
#endif
    if (nSent == SOCK_ERR) {
        retVal = RTIOSTREAM_ERROR;
    } else { 
        *sizeSent = (size_t)nSent;
    }

    return retVal;
}

/* Function: serverStreamRecv =================================================
 * Abstract:
 *  Send data from the server-side
 */
static int serverStreamRecv( 
    void * dst,
    size_t size,
    size_t * sizeRecvd)
{
    int retVal = RTIOSTREAM_NO_ERROR;
    int pending = 1;

    *sizeRecvd = 0;

    if (ServerData.sock == INVALID_SOCKET) {
        
        /* Attempt to open connection */
        ServerData.sock = serverOpenConnection();
    }   

    if (ServerData.sock != INVALID_SOCKET) {
        
        if (Blocking==0) {
            retVal = socketDataPending(ServerData.sock, &pending, 0);
        }
        
        if ( (pending !=0) && (retVal==RTIOSTREAM_NO_ERROR) && (size>0) ) {
            
            retVal = socketDataGet(ServerData.sock, (char *)dst, size, sizeRecvd);
            
            if (*sizeRecvd == 0) {
                
                if (errno == RTIOSTREAM_ECONNRESET) {
                    /* If we are closing the connection and we received this
                     * error, it means the other side of the connection was
                     * already closed.  Since we are expecting this, we can
                     * ignore this particular error.
                     */
                    retVal = RTIOSTREAM_NO_ERROR;
                } else {
                    /* Connection closed gracefully by client */
                }

                close(ServerData.sock);
                ServerData.sock = INVALID_SOCKET;
            }
        }
        
        if ( retVal == RTIOSTREAM_ERROR ) {
            
            close(ServerData.sock);
            ServerData.sock = INVALID_SOCKET;
        }
    }

    return retVal;
}

/* Function: serverStreamOpen =================================================
 * Abstract:
 *  Opens the listening socket to be used for accepting an incoming connection.
 */
static int serverStreamOpen(void)
{

    struct sockaddr_in serverAddr;
    int sockStatus;
#if (!defined(_WIN32)) && (!defined(VXWORKS)) && (!defined(QNX_OS))
    
    socklen_t sFdAddSize     = (socklen_t) sizeof(struct sockaddr_in);
#else
    int sFdAddSize     = (int) sizeof(struct sockaddr_in);
#endif
    SOCKET lFd         = ServerData.listenSock;
    int option         = 1;     
    int streamID       = SERVER_STREAM_ID;

    
    if (lFd == INVALID_SOCKET) {
        /*
         * Create a TCP-based socket.
         */
        memset((void *) &serverAddr,0,(size_t)sFdAddSize);
        serverAddr.sin_family      = AF_INET;
        serverAddr.sin_port        = htons((unsigned short int) ServerData.port);
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        
        lFd = socket(AF_INET, SOCK_STREAM, 0);

        if (lFd == INVALID_SOCKET) {
            printf("socket() call failed.\n");
        } else {
            printf("Socket created\n");
            /*
             * Listening socket should always use the SO_REUSEADDR option
             * ("Unix Network Programming - Networking APIs:Sockets and XTI",
             *   Volume 1, 2nd edition, by W. Richard Stevens).
             */
            sockStatus = 
                setsockopt(lFd,SOL_SOCKET,SO_REUSEADDR,(char*)&option,sizeof(option));
            if (sockStatus == SOCK_ERR) {
                printf("setsocketopt() call failed.\n");
                close(lFd);
                lFd = INVALID_SOCKET;
            }
            /* Disable Nagle's Algorithm*/ 
            #if 0
            sockStatus = 
               setsockopt(lFd,IPPROTO_TCP,TCP_NODELAY,(char*)&option,sizeof(option));
            if (sockStatus == SOCK_ERR) { 
                printf("setsocketopt() TCP_NODELAY call failed.\n");
                close(lFd); 
                lFd = INVALID_SOCKET; 
            } 
            #endif
                    
                    
        }
        
        
        if (lFd != INVALID_SOCKET) {
           
            sockStatus = bind(lFd, (struct sockaddr *) &serverAddr, sFdAddSize);
            
            if (sockStatus == SOCK_ERR) {
                printf("bind() call failed: %s\n", strerror(errno));
                close(lFd);
                lFd = INVALID_SOCKET;
            }
        }
        
        if (lFd != INVALID_SOCKET) {
            if (ServerData.port == 0) {
               /* port 0 specifies dynamic free port allocation
                * reuse serverAddr to store the actual address / port */
               sockStatus = getsockname(lFd, (struct sockaddr *) &serverAddr, &sFdAddSize);           
               if (sockStatus == SOCK_ERR) {
                  fprintf(stderr,"getsockname() call failed: %s\n", strerror(errno));
                  close(lFd);
                  lFd = INVALID_SOCKET;               
               } else { 
                   if(ServerData.serverInfoFile != NULL) {
                       FILE* fh;
                       
                       /* Open file in append mode to save info already stored in the file*/
                       fh = fopen(ServerData.serverInfoFile,"a"); 
                       if (fh == NULL) {
                           fprintf(stderr,"Unable to open output file to write server port number: %s\n", strerror(errno));
                           lFd = INVALID_SOCKET;
                       }
                       
                       (void)fprintf(fh, "Server Port Number: %u\n", ntohs(serverAddr.sin_port));
                       fclose(fh);
                   } else {
                       /* write the server port number to stdout */
                       SERVER_PORT_PRINTF("Server Port Number: %u\n", ntohs(serverAddr.sin_port));
                   }
               }                 
            }
        } 

        if (lFd != INVALID_SOCKET) {
           
            sockStatus = listen(lFd, 2);
            if (sockStatus == SOCK_ERR) {
                printf("listen() call failed.\n");
                close(lFd);
                lFd = INVALID_SOCKET;
            }
          
        }
        ServerData.listenSock = lFd;
      
    }

    if (lFd == INVALID_SOCKET) {
        streamID = RTIOSTREAM_ERROR;
    }
    return streamID;

}
/* Function: serverOpenConnection =================================================
 * Abstract:
 *  Called when the target is not currently connected to the host, this 
 *  function attempts to open the connection.  
 *
 *  In the case of sockets, this is a passive operation in that the host
 *  initiates contact, the target simply listens for connection requests.
 *
 * NOTES:
 
 * Blocks if Blocking == 1, poll for pending connections otherwise. When
 * polling, there may be no open requests pending.  In this case, this
 * function returns without making a connection; this is not an error.
 */
static SOCKET serverOpenConnection(void)
{
    struct sockaddr_in clientAddr;
    int     sFdAddSize     = (int) sizeof(struct sockaddr_in);
    SOCKET  cFd            = INVALID_SOCKET;
    SOCKET  lFd            = ServerData.listenSock;
    int error             = RTIOSTREAM_NO_ERROR;
    int pending            = 1;

    /* Check that the listening socket is still valid and open a new socket if
     * not */
    if (lFd == INVALID_SOCKET) {
        serverStreamOpen();
        lFd = ServerData.listenSock;
    }

    if (Blocking==0) {
        error = socketDataPending(lFd, &pending, 0);
    }
    
    
    if ( (pending > 0) && (error==RTIOSTREAM_NO_ERROR) ) {

        /*
         * Wait to accept a connection on the comm socket.
         */
        printf("Waiting to accept connection\n");
        cFd = accept(lFd, (struct sockaddr *)&clientAddr,
#if (!defined(_WIN32)) && (!defined(VXWORKS))
                     (socklen_t *)
#endif
                     &sFdAddSize);
        
        if (cFd == INVALID_SOCKET) {
            printf("accept() for comm socket failed.\n");
            error = RTIOSTREAM_ERROR;
        } 

        if (error == RTIOSTREAM_ERROR) {
            close(ServerData.listenSock);
            ServerData.listenSock = INVALID_SOCKET;
        } 
    }

    return cFd;
} 


/* Function: nameLookup =======================
 * Lookup target network name.
 */
#if (!defined(VXWORKS))
static struct hostent * nameLookup(char * hostName) {

    struct hostent * hp = NULL;

    /*
     * Default to localhost if hostname not specified.
     */
    if (hostName == NULL) {
        static char localhost[] = "localhost";
        hostName = localhost;
    }
    
    if (!isdigit(*hostName)) {
        /* Try gethostbyname first for speed*/
        hp = gethostbyname(hostName);
        if (hp == NULL) {
            unsigned long addr;
            
            addr = inet_addr(hostName);
            if ( ( (int) addr )==(-1) ) {
                printf("gethostbyname() and gethostbyaddr() calls failed.\n");
            } else {
                hp = gethostbyaddr((void*)&addr,sizeof(addr),AF_INET);
                if ( hp ==NULL ) {
                    printf("gethostbyname() and gethostbyaddr() calls failed.\n");
                }
            }
        }
    } else {
        /* Try gethostbyaddr first for speed*/
        unsigned long addr;
        
        if (( (int) (addr = inet_addr(hostName)) == -1) ) {
            printf("Failed to get internet address\n");
        } else {
            hp = gethostbyaddr((void*)&addr,sizeof(addr),AF_INET);
            if (hp==NULL) {
                hp = gethostbyname(hostName);
                if (hp == NULL) {
                    printf(
                        "gethostbyaddr() and gethostbyname() calls failed.\n");
                }
            }
        }
    }
    return hp;
}
#endif

/* Function: processArgs ====================================================
 * Abstract:
 *  Process the arguments specified by the user when opening the rtIOStream.
 *      
 *  If any unrecognized options are encountered, ignore them.
 *
 * Returns zero if successful or RTIOSTREAM_ERROR if 
 * an error occurred.
 *
 *  o IMPORTANT!!!
 *    As the arguments are processed, their strings should be NULL'd out in
 *    the argv array. 
 */
static int processArgs(
    const int      argc,
    void         * argv[],
    char        ** hostName, 
    unsigned int * portNum,
    unsigned int * isClient,
    unsigned int * isBlocking,
    unsigned int * recvTimeout)
{
    int        retVal    = RTIOSTREAM_NO_ERROR;
    int        count           = 0;

    while(count < argc) {
        const char *option = (char *)argv[count];
        count++;

        if (option != NULL) {

            if ((strcmp(option, "-hostname") == 0) && (count != argc)) {

                *hostName = (char *)argv[count];
                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;

            } else if ((strcmp(option, "-port") == 0) && (count != argc)) {
                char       tmpstr[2];
                int itemsConverted;
                const char *portStr = (char *)argv[count];

                count++;     
                
                itemsConverted = sscanf(portStr,"%d%1s", (int *) portNum, tmpstr);
                if ( (itemsConverted != 1) || 
                     ( ((*portNum != 0) && (*portNum < 255)) || (*portNum > 65535)) 
                    ) {
                    
                    retVal = RTIOSTREAM_ERROR;
                } else {

                    argv[count-2] = NULL;
                    argv[count-1] = NULL;
                }           
                
            } else if ((strcmp(option, "-client") == 0) && (count != argc)) {
                
                *isClient = ( strcmp( (char *)argv[count], "1") == 0 );

                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;

            } else if ((strcmp(option, "-blocking") == 0) && (count != argc)) {
                
                *isBlocking = ( strcmp( (char *)argv[count], "1") == 0 );

                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;

            } else if ((strcmp(option, "-recv_timeout_secs") == 0) && (count != argc)) {
                char       tmpstr[2];
                int itemsConverted;
                const char *timeoutSecsStr = (char *)argv[count];

                count++;     
                
                itemsConverted = sscanf(timeoutSecsStr,"%d%1s", (int *) recvTimeout, tmpstr);
                if ( itemsConverted != 1 ) {
                    retVal = RTIOSTREAM_ERROR;
                } else {

                    argv[count-2] = NULL;
                    argv[count-1] = NULL;
                }           

            } else if((strcmp(option, "-server_info_file") == 0) && (count != argc)) {
                ServerData.serverInfoFile= (char *) argv[count];
                
                count++;
                argv[count-2] = NULL;
                argv[count-1] = NULL;
            } else{
                /* do nothing */
            }
        }
    }
    return retVal;
}

/* Function: clientStreamOpen =================================================
 * Abstract:
 *  Open a connection as Client
 */
#if (!defined(VXWORKS))
static SOCKET clientStreamOpen(char * hostName, unsigned int portNum) {
    
    struct sockaddr_in sa;
    struct hostent     *hp;
    int errStatus = RTIOSTREAM_NO_ERROR;
    SOCKET cSock = INVALID_SOCKET;
    
    hp = nameLookup(hostName);
    if (hp==NULL) {
        errStatus = RTIOSTREAM_ERROR;
    }

    if (errStatus!=RTIOSTREAM_ERROR) {
        
        memcpy((char *)&sa.sin_addr,(char *)hp->h_addr,hp->h_length);
        sa.sin_family = hp->h_addrtype;
        sa.sin_port   = htons((unsigned short) portNum);

        /*
         * Create the sockets & make connections.
         */
        cSock = socket(PF_INET,SOCK_STREAM,0);
        if (cSock == INVALID_SOCKET) {
            errStatus = RTIOSTREAM_ERROR;
            printf("socket() call failed for comm socket.\n");
        }
    }

    if (errStatus!=RTIOSTREAM_ERROR) {
        if (connect(cSock, (struct sockaddr *)&sa, sizeof(sa)) == SOCK_ERR) {
            char tmp[1024];

            sprintf(tmp,
                    "Unable to establish connection with TCP/IP server '%s' "
                    ". Verify that your application is serving on port '%d'.\n", 
                    hp->h_name,
                    ntohs(sa.sin_port));
            cSock = INVALID_SOCKET;
            printf("%s",tmp);
        } 
    }

    return cSock;
}
#endif

/***************** VISIBLE FUNCTIONS ******************************************/

/* Function: rtIOStreamOpen =================================================
 * Abstract:
 *  Open the connection with the target.
 */
int rtIOStreamOpen(int argc, void * argv[])
{
    char               *xHostName = NULL;
    unsigned int        xPortNum     = (SERVER_PORT_NUM);
    unsigned int        isClient = 0;
    int result = RTIOSTREAM_NO_ERROR;
    int streamID = RTIOSTREAM_ERROR;

    printf("rtIOStreamOpen\n");
    Blocking = EXT_BLOCKING; /* Set default value */

    processArgs(argc, argv, &xHostName, &xPortNum, &isClient, &Blocking,
        &RecvTimeoutSecs);    

#ifdef _WIN32
    {
        WSADATA data;
        if (WSAStartup((MAKEWORD(1,1)), &data)) {
            result = RTIOSTREAM_ERROR;
            printf("WSAStartup() call failed.\n");
        }
    }
#endif
    
    if (result != RTIOSTREAM_ERROR) {
        if (isClient == 1) {

#if (!defined(VXWORKS)) && (!defined(QNX_OS)) /* Client side connection not supported on VxWorks */
            SOCKET sock = clientStreamOpen(xHostName, xPortNum);
            /* On Windows platform SOCKET is unsigned type, on Unix it is
             * signed. Tested with LCC and MSVC compilers that this conversion
             * from unsigned to signed then back to signed does restore the
             * original value.
             */ 
            streamID = (int) sock;
#endif
        } else {
            ServerData.port = xPortNum;
            ServerData.sock = INVALID_SOCKET;
            streamID = serverStreamOpen();
        }
    }
    printf("Returning from rtIOStreamOpen %d \n", streamID);
    return streamID;
}

/* Function: rtIOStreamSend =====================================================
 * Abstract:
 *  Sends the specified number of bytes on the comm line. Returns the number of
 *  bytes sent (if successful) or a negative value if an error occurred. As long
 *  as an error does not occur, this function is guaranteed to set the requested
 *  number of bytes; the function blocks if tcpip's send buffer doesn't have
 *  room for all of the data to be sent
 */
int rtIOStreamSend(
    int streamID,
    const void *src,
    size_t size,
    size_t *sizeSent)
{
    int retVal;

    if (streamID == SERVER_STREAM_ID) {
        if (ServerData.sock == INVALID_SOCKET) {

            ServerData.sock = serverOpenConnection();

        }
    
#ifdef QNX_OS
        if (sem_wait(commSem) == 0) {
           retVal = socketDataSet(ServerData.sock, src, size, sizeSent);
        }
        else {
            printf("@@@@@@@@@@@Error commSem not decremented@@@@@@@@@@\n");
        }
           
        sem_post(commSem);
#else
        semTake(commSem, WAIT_FOREVER);
        
        /*
         * VXWORKS send prototype does not have src as const.  This suppresses
         * the compiler warning.
         */

        retVal = socketDataSet(ServerData.sock, (char *)src, size, sizeSent);
        
        semGive(commSem);
#endif

    } else { /* Client stream */

        SOCKET sock = (SOCKET) streamID;
        retVal = socketDataSet(sock, src, size, sizeSent);

    }
    return retVal;
}


/* Function: rtIOStreamRecv ================================================
 * Abstract: receive data
 *
 */
int rtIOStreamRecv(
    int      streamID,
    void   * dst, 
    size_t   size,
    size_t * sizeRecvd) 
{
    int retVal = RTIOSTREAM_NO_ERROR;

    if (streamID == SERVER_STREAM_ID) {

        retVal = serverStreamRecv(dst, size, sizeRecvd); 

    } else { /* Client stream */

        int pending = 1;
        SOCKET cSock = (SOCKET) streamID;

        {
            unsigned int timeout=0;
            if (Blocking == 0) {
                timeout = 0;
            } else {
                timeout = RecvTimeoutSecs;
            }
            retVal = socketDataPending(cSock,&pending,timeout);
        }

        if (pending==0) {
            *sizeRecvd = 0U;
        } else {
            retVal = socketDataGet(cSock, (char *)dst, size, sizeRecvd);
        }
    }

    return retVal;
}


/* Function: rtIOStreamClose ================================================
 * Abstract: close the connection.
 *
 */
int rtIOStreamClose(int streamID)
{
    int retVal = RTIOSTREAM_NO_ERROR;
    
    if (streamID == SERVER_STREAM_ID) {
        char * tmpBuf[TMP_BUF_SIZ];
        int numRecvd;
        numRecvd = recv( ServerData.sock, (void *) tmpBuf, TMP_BUF_SIZ, 0);
        while (numRecvd > 0) {
            numRecvd = recv( ServerData.sock, (void *) tmpBuf, TMP_BUF_SIZ, 0);
        }
        close(ServerData.sock);
        ServerData.sock = INVALID_SOCKET;

        close(ServerData.listenSock);

        ServerData.listenSock = INVALID_SOCKET;

    } else {
        SOCKET cSock = (SOCKET) streamID;
        close(cSock);
        
    }

    return retVal;
}


