/*
 * rpit_socket_server : server on distant PC answering requests from
 * 											RPIt socket block.
 * 
 * Compile with : 
 * Linux : gcc -Wall -o rpit_socket_server -lpthread -lrt rpit_socket_server.c
 * OSX   : gcc -Wall -o rpit_socket_server -lpthread rpit_socket_server.c
 * 
 * JG, July 15 2016.
 */

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

/* 
 * 
 * 
 * Insert measurements definitions code here 
 * 
 * 
 * 
 */








/**********************************************/
	

/* Check that these definitions are identical in client code */

#define RPIT_SOCKET_CON_N					10			// Nb of double sent (control)
#define RPIT_SOCKET_MES_N					10			// Nb of double returned (measurement)
#define RPIT_SOCKET_PORT					"31415"	// Port of the server
#define RPIT_SOCKET_MES_PERIOD		2000		// Sampling period of the measurement (us)
#define RPIT_SOCKET_MAGIC					3141592	// Magic number
#define RPIT_SOCKET_WATCHDOG_TRIG	1000000	// Delay in us before watchdog is triggered

struct RPIt_socket_mes_struct	{
	unsigned int				magic;							// Magic number
	unsigned long long 	timestamp;					// Absolute server time in ns 
	double							mes[RPIT_SOCKET_MES_N];	// Measurements
};

struct RPIt_socket_con_struct	{
	unsigned int				magic;							// Magic number
	unsigned long long 	timestamp;					// Absolute client time in ns
	double							con[RPIT_SOCKET_CON_N];	// Control signals
};

pthread_t 										mes_thread;
pthread_mutex_t 							mes_mutex;
struct RPIt_socket_mes_struct	mes;
struct RPIt_socket_con_struct	con;
unsigned char									exit_req = 0;

/*
 *	rpit_socket_get_time : get current absolute time in ns
 */
void rpit_socket_get_time( struct timespec *ts )	{

	if ( !ts )
		return;
  
	#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	
	host_get_clock_service( mach_host_self(), CALENDAR_CLOCK, &cclock );
	clock_get_time( cclock, &mts );
	mach_port_deallocate( mach_task_self(), cclock );
	ts->tv_sec = mts.tv_sec;
	ts->tv_nsec = mts.tv_nsec;

	#else
	clock_gettime( CLOCK_MONOTONIC, ts );
	#endif
}

/*
 *	Measurement thread. Runs asynchronously at a higher rate.
 * 	It is possible to implement signal filtering here.
 */
void *rpit_socket_server_update( void *ptr )	{
	struct timespec 		current_time, last_time;
	unsigned long long	period;
	int									i;
	unsigned long long	watchdog_counter = 0;
	unsigned long long	last_timestamp = 0;
	
	
	rpit_socket_get_time( &last_time );
	mes.magic = RPIT_SOCKET_MAGIC;
	
	while( 1 )	{
		
		/* Check if exit is requested */
		
		if ( exit_req )
			break;
			
		/* Sleep to synchronize acquisition (adapt to your case) */
	
		usleep( RPIT_SOCKET_MES_PERIOD );
		
		/* Get current time */
		
		rpit_socket_get_time( &current_time );
		
		/* Critical section */
		
		pthread_mutex_lock( &mes_mutex );	
		
		mes.timestamp = (unsigned long long)current_time.tv_sec * 1000000000
									+ (unsigned long long)current_time.tv_nsec;
		
		/* 
		 * 
		 * 
		 * Insert the measurements acquisition code here.
		 * This code can used control signals safely:
		 * they are protected by a mutex.
		 * 
		 * 
		 * 
		 */
		
		
		
		
		
		
		
		
		/**********************************************/
		
		/* Fake measurements for test only (mes = con). Comment this out. */
		
		for( i = 0; ( i < RPIT_SOCKET_MES_N ) || ( i < RPIT_SOCKET_CON_N ); i++ )
			mes.mes[i] = con.con[i];
		
		/* Whatchdog: if control signals are not updated, force them to 0 */
		
		if ( last_timestamp == con.timestamp )
			watchdog_counter++;
		else
			watchdog_counter = 0;
		
		last_timestamp = con.timestamp;
		
		if ( watchdog_counter >= ( RPIT_SOCKET_WATCHDOG_TRIG / RPIT_SOCKET_MES_PERIOD ) )	{
			
			flockfile( stderr );
			fprintf( stderr, "rpit_socket_server_update: watchdog triggered (%ds).\n",
												(int)( ( watchdog_counter * RPIT_SOCKET_MES_PERIOD ) / 1000000 ) );
			funlockfile( stderr );
			
			for( i = 0; i < RPIT_SOCKET_CON_N; i++ )
				con.con[i] = 0.0;
		}
	
		pthread_mutex_unlock( &mes_mutex );	
		
		/* Display period */
		
		period = mes.timestamp - ( (unsigned long long)last_time.tv_sec * 1000000000
														 + (unsigned long long)last_time.tv_nsec );
		last_time = current_time;
		
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_server_update: iteration period = %llu us.\n", period / 1000 );
		funlockfile( stderr );
	}
	
	return NULL;
}

/*
 *	SIGINT handler
 */
void rpit_socket_server_int_handler( int dummy )	{
	
	/* Request termination of the thread */
	
	exit_req = 1;
	
	/* Wait for thread to terminate */
	
	pthread_join( mes_thread, NULL );
	
	/* 
	 * 
	 * 
	 * Insert measurements cleanup code here.
	 * 
	 * 
	 * 
	 */
	
	
	
	
	
	
	
	
	/**********************************************/
	
	/* Cleanup */
	
	flockfile( stderr );
	fprintf( stderr, "\nrpit_socket_server_int_handler: measurement thread stopped. Cleaning up...\n" );
	funlockfile( stderr );
	
	/* Exit */
	
	exit( EXIT_SUCCESS );
}

/*
 *	Main.
 */
int main( void )	{
	
	struct addrinfo 							hints;
	struct addrinfo 							*result, *rp;
	int 													sfd, s, i;
	struct sockaddr_storage 			peer_addr;
	socklen_t 										peer_addr_len;
	ssize_t 											nread;
	struct RPIt_socket_mes_struct	local_mes;
	struct RPIt_socket_con_struct	local_con;
	
	/* 
	 * 
	 * 
	 * Insert measurements initialization code here 
	 * 
	 * 
	 * 
	 */
	
	
	
	
	
	
	
	
	/**********************************************/
	
	
	/* Initialize mutex */
	
	pthread_mutex_init( &mes_mutex, NULL );
	
	/* Clear mes structure */
	
	mes.timestamp = 0;
	for ( i = 0; i < RPIT_SOCKET_MES_N; i++ )
		mes.mes[i] = 0.0;
	
	/* Clear con structure */
	
	con.magic = 0;
	con.timestamp = 0;
	for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
		con.con[i] = 0.0;
	
	/* Initialize SIGINT handler */
	
	signal( SIGINT, rpit_socket_server_int_handler );
	
	memset( &hints, 0, sizeof( struct addrinfo ) );
	hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
	hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
	hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
	hints.ai_protocol = 0;					/* Any protocol */
	hints.ai_canonname = NULL;
	hints.ai_addr = NULL;
	hints.ai_next = NULL;

	s = getaddrinfo( NULL, RPIT_SOCKET_PORT, &hints, &result );
	
	if ( s != 0 ) {
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_server: function getaddrinfo returned: %s\n", gai_strerror( s ) );
		funlockfile( stderr );
		exit( EXIT_FAILURE );
	 }
	 
	/* 	
		getaddrinfo() returns a list of address structures.
		Try each address until we successfully bind(2).
		If socket(2) (or bind(2)) fails, we (close the socket
		and) try the next address. 
	*/

	for ( rp = result; rp != NULL; rp = rp->ai_next ) {
		sfd = socket( rp->ai_family, rp->ai_socktype, rp->ai_protocol );
		if ( sfd == -1 )
			continue;

		if ( bind( sfd, rp->ai_addr, rp->ai_addrlen ) == 0 )
			break;									/* Success */

		close( sfd );
	}

	if ( rp == NULL ) {					/* No address succeeded */
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_server: could not bind. Aborting.\n" );
		funlockfile( stderr );
		exit( EXIT_FAILURE );
	}

	freeaddrinfo( result );			/* No longer needed */ 
	
	/* Start measurement thread */
	
	pthread_create( &mes_thread, NULL, rpit_socket_server_update, (void*) NULL );
	
	/* Wait for control datagram and answer measurement to sender */

	while ( 1 ) {
		
		/* Read control signals from the socket */
		
		peer_addr_len = sizeof( struct sockaddr_storage );
		nread = recvfrom(	sfd, (char*)&local_con, sizeof( struct RPIt_socket_con_struct ), 0,
											(struct sockaddr *)&peer_addr, &peer_addr_len );
		
		/* Memcopy is faster than socket read: avoid holding the mutex too long */
		
		pthread_mutex_lock( &mes_mutex );
		
		memcpy( &con, &local_con, sizeof( struct RPIt_socket_con_struct ) );
		
		if ( nread == -1 )	{
			flockfile( stderr );
			fprintf( stderr, "rpit_socket_server: function recvfrom exited with error.\n" );
			funlockfile( stderr );
			
			/* Clear control in case of error */
			
			for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
				con.con[i] = 0.0;
		}
		
		if ( nread != sizeof( struct RPIt_socket_con_struct ) )	{
			flockfile( stderr );
			fprintf( stderr, "rpit_socket_server: function recvfrom did not receive the expected packet size.\n" );
			funlockfile( stderr );
			
			/* Clear control in case of error */
			
			for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
				con.con[i] = 0.0;
		}
										
		if ( con.magic != RPIT_SOCKET_MAGIC )	{
			flockfile( stderr );
			fprintf( stderr, "rpit_socket_server: magic number problem. Expected %d but received %d.\n", RPIT_SOCKET_MAGIC, con.magic );
			funlockfile( stderr );
			
			/* Clear control in case of error */
			
			for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
				con.con[i] = 0.0;
		}
		
		pthread_mutex_unlock( &mes_mutex );
		
		/*
		 * 
		 * 
		 *	Insert here the handling of control signals.
		 * 
		 * 
		 * 
		 */
		 
		 
		 
		 
		 
		 
		 
		 
		 
		/**********************************************/
		
		/* Critical section : copy of the measurements to a local variable */
		
		pthread_mutex_lock( &mes_mutex );
		memcpy( &local_mes, &mes, sizeof( struct RPIt_socket_mes_struct ) );
		pthread_mutex_unlock( &mes_mutex );	
		
		/* Send measurements to the socket */
		
		if ( sendto(	sfd, (char*)&local_mes, sizeof( struct RPIt_socket_mes_struct ), 0,
									(struct sockaddr *)&peer_addr,
									peer_addr_len) != sizeof( struct RPIt_socket_mes_struct ) )	{
			flockfile( stderr );
			fprintf( stderr, "rpit_socket_server: error sending measurements.\n" );
			funlockfile( stderr );
		}		
	}
		
	exit( EXIT_SUCCESS );
}
