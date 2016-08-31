/*
 * rpit_socket_server : server on distant PC answering requests from
 * 											RPIt socket block.
 * 
 * Compile with : gcc -Wall -o rpit_socket_server -lpthread -lrt rpit_socket_server.c
 * 
 * JG, May 2 2016.
 */

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

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

#define RPIT_SOCKET_N							10			// Nb of double returned
#define RPIT_SOCKET_PORT					"31415"	// Port of the sever
#define RPIT_SOCKET_MAGIC					"RPIt"	// Magic string			
																					// Size of the packet buffer																		
#define RPIT_SOCKET_BUF_SIZE			sizeof( RPIT_SOCKET_MAGIC )
#define RPIT_SOCKET_PERIOD				2000		// Sampling rate of the measurement (us)

struct RPIt_socket_struct	{
	unsigned long long 	timestamp;					// Absolute server time in ns 
	double							mes[RPIT_SOCKET_N];	// Measurements
};

pthread_t 								mes_thread;
pthread_mutex_t 					mes_mutex;
struct RPIt_socket_struct	mes;
unsigned char							exit_req = 0;

/*
 *	Measurement thread. Runs asynchronously.
 */
void *measurement_thread( void *ptr )	{
	struct timespec 		current_time, last_time;
	unsigned long long	period;
	int									i;
	
	clock_gettime( CLOCK_MONOTONIC, &last_time );
	
	while( 1 )	{
		
		/* Check if exit is requested */
		
		if ( exit_req )
			break;
			
		/* Sleep to synchronize acquisition (adapt to your case) */
	
		usleep( RPIT_SOCKET_PERIOD );
		
		/* Get current time */
		
		clock_gettime( CLOCK_MONOTONIC, &current_time );
		
		/* 
		 * 
		 * 
		 * Insert the measurements acquisition code here 
		 * 
		 * 
		 * 
		 */
		
		
		
		
		
		
		
		
		/**********************************************/
		
		/* Critical section */
		
		pthread_mutex_lock( &mes_mutex );	
		
		mes.timestamp = (unsigned long long)current_time.tv_sec * 1000000000
									+ (unsigned long long)current_time.tv_nsec;
		
		for( i = 0; i < RPIT_SOCKET_N; i++ )
			mes.mes[i] = i;
			
		pthread_mutex_unlock( &mes_mutex );	
		
		/* Display period */
		
		period = mes.timestamp - ( (unsigned long long)last_time.tv_sec * 1000000000
														 + (unsigned long long)last_time.tv_nsec );
		last_time = current_time;
		
		fprintf( stderr, "measurement_thread iteration period = %llu us.\n", period / 1000 );
	}
	
	return NULL;
}

/*
 *	SIGINT handler
 */
void intHandler( int dummy )	{
	
	/* Request termination of the thread */
	
	exit_req = 1;
	
	/* Wait for thread to terminate */
	
	pthread_join( mes_thread, NULL );
	
	/* 
	 * 
	 * 
	 * Insert measurements cleanup code here 
	 * 
	 * 
	 * 
	 */
	
	
	
	
	
	
	
	
	/**********************************************/
	
	/* Cleanup */
	
	fprintf( stderr, "\nMeasurement thread stopped. Cleaning up...\n" );
	
	/* Exit */
	
	exit( EXIT_SUCCESS );
}

/*
 *	Main.
 */
int main( void )	{
	
	struct addrinfo 				hints;
	struct addrinfo 				*result, *rp;
	int 										sfd, s, i;
	struct sockaddr_storage peer_addr;
	socklen_t 							peer_addr_len;
	ssize_t 								nread;
	char 										buf[RPIT_SOCKET_BUF_SIZE];
	
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
	for ( i = 0; i < RPIT_SOCKET_N; i++ )
		mes.mes[i] = 0.0;
	
	/* Initialize SIGINT handler */
	
	signal( SIGINT, intHandler );
	
	memset( &hints, 0, sizeof( struct addrinfo ) );
	hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
	hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
	hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
	hints.ai_protocol = 0;          /* Any protocol */
	hints.ai_canonname = NULL;
	hints.ai_addr = NULL;
	hints.ai_next = NULL;

	s = getaddrinfo( NULL, RPIT_SOCKET_PORT, &hints, &result );
	
	if ( s != 0 ) {
		fprintf( stderr, "Function getaddrinfo returned: %s\n", gai_strerror( s ) );
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
		fprintf( stderr, "Could not bind. Aborting.\n" );
		exit( EXIT_FAILURE );
	}

	freeaddrinfo( result );			/* No longer needed */ 
	
	/* Start measurement thread */
	
	pthread_create( &mes_thread, NULL, measurement_thread, (void*) NULL );
	
	/* Wait for datagram and answer measurement to sender */

	while ( 1 ) {
		peer_addr_len = sizeof( struct sockaddr_storage );
		nread = recvfrom(	sfd, buf, RPIT_SOCKET_BUF_SIZE, 0,
											(struct sockaddr *)&peer_addr, &peer_addr_len );
		if ( nread == -1 )	{
			fprintf( stderr, "Function recvfrom exited with error.\n" );
			continue;							/* Ignore failed request */
		}
											
		if ( strncmp( buf, RPIT_SOCKET_MAGIC, sizeof( RPIT_SOCKET_MAGIC ) ) )
			printf( "Expected %s but received %s.\n", RPIT_SOCKET_MAGIC, buf );
		
		/* Critical section : transfer of the measurements */
		
		pthread_mutex_lock( &mes_mutex );
		
		if ( sendto(	sfd, (char*)&mes, sizeof( mes ), 0,
									(struct sockaddr *)&peer_addr,
									peer_addr_len) != sizeof( mes ) )
			fprintf( stderr, "Error sending response.\n" );
			
		pthread_mutex_unlock( &mes_mutex );	
			
	}
		
	exit( EXIT_SUCCESS );
}
