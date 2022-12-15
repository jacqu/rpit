/*
 * rpit_socket_client : client of distant PC answering requests from
 * 											RPIt socket block.
 * 
 * Compile with :
 *   Linux : gcc -Wall -o rpit_socket_client -lpthread -lrt rpit_socket_client.c
 *   OSX   : gcc -Wall -o rpit_socket_client -lpthread rpit_socket_client.c
 * 
 * JG, July 16 2016.
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
#include "rpit_socket_client.h"

//#define RPIT_SOCKET_DISPLAY_MES
//#define RPIT_SOCKET_API

/* Check that these definitions are identical in server code */

#define RPIT_SOCKET_CON_N					150			// Nb of double sent (control)
#define RPIT_SOCKET_MES_N					150			// Nb of double returned (measurement)
#define RPIT_SOCKET_TIMEOUT				1000000	// Server answering timeout in us
#define RPIT_SOCKET_SERVER_START	1000000	// Server startup time in us
#define RPIT_SIZEOF_IP            20      // Size of an IP address
#define RPIT_SIZEOF_PORT          10      // Size of a Port number
#define RPIT_SOCKET_MAGIC					3141592	// Magic number
#define RPIT_SOCKET_MAX_INSTANCES	10			// Max number of client instances 

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

struct RPIt_socket_instance_struct	{
	unsigned char									exit_req;	// Termination request
	unsigned char									ip1;			// IP address of the server
	unsigned char									ip2;
	unsigned char									ip3;
	unsigned char									ip4;
  unsigned int                  port;
	int														sfd;			// Socket file descriptor
	pthread_t											update_thread;
	pthread_mutex_t 							update_mutex;
	struct RPIt_socket_mes_struct	mes;			// Measurements
	struct RPIt_socket_con_struct	con;			// Control signals
};



struct RPIt_socket_instance_struct				instances[RPIT_SOCKET_MAX_INSTANCES] = { { 0 } };
unsigned int															nb_instances = 0;

/*
 *	rpit_socket_client_ip2id : converts an ip:port address to instance ID
 */
int rpit_socket_client_ip2id( 	unsigned char ip1, 
																unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                unsigned int  port
                                                                )	{
	int	i;
	
	/* Scan the instance array looking for the given IP:port address */
	
	for ( i = 0; i < RPIT_SOCKET_MAX_INSTANCES; i++ )	{
		if (	( ip1 == instances[i].ip1 ) &&
					(	ip2 == instances[i].ip2 ) &&
					(	ip3 == instances[i].ip3 ) &&
					(	ip4 == instances[i].ip4 ) && 
          ( port == instances[i].port ) )
			return i;
	}
	
	/* IP:port address was not found */
	
	flockfile( stderr );
	fprintf( stderr, "rpit_socket_client_ip2id: %d.%d.%d.%d:%d uninitialized.\n", ip1, ip2, ip3, ip4, port );
	funlockfile( stderr );
	
	return -1;
}

/* 
 * rpit_socket_client_update : thread updating measurements from server asynchronously
 */
void* rpit_socket_client_update( void* prt )	{
	struct RPIt_socket_instance_struct* instance = (struct RPIt_socket_instance_struct*)prt;
	ssize_t                   		nread;
	struct timespec           		before_time, after_time;
  struct RPIt_socket_mes_struct	local_mes;
  struct RPIt_socket_con_struct	local_con;
	#ifdef RPIT_SOCKET_DISPLAY_MES
	int                       		i;
	unsigned long long        		period;
	#endif
  
	while ( 1 )	{
		
		/* Check if instance pointer is consistent */
		
		if ( !instance )
			continue;
			
		/* Check if exit is requested */
		
		if ( instance->exit_req )
			break;
		
		/* Check if socket exists */
		
		if ( !instance->sfd )
			continue;
		
		/* Get time before sending request */
		
		clock_gettime( CLOCK_MONOTONIC, &before_time );
		
		/* Update control signals */
		
		pthread_mutex_lock( &instance->update_mutex );
		memcpy( &local_con, &instance->con, sizeof( struct RPIt_socket_con_struct ) );
		pthread_mutex_unlock( &instance->update_mutex );
		
		/* Update timestamp */
		
		local_con.timestamp = (unsigned long long)before_time.tv_sec * 1000000000
												+ (unsigned long long)before_time.tv_nsec;
		
		/* 
			Send control packet and read measurement packet from server.
		*/
		
		if (	write(	instance->sfd, 
									(char*)&local_con, 
									sizeof( struct RPIt_socket_con_struct ) ) != 
					sizeof( struct RPIt_socket_con_struct ) )	{
			flockfile( stderr );
			fprintf( stderr, "rpit_socket_client_update: partial/failed write on %d.%d.%d.%d:%d.\n",
																												instance->ip1,
																												instance->ip2,
																												instance->ip3,
																												instance->ip4,
                                                        instance->port);
			funlockfile( stderr );
		}
		
		/* 
			Read measurement packet from server.
		*/
		
		nread = read( instance->sfd, (char*)&local_mes, sizeof( struct RPIt_socket_mes_struct ) );
		
		/* Get time after receiving response */
		
		clock_gettime( CLOCK_MONOTONIC, &after_time );
		
		/* Compute read duration = sampling period */
		#ifdef RPIT_SOCKET_DISPLAY_MES
		period = (unsigned long long)after_time.tv_sec * 1000000000
					 + (unsigned long long)after_time.tv_nsec
				 - ( (unsigned long long)before_time.tv_sec * 1000000000
					 + (unsigned long long)before_time.tv_nsec );
		#endif
		
		/* Check response */
		
		if ( nread == -1 ) {
			flockfile( stderr );
			fprintf( stderr, "(%d.%d.%d.%d:%d)\t",
												instance->ip1,
												instance->ip2,
												instance->ip3,
												instance->ip4,
                        instance->port );
			perror( "rpit_socket_client_update" );
			funlockfile( stderr );
			
		}
		else
		{
			if ( nread != sizeof( struct RPIt_socket_mes_struct ) )	{
				flockfile( stderr );
				fprintf( stderr, 
				"rpit_socket_client_update: received %zd bytes from %d.%d.%d.%d:%d instead of %zd.\n", 
																							nread, 
																							instance->ip1,
																							instance->ip2,
																							instance->ip3,
																							instance->ip4,
                                              instance->port,
																							sizeof( struct RPIt_socket_mes_struct ) );
				funlockfile( stderr );
			}
			else
			{
				if ( local_mes.magic != RPIT_SOCKET_MAGIC )	{
					flockfile( stderr );
					fprintf( stderr, 
				"rpit_socket_client_update: received bad magic number from %d.%d.%d.%d:%d\n",  
																							instance->ip1,
																							instance->ip2,
																							instance->ip3,
																							instance->ip4,
                                              instance->port );
					funlockfile( stderr );
				}
				else
				{
					/* Update mes */
					
					pthread_mutex_lock( &instance->update_mutex );
					memcpy( &instance->mes, &local_mes, sizeof( struct RPIt_socket_mes_struct ) );
					pthread_mutex_unlock( &instance->update_mutex );
					
					/* Display measurements */
					
					#ifdef RPIT_SOCKET_DISPLAY_MES
					flockfile( stderr );
					fprintf( stderr, "> Server IP: %d.%d.%d.%d:%d\n",
														instance->ip1,
														instance->ip2,
														instance->ip3,
														instance->ip4,
                            instance->port );
					fprintf( stderr, "> Timestamp: %llu\n", local_mes.timestamp );
					fprintf( stderr, "> Ping: %llu us\n", period / 1000 );
					for ( i = 0; i < RPIT_SOCKET_MES_N; i++ )
						fprintf( stderr, "> mes[%d] = %e\n", i, local_mes.mes[i] );
					funlockfile( stderr );
					#endif
				}
			}
		}
	}
	
	return NULL;
}

/*
 * rpit_socket_client_add : add new connection to a server
 */
void rpit_socket_client_add( 	unsigned char ip1, 
															unsigned char ip2, 
															unsigned char ip3, 
															unsigned char ip4,
                              unsigned int  port )	{
																
	struct addrinfo 			hints;
	struct addrinfo 			*result, *rp;
	int 									s;
	struct timeval 				tv;
  char                  ip[RPIT_SIZEOF_IP];
  char                  portStr[RPIT_SIZEOF_PORT];
  int										sfd;
  int										inst_id;
  
  /* Compute IP address */

  snprintf( ip, RPIT_SIZEOF_IP, "%d.%d.%d.%d", ip1, ip2, ip3, ip4 );
  snprintf( portStr, RPIT_SIZEOF_PORT, "%d", port );

	/* Obtain address(es) matching host/port */

	memset( &hints, 0, sizeof( struct addrinfo ) );
	hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
	hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
	hints.ai_flags = 0;
	hints.ai_protocol = 0;					/* Any protocol (should be UDP with SOCK_DGRAM) */

	s = getaddrinfo( ip, portStr, &hints, &result );
	if ( s != 0 ) {
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_client: function getaddrinfo returned: %s\n", gai_strerror( s ) );
		funlockfile( stderr );
		return;
	}
	
	/* 
		getaddrinfo() returns a list of address structures.
		Try each address until we successfully connect(2).
		If socket(2) (or connect(2)) fails, we (close the socket
		and) try the next address.
	*/

	for ( rp = result; rp != NULL; rp = rp->ai_next )	{
		sfd = socket( rp->ai_family, rp->ai_socktype, rp->ai_protocol );
		if ( sfd == -1 )
			continue;

		if ( connect( sfd, rp->ai_addr, rp->ai_addrlen ) != -1 )
			break;									/* Success */

		close( sfd );
	}

	if ( rp == NULL )	{					/* No address succeeded */
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_client: could not connect. Aborting.\n" );
		funlockfile( stderr );
		return;
	}

	freeaddrinfo( result );			/* No longer needed */
	
	/* Set socket options */
	
	tv.tv_sec =		RPIT_SOCKET_TIMEOUT / 1000000;
	tv.tv_usec = 	RPIT_SOCKET_TIMEOUT % 1000000;
	setsockopt( sfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval) );
	
	/* Add connection to instances structure */
	
	if ( nb_instances == RPIT_SOCKET_MAX_INSTANCES )	{
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_client: maximum socket instances reached. Aborting.\n" );
		funlockfile( stderr );
		close( sfd );
		return;
	}
	
	/* Get first free instance pointer */
	
	for ( inst_id = 0; inst_id < RPIT_SOCKET_MAX_INSTANCES; inst_id++ )
		if ( instances[inst_id].ip1 == 0 )
			break;
	
	if ( inst_id == RPIT_SOCKET_MAX_INSTANCES )	{
		flockfile( stderr );
		fprintf( stderr, "rpit_socket_client: no more free instance left. Inernal error.\n" );
		funlockfile( stderr );
		close( sfd );
		return;
	}		
		
	/* Reset structure data */
	
	memset( &instances[inst_id], 0, sizeof( struct RPIt_socket_instance_struct ) );
	
	/* Initialize connection data */
	
	instances[inst_id].ip1 = ip1;
	instances[inst_id].ip2 = ip2;
	instances[inst_id].ip3 = ip3;
	instances[inst_id].ip4 = ip4;
  instances[inst_id].port = port;
	instances[inst_id].sfd = sfd;
	
	/* Initialize magic number in control structure */
	
	instances[inst_id].con.magic = RPIT_SOCKET_MAGIC;
	
	/* Initialize mutex */
	
	pthread_mutex_init( &instances[inst_id].update_mutex, NULL );
	
	/* Start one thread for each server */
	
	pthread_create( &instances[inst_id].update_thread, 
									NULL, 
									rpit_socket_client_update, 
									(void*) &instances[inst_id] );
	
	/* Increment instance counter */
  
  nb_instances++;
  
  /* Wait so that the server periodic thread can update the measurement */
  
  usleep( RPIT_SOCKET_SERVER_START );
}
 
/* 
 * rpit_socket_client_close : close communication with the server
 */
void rpit_socket_client_close( 	unsigned char ip1, 
																unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                unsigned int port )	{
	
	int 						inst_id;
	
	/* Get instance id */
	
	inst_id = rpit_socket_client_ip2id( ip1, ip2, ip3, ip4, port );
	
	/* Check instance id */
	
	if ( ( inst_id < 0 ) || ( inst_id >= RPIT_SOCKET_MAX_INSTANCES ) )
		return;
	
	/* Request termination of the thread */
	
	instances[inst_id].exit_req = 1;

	/* Wait for thread to terminate */
	
	pthread_join( instances[inst_id].update_thread, NULL );
	
	/* Close socket */
	
	close( instances[inst_id].sfd );
	
	/* Reset structure data */
	
	memset( &instances[inst_id], 0, sizeof( struct RPIt_socket_instance_struct ) );
	
	/* Decrement instance counter */
	
	nb_instances--;
}

/* 
 * rpit_socket_client_write : write control signal to be sent to server
 */
void rpit_socket_client_write(	unsigned char ip1, 
																unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                unsigned int  port,
																const double* values )	{
	int 							inst_id, i;
	
	/* Get instance id */
	
	inst_id = rpit_socket_client_ip2id( ip1, ip2, ip3, ip4, port );
	
	/* Check instance id */
	
	if ( ( inst_id < 0 ) || ( inst_id >= RPIT_SOCKET_MAX_INSTANCES ) )
		return;
	
	/* Set current control signals */
	
	pthread_mutex_lock( &instances[inst_id].update_mutex );
	for ( i = 0; i < RPIT_SOCKET_CON_N; i++ )
		instances[inst_id].con.con[i] = values[i];
	pthread_mutex_unlock( &instances[inst_id].update_mutex );			
	
	return;
}

/* 
 * rpit_socket_client_read : read measurements sent by the server
 */
void rpit_socket_client_read(	unsigned char ip1, 
															unsigned char ip2, 
															unsigned char ip3, 
															unsigned char ip4,
                              unsigned int  port,
															double*       values )	{
	int inst_id, i;
	
	/* Get instance id */
	
	inst_id = rpit_socket_client_ip2id( ip1, ip2, ip3, ip4, port );
	
	/* Check instance id */
	
	if ( ( inst_id < 0 ) || ( inst_id >= RPIT_SOCKET_MAX_INSTANCES ) )	{
		for ( i = 0; i < RPIT_SOCKET_MES_N; i++ )
			values[i] = 0.0;
		return;
	}
	
	/* Get current measurements */
	
	pthread_mutex_lock( &instances[inst_id].update_mutex );
	for ( i = 0; i < RPIT_SOCKET_MES_N; i++ )
		values[i] = instances[inst_id].mes.mes[i];
	pthread_mutex_unlock( &instances[inst_id].update_mutex );
	
	return;
} 
 
#ifndef RPIT_SOCKET_API
 
#define RPIT_SOCKET_IP1					127, 0, 0, 1
#define RPIT_SOCKET_PORT1				31415
#define RPIT_SOCKET_IP2					127, 0, 0, 1
#define RPIT_SOCKET_PORT2				31416
#define RPIT_SOCKET_DISPLAY_N		10
#define RPIT_SOCKET_MAIN_PERIOD	10000
#define RPIT_SOCKET_MAIN_ITER		10

int main( void )	{
	
	int i, j;
	double write_val[RPIT_SOCKET_CON_N], read_val[RPIT_SOCKET_MES_N];
	
	/* Add connections */
	
	rpit_socket_client_add( RPIT_SOCKET_IP1, RPIT_SOCKET_PORT1 );
	rpit_socket_client_add( RPIT_SOCKET_IP2, RPIT_SOCKET_PORT2 );
	
	/* Check on the fly removing and adding connection */
	
	rpit_socket_client_close( RPIT_SOCKET_IP1, RPIT_SOCKET_PORT1 );
	rpit_socket_client_add( RPIT_SOCKET_IP1, RPIT_SOCKET_PORT1 );
	
	/* Print some iterations */
	
	for ( i = 0; i < RPIT_SOCKET_MAIN_ITER; i++ )	{
		
		usleep( RPIT_SOCKET_MAIN_PERIOD );
		
		/* Write iteration index on every control signals */
		
		for ( j = 0; j < RPIT_SOCKET_CON_N; j++ )
			write_val[j] = (double)i;
		
		/* Send control signals on socket 1 */
		
		rpit_socket_client_write( RPIT_SOCKET_IP1, RPIT_SOCKET_PORT1, write_val );
		
		/* Write iteration index x -1 on every control signals */
		
		for ( j = 0; j < RPIT_SOCKET_CON_N; j++ )
			write_val[j] = -(double)i;
		
		/* Send control signals on socket 2 */
		
		rpit_socket_client_write( RPIT_SOCKET_IP2, RPIT_SOCKET_PORT2, write_val );

		/* Read measurements on socket 1 */
		
		rpit_socket_client_read( RPIT_SOCKET_IP1, RPIT_SOCKET_PORT1, read_val );
		
		/* Print socket 1 measurements */
		
		flockfile( stderr );
		fprintf( stderr, "Socket 1 mes:" );
		for ( j = 0; j < RPIT_SOCKET_DISPLAY_N; j++ )
			fprintf( stderr, "\t%d", (int)read_val[j] );
		fprintf( stderr, "\n" );
		funlockfile( stderr );
			
		/* Read measurements on socket 2 */
		
		rpit_socket_client_read( RPIT_SOCKET_IP2, RPIT_SOCKET_PORT2, read_val );
		
		/* Print socket 2 measurements */
		
		flockfile( stderr );
		fprintf( stderr, "Socket 2 mes:" );
		for ( j = 0; j < RPIT_SOCKET_DISPLAY_N; j++ )
			fprintf( stderr, "\t%d", (int)read_val[j] );
		fprintf( stderr, "\n" );
		funlockfile( stderr );
		
	}
	
	/* Close connections */
	
	rpit_socket_client_close( RPIT_SOCKET_IP1, RPIT_SOCKET_PORT1 );
	rpit_socket_client_close( RPIT_SOCKET_IP2, RPIT_SOCKET_PORT2 );

return 0;
}

#endif

