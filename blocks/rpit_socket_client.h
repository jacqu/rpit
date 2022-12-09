/*
 * rpit_socket_client.h
 */

int rpit_socket_client_ip2id( 	unsigned char ip1, 
																unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                                                unsigned int port);
void rpit_socket_client_add( 		unsigned char ip1, 
                                                                unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                                                unsigned int port);
void rpit_socket_client_close( 	unsigned char ip1, 
																unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                                                unsigned int port);
void rpit_socket_client_write(	unsigned char ip1, 
																unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                                                unsigned int port,
																const double* values );
void rpit_socket_client_read(		unsigned char ip1, 
																unsigned char ip2, 
																unsigned char ip3, 
																unsigned char ip4,
                                                                unsigned int port,
																double* values );
