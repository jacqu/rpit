/*
 *	rpitdd.h: common definitions for rpitdd_client.c and rpitdd_server.c
 * 
 */

#define RPITDD_ACK_TIMEOUT_MS			10000
#define RPITDD_CHAR_BUF_LEN				255

#define RPITDD_CMD_NOCMD					0
#define RPITDD_CMD_EXIT						1
#define RPITDD_CMD_START					2
#define RPITDD_CMD_STOP						3

/* Polaris specific */

#define RPITDD_POL_STATUS_OK			0			// Pose OK
#define RPITDD_POL_STATUS_PART		1			// Target partially out of volume
#define RPITDD_POL_STATUS_OUT			2			// Target completely out of volume
#define RPITDD_POL_STATUS_INV			3			// Pose invalid

struct rpitdd_data					//	Server		Client
{
	unsigned char cmd;				//	R/W				R/W
	unsigned char	started;		//	R/W				R
	
	/* Polaris specific */
	
	// Translation
	double 				x;					//	W					R
	double 				y;					//	W					R
	double 				z;
	
	// Quaternions
	double 				q0;					//	W					R
	double 				qx;					//	W					R
	double 				qy;					//	W					R
	double 				qz;					//	W					R
	
	// Status
	unsigned char	status;			//	W					R
	
	// Frame number
	unsigned long frame;			//	W					R
};
