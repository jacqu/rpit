/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* External defines */

// Version
#define DXL_VERSION_MAJOR								0
#define DXL_VERSION_MINOR								6

// Special IDs
#define NOT_USED_ID         						0xFF		// 255
#define BROADCAST_ID        						0xFE    // 254
#define MAX_ID              						0xFC    // 252

// Instruction for DXL Protocol
#define INST_PING           						1
#define INST_READ           						2
#define INST_WRITE          						3
#define INST_REG_WRITE      						4
#define INST_ACTION         						5
#define INST_FACTORY_RESET  						6
#define INST_SYNC_WRITE     						131     // 0x83
#define INST_BULK_READ      						146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT         						8
#define INST_STATUS         						85      // 0x55
#define INST_SYNC_READ      						130     // 0x82
#define INST_BULK_WRITE     						147     // 0x93

// Communication Result
#define COMM_SUCCESS        						0       // tx or rx packet communication success
#define COMM_PORT_BUSY      						-1000   // Port is busy (in use)
#define COMM_TX_FAIL        						-1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        						-1002   // Failed get status packet
#define COMM_TX_ERROR       						-2000   // Incorrect instruction packet
#define COMM_RX_WAITING     						-3000   // Now recieving status packet
#define COMM_RX_TIMEOUT     						-3001   // There is no status packet
#define COMM_RX_CORRUPT     						-3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  						-9000

/*
 * 
 * port_handler_linux.h
 * 
 * Low-level access functions to Linux serial port.
 * 
 */

int 		portName2portNumLinux			(const char *port_name);
int 		portHandlerLinux        	(const char *port_name);

uint8_t setupPortLinux          	(int port_num, const int cflag_baud);
uint8_t setCustomBaudrateLinux  	(int port_num, int speed);
int     getCFlagBaud            	(const int baudrate);

double  getCurrentTimeLinux     	( void );
double  getTimeSinceStartLinux  	(int port_num);

uint8_t openPortLinux           	(int port_num);
void    closePortLinux          	(int port_num);
void    clearPortLinux          	(int port_num);

void    setPortNameLinux        	(int port_num, const char *port_name);
char   *getPortNameLinux        	(int port_num);

uint8_t setBaudRateLinux        	(int port_num, const int baudrate);
int     getBaudRateLinux        	(int port_num);

int     getBytesAvailableLinux  	(int port_num);

int     readPortLinux           	(int port_num, uint8_t *packet, int length);
int     writePortLinux          	(int port_num, uint8_t *packet, int length);

void    setPacketTimeoutLinux     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecLinux (int port_num, double msec);
uint8_t isPacketTimeoutLinux      (int port_num);

/*
 * 
 * port_handler.h
 * 
 * Low-level access functions to Linux serial port.
 * 
 */

int     portName2portNum    			(const char *port_name);
int     portHandler             	(const char *port_name);

uint8_t openPort                	(int port_num);
void    closePort               	(int port_num);
void    clearPort               	(int port_num);

void    setPortName             	(int port_num, const char* port_name);
char   *getPortName             	(int port_num);

uint8_t setBaudRate             	(int port_num, const int baudrate);
int     getBaudRate             	(int port_num);

int     getBytesAvailable       	(int port_num);

int     readPort                	(int port_num, uint8_t *packet, int length);
int     writePort               	(int port_num, uint8_t *packet, int length);

void    setPacketTimeout        	(int port_num, uint16_t packet_length);
void    setPacketTimeoutMSec    	(int port_num, double msec);
uint8_t isPacketTimeout         	(int port_num);

/*
 * 
 * packet_handler.h
 * 
 * Packet handling funtions.
 * 
 */

void        packetHandler       	( void );

void        printTxRxResult     	(int protocol_version, int result);
void        printRxPacketError  	(int protocol_version, uint8_t error);

int         getLastTxRxResult   	(int port_num, int protocol_version);
uint8_t     getLastRxPacketError  (int port_num, int protocol_version);

void        setDataWrite        	(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos, uint32_t data);
uint32_t    getDataRead         	(int port_num, int protocol_version, uint16_t data_length, uint16_t data_pos);

void        txPacket            	(int port_num, int protocol_version);

void        rxPacket            	(int port_num, int protocol_version);

void        txRxPacket          	(int port_num, int protocol_version);

void        ping                	(int port_num, int protocol_version, uint8_t id);

uint16_t    pingGetModelNum     	(int port_num, int protocol_version, uint8_t id);

// broadcastPing
void        broadcastPing       	(int port_num, int protocol_version);
uint8_t     getBroadcastPingResult  (int port_num, int protocol_version, int id);

void        reboot              	(int port_num, int protocol_version, uint8_t id);

void        factoryReset        	(int port_num, int protocol_version, uint8_t id, uint8_t option);

void        readTx              	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
void        readRx              	(int port_num, int protocol_version, uint16_t length);
void        readTxRx            	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

void        read1ByteTx         	(int port_num, int protocol_version, uint8_t id, uint16_t address);
uint8_t     read1ByteRx         	(int port_num, int protocol_version);
uint8_t     read1ByteTxRx       	(int port_num, int protocol_version, uint8_t id, uint16_t address);

void        read2ByteTx         	(int port_num, int protocol_version, uint8_t id, uint16_t address);
uint16_t    read2ByteRx         	(int port_num, int protocol_version);
uint16_t    read2ByteTxRx       	(int port_num, int protocol_version, uint8_t id, uint16_t address);

void        read4ByteTx         	(int port_num, int protocol_version, uint8_t id, uint16_t address);
uint32_t    read4ByteRx         	(int port_num, int protocol_version);
uint32_t    read4ByteTxRx       	(int port_num, int protocol_version, uint8_t id, uint16_t address);
uint8_t* 		readNByteTxRx					(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t length);

void    writeTxOnly             	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
void    writeTxRx               	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

void    write1ByteTxOnly        	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data);
void    write1ByteTxRx          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint8_t data);

void    write2ByteTxOnly        	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data);
void    write2ByteTxRx          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t data);

void    write4ByteTxOnly        	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data);
void    write4ByteTxRx          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint32_t data);

void    regWriteTxOnly          	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);
void    regWriteTxRx            	(int port_num, int protocol_version, uint8_t id, uint16_t address, uint16_t length);

void    syncReadTx              	(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

void    syncWriteTxOnly         	(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length, uint16_t param_length);

void    bulkReadTx              	(int port_num, int protocol_version, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

void    bulkWriteTxOnly         	(int port_num, int protocol_version, uint16_t param_length);

/*
 * 
 * protocol1_packet_handler.h
 * 
 * Protocol 1 packet handling funtions.
 * 
 */

void        printTxRxResult1    	(int result);
void        printRxPacketError1 	(uint8_t error);

int         getLastTxRxResult1  	(int port_num);
uint8_t     getLastRxPacketError1 (int port_num);

void        setDataWrite1       	(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data);
uint32_t    getDataRead1        	(int port_num, uint16_t data_length, uint16_t data_pos);

void        txPacket1           	(int port_num);
void        rxPacket1           	(int port_num);
void        txRxPacket1         	(int port_num);

void        ping1               	(int port_num, uint8_t id);
uint16_t    pingGetModelNum1    	(int port_num, uint8_t id);

// broadcastPing
void        broadcastPing1      	(int port_num);
uint8_t     getBroadcastPingResult1 (int port_num, int id);

void        action1             	(int port_num, uint8_t id);
void        reboot1             	(int port_num, uint8_t id);
void        factoryReset1       	(int port_num, uint8_t id, uint8_t option);

void        readTx1             	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        readRx1             	(int port_num, uint16_t length);
void        readTxRx1           	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        read1ByteTx1        	(int port_num, uint8_t id, uint16_t address);
uint8_t     read1ByteRx1        	(int port_num);
uint8_t     read1ByteTxRx1      	(int port_num, uint8_t id, uint16_t address);

void        read2ByteTx1        	(int port_num, uint8_t id, uint16_t address);
uint16_t    read2ByteRx1        	(int port_num);
uint16_t    read2ByteTxRx1      	(int port_num, uint8_t id, uint16_t address);

void        read4ByteTx1        	(int port_num, uint8_t id, uint16_t address);
uint32_t    read4ByteRx1        	(int port_num);
uint32_t    read4ByteTxRx1      	(int port_num, uint8_t id, uint16_t address);
uint8_t* 		readNByteTxRx1				(int port_num, uint8_t id, uint16_t address, uint8_t length);

void        writeTxOnly1        	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        writeTxRx1          	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        write1ByteTxOnly1   	(int port_num, uint8_t id, uint16_t address, uint8_t data);
void        write1ByteTxRx1     	(int port_num, uint8_t id, uint16_t address, uint8_t data);

void        write2ByteTxOnly1   	(int port_num, uint8_t id, uint16_t address, uint16_t data);
void        write2ByteTxRx1     	(int port_num, uint8_t id, uint16_t address, uint16_t data);

void        write4ByteTxOnly1   	(int port_num, uint8_t id, uint16_t address, uint32_t data);
void        write4ByteTxRx1     	(int port_num, uint8_t id, uint16_t address, uint32_t data);

void        regWriteTxOnly1     	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        regWriteTxRx1       	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        syncReadTx1         	(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
void        syncWriteTxOnly1    	(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);

// param : LEN1 ID1 ADDR1 LEN2 ID2 ADDR2 ...
void        bulkReadTx1         	(int port_num, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
void        bulkWriteTxOnly1    	(int port_num, uint16_t param_length);

/*
 * 
 * protocol2_packet_handler.h
 * 
 * Protocol 2 packet handling funtions.
 * 
 */

uint16_t    updateCRC           	(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
void        addStuffing         	(uint8_t *packet);
void        removeStuffing      	(uint8_t *packet);

void        printTxRxResult2    	(int result);
void        printRxPacketError2   (uint8_t error);

int         getLastTxRxResult2  	(int port_num);
uint8_t     getLastRxPacketError2 (int port_num);

void        setDataWrite2       	(int port_num, uint16_t data_length, uint16_t data_pos, uint32_t data);
uint32_t    getDataRead2        	(int port_num, uint16_t data_length, uint16_t data_pos);

void        txPacket2           	(int port_num);
void        rxPacket2           	(int port_num);
void        txRxPacket2         	(int port_num);

void        ping2               	(int port_num, uint8_t id);
uint16_t    pingGetModelNum2    	(int port_num, uint8_t id);

// BroadcastPing
void        broadcastPing2      	(int port_num);
uint8_t     getBroadcastPingResult2 (int port_num, int id);

void        action2             	(int port_num, uint8_t id);
void        reboot2             	(int port_num, uint8_t id);
void        factoryReset2       	(int port_num, uint8_t id, uint8_t option);

void        readTx2             	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        readRx2             	(int port_num, uint16_t length);
void        readTxRx2           	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        read1ByteTx2        	(int port_num, uint8_t id, uint16_t address);
uint8_t     read1ByteRx2        	(int port_num);
uint8_t     read1ByteTxRx2      	(int port_num, uint8_t id, uint16_t address);

void        read2ByteTx2        	(int port_num, uint8_t id, uint16_t address);
uint16_t    read2ByteRx2        	(int port_num);
uint16_t    read2ByteTxRx2      	(int port_num, uint8_t id, uint16_t address);

void        read4ByteTx2        	(int port_num, uint8_t id, uint16_t address);
uint32_t    read4ByteRx2        	(int port_num);
uint32_t    read4ByteTxRx2      	(int port_num, uint8_t id, uint16_t address);
uint8_t* 		readNByteTxRx2				(int port_num, uint8_t id, uint16_t address, uint8_t length);

void        writeTxOnly2        	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        writeTxRx2          	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        write1ByteTxOnly2   	(int port_num, uint8_t id, uint16_t address, uint8_t data);
void        write1ByteTxRx2     	(int port_num, uint8_t id, uint16_t address, uint8_t data);

void        write2ByteTxOnly2   	(int port_num, uint8_t id, uint16_t address, uint16_t data);
void        write2ByteTxRx2     	(int port_num, uint8_t id, uint16_t address, uint16_t data);

void        write4ByteTxOnly2   	(int port_num, uint8_t id, uint16_t address, uint32_t data);
void        write4ByteTxRx2     	(int port_num, uint8_t id, uint16_t address, uint32_t data);

void        regWriteTxOnly2     	(int port_num, uint8_t id, uint16_t address, uint16_t length);
void        regWriteTxRx2       	(int port_num, uint8_t id, uint16_t address, uint16_t length);

void        syncReadTx2         	(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);
// syncReadRx   -> GroupSyncRead
// syncReadTxRx -> GroupSyncRead

// param : ID1 DATA0 DATA1 ... DATAn ID2 DATA0 DATA1 ... DATAn ID3 DATA0 DATA1 ... DATAn
void        syncWriteTxOnly2   		(int port_num, uint16_t start_address, uint16_t data_length, uint16_t param_length);

// param : ID1 ADDR_L1 ADDR_H1 LEN_L1 LEN_H1 ID2 ADDR_L2 ADDR_H2 LEN_L2 LEN_H2 ...
void        bulkReadTx2        		(int port_num, uint16_t param_length);
// bulkReadRx   -> GroupBulkRead
// bulkReadTxRx -> GroupBulkRead

// param : ID1 START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H DATA0 DATA1 ... DATAn ID2 START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H DATA0 DATA1 ... DATAn
void        bulkWriteTxOnly2   		(int port_num, uint16_t param_length);

/*
 * 
 * group_bulk_read.h
 * 
 * Bulk read funtions.
 * 
 */
 
int         groupBulkRead               (int port_num, int protocol_version);

uint8_t     groupBulkReadAddParam       (int group_num, uint8_t id, uint16_t start_address, uint16_t data_length);
void        groupBulkReadRemoveParam    (int group_num, uint8_t id);
void        groupBulkReadClearParam     (int group_num);

void        groupBulkReadTxPacket       (int group_num);
void        groupBulkReadRxPacket       (int group_num);
void        groupBulkReadTxRxPacket     (int group_num);

uint8_t     groupBulkReadIsAvailable    (int group_num, uint8_t id, uint16_t address, uint16_t data_length);
uint32_t    groupBulkReadGetData        (int group_num, uint8_t id, uint16_t address, uint16_t data_length);

/*
 * 
 * group_bulk_write.h
 * 
 * Bulk write funtions.
 * 
 */
 
int     		groupBulkWrite              (int port_num, int protocol_version);

uint8_t 		groupBulkWriteAddParam      (int group_num, uint8_t id, uint16_t start_address, uint16_t data_length, uint32_t data, uint16_t input_length);
void    		groupBulkWriteRemoveParam   (int group_num, uint8_t id);
uint8_t 		groupBulkWriteChangeParam   (int group_num, uint8_t id, uint16_t start_address, uint16_t data_length, uint32_t data, uint16_t input_length, uint16_t data_pos);
void    		groupBulkWriteClearParam    (int group_num);

void    		groupBulkWriteTxPacket      (int group_num);

/*
 * 
 * group_sync_read.h
 * 
 * Sync read funtions.
 * 
 */

int         groupSyncRead               (int port_num, int protocol_version, uint16_t start_address, uint16_t data_length);

uint8_t     groupSyncReadAddParam       (int group_num, uint8_t id);
void        groupSyncReadRemoveParam    (int group_num, uint8_t id);
void        groupSyncReadClearParam     (int group_num);

void        groupSyncReadTxPacket       (int group_num);
void        groupSyncReadRxPacket       (int group_num);
void        groupSyncReadTxRxPacket     (int group_num);

uint8_t     groupSyncReadIsAvailable    (int group_num, uint8_t id, uint16_t address, uint16_t data_length);
uint32_t    groupSyncReadGetData        (int group_num, uint8_t id, uint16_t address, uint16_t data_length);

/*
 * 
 * group_sync_write.h
 * 
 * Sync write funtions.
 * 
 */
 
int     		groupSyncWrite              (int port_num, int protocol_version, uint16_t start_address, uint16_t data_length);

uint8_t 		groupSyncWriteAddParam      (int group_num, uint8_t id, uint32_t data, uint16_t data_length);
void    		groupSyncWriteRemoveParam   (int group_num, uint8_t id);
uint8_t 		groupSyncWriteChangeParam   (int group_num, uint8_t id, uint32_t data, uint16_t data_length, uint16_t data_pos);
void    		groupSyncWriteClearParam    (int group_num);

void    		groupSyncWriteTxPacket      (int group_num);

/*
 * 
 * dxl high level functions definitions
 * 
 */
int 	dxl_open						( char *port_name, int baudrate );
void 	dxl_close						( char *port_name );
int 	dxl_read						( char*	port_name, u_int8_t	protocol, u_int8_t start_ID, u_int8_t	nb_device, u_int16_t start_address, u_int8_t data_length, u_int8_t sign, double* data );
int 	dxl_write						( char* port_name, u_int8_t protocol, u_int8_t start_ID, u_int8_t nb_device, u_int16_t start_address, u_int8_t data_length, double*	data );
char* dxl_model_nb_2_name	( uint16_t	dxl_model_number );
int 	dxl_scan						( char *port_name );
int 	dxl_status					( char *port_name, int baudrate, uint8_t devid, int proto );
int 	dxl_ping						( char *port_name, int baudrate, uint8_t devid, int proto );

/*
 * 
 * Actuators registers definitions
 * 
 */

// Register definition structure
typedef struct
{
	uint8_t					mem_type;
	uint16_t				address;
	uint8_t					size;
	const char*			short_description;
	const char*			description;
	uint8_t					access;
	int32_t					initial;
	uint8_t					data_type;
	int32_t					range_min;
	int32_t					range_max;
	double					unit_quantum;
	const char*			unit_name;	
}	dxl_registers_struct_type;

#define DXL_REG_MEM_EEPROM															0
#define DXL_REG_MEM_RAM																	1
#define DXL_REG_MEM_END																	2
#define DXL_REG_ACCESS_R																1
#define DXL_REG_ACCESS_W																2
#define DXL_REG_ACCESS_RW																3
#define DXL_REG_UNDEFINED_INITIAL_VALUE									0
#define DXL_REG_TYPE_UINT8															0
#define DXL_REG_TYPE_SINT8															1
#define DXL_REG_TYPE_UINT16															2
#define DXL_REG_TYPE_SINT16															3
#define DXL_REG_TYPE_UINT32															4
#define DXL_REG_TYPE_SINT32															5
#define DXL_REG_MAX_UINT32															4294967295
#define DXL_REG_MIN_SINT32															-2147483648
#define DXL_REG_MAX_SINT32															2147483647
#define DXL_REG_NO_UNIT																	"NA"
#define DXL_REG_DESCRIPTION_LENGTH											50

// XM430-W210 actuator registers
dxl_registers_struct_type dxl_reg_XM430_W210[] =	{
	{	DXL_REG_MEM_EEPROM, 0,		2,	"XM430-W210", "Model number", 
		DXL_REG_ACCESS_R, 1030,
		DXL_REG_TYPE_UINT16, 0, 65535, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 2,		4,	"Model Information", "Model Information",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT32, 0, 0, 0.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 6,		1,	"Version of Firmware", "Firmware Version",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 7,		1,	"ID,Dynamixel", "ID",
		DXL_REG_ACCESS_RW, 1,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 8,		1,	"Baud Rate", "Communication Baud Rate",
		DXL_REG_ACCESS_RW, 1,
		DXL_REG_TYPE_UINT8, 0, 7, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 9,		1,	"Return Delay Time", "Response Delay Time",
		DXL_REG_ACCESS_RW, 250,
		DXL_REG_TYPE_UINT8, 0, 254, 2.0, "us" },
		
	{	DXL_REG_MEM_EEPROM, 11,		1,	"Operating Mode", "Operating Mode",
		DXL_REG_ACCESS_RW, 3,
		DXL_REG_TYPE_UINT8, 0, 16, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 13,		1,	"Protocol Version", "Protocol Version",
		DXL_REG_ACCESS_RW, 2,
		DXL_REG_TYPE_UINT8, 1, 2, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 20,		4,	"Homing Offset", "Home Position Offset",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_SINT32, -1044479, 1044479, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 24,		4,	"Moving Threshold", "Velocity Threshold for Movement Detection",
		DXL_REG_ACCESS_RW, 10,
		DXL_REG_TYPE_SINT32, -1044479, 1044479, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 31,		1,	"Temperature Limit", "Maximum Internal Temperature Limit",
		DXL_REG_ACCESS_RW, 80,
		DXL_REG_TYPE_UINT8, 0, 100, 1.0, "°C" },
		
	{	DXL_REG_MEM_EEPROM, 32,		2,	"Max Voltage Limit", "Maximum Voltage Limit",
		DXL_REG_ACCESS_RW, 160,
		DXL_REG_TYPE_UINT16, 95, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 34,		2,	"Min Voltage Limit", "Minimum Voltage Limit",
		DXL_REG_ACCESS_RW, 95,
		DXL_REG_TYPE_UINT16, 95, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 36,		2,	"PWM Limit", "Maximum PWM Limit",
		DXL_REG_ACCESS_RW, 885,
		DXL_REG_TYPE_UINT16, 0, 885, 0.11299435028249, "%" },
		
	{	DXL_REG_MEM_EEPROM, 38,		2,	"Current Limit", "Maximum Current Limit",
		DXL_REG_ACCESS_RW, 1193,
		DXL_REG_TYPE_UINT16, 0, 1193, 2.69, "mA" },
		
	{	DXL_REG_MEM_EEPROM, 40,		4,	"Acceleration Limit", "Maximum Acceleration Limit",
		DXL_REG_ACCESS_RW, 32767,
		DXL_REG_TYPE_UINT32, 0, 32767, 0.0036, "RPM/min" },
		
	{	DXL_REG_MEM_EEPROM, 44,		4,	"Velocity Limit", "Maximum Velocity Limit",
		DXL_REG_ACCESS_RW, 480,
		DXL_REG_TYPE_UINT32, 0, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_EEPROM, 48,		4,	"Max Position Limit", "Maximum Position Limit",
		DXL_REG_ACCESS_RW, 4095,
		DXL_REG_TYPE_UINT32, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 52,		4,	"Min Position Limit", "Minimum Position Limit",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT32, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 63,		1,	"Shutdown", "Shutdown Dynamixel",
		DXL_REG_ACCESS_RW, 52,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		64,		1,	"Torque Enable", "Motor Torque On/Off",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		65,		1,	"LED", "Status LED On/Off",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		68,		1,	"Status Return Level", "Select Types of Status Return",
		DXL_REG_ACCESS_RW, 2,
		DXL_REG_TYPE_UINT8, 0, 2, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		69,		1,	"Registered Instruction", "Check Reception of Instruction",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		70,		1,	"Hardware Error Status", "Hardware Error Status",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		76,		2,	"Velocity I Gain", "I Gain of Velocity",
		DXL_REG_ACCESS_RW, 1920,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		78,		2,	"Velocity P Gain", "P Gain of Velocity",
		DXL_REG_ACCESS_RW, 50,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		80,		2,	"Position D Gain", "D Gain of Position",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		82,		2,	"Position I Gain", "I Gain of Position",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		84,		2,	"Position P Gain", "P Gain of Position",
		DXL_REG_ACCESS_RW, 800,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		88,		2,	"Feedforward 2nd Gain", "2nd Gain of Feed-Forward",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		90,		2,	"Feedforward 1st Gain", "1st Gain of Feed-Forward",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		100,	2,	"Goal PWM", "Target PWM Value",
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -885, 885, 0.11299435028249, "%" },
		
	{	DXL_REG_MEM_RAM, 		102,	2,	"Goal Current", "Target Current Value",
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -1193, 1193, 2.69, "mA" },
		
	{	DXL_REG_MEM_RAM, 		104,	4,	"Goal Velocity", "Target Velocity Value",
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1023, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		108,	4,	"Profile Acceleration", "Acceleration Value of Profile",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT32, 0, 32767, 0.0036, "RPM/min" },
		
	{	DXL_REG_MEM_RAM, 		112,	4,	"Profile Velocity", "Velocity Value of Profile",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT32, 0, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		116,	4,	"Goal Position", "Target Position Value", 
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1048575, 1048575, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		120,	2,	"Realtime Tick", "Count Time in millisecond", 
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, "ms" },
		
	{	DXL_REG_MEM_RAM, 		122,	1,	"Moving", "Movement Status",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		123,	1,	"Moving Status", "Detailed Information of Movement Status",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		124,	2,	"Present PWM", "Current PWM Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -885, 885, 0.11299435028249, "%" },
		
	{	DXL_REG_MEM_RAM, 		126,	2,	"Present Current", "Current Current Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -1193, 1193, 2.69, "mA" },
		
	{	DXL_REG_MEM_RAM, 		128,	4,	"Present Velocity", "Current Velocity Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1023, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		132,	4,	"Present Position", "Current Position Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1048575, 1048575, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		136,	4,	"Velocity Trajectory", "Target Velocity Trajectory Generated by Profile",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT32, 0, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		140,	4,	"Position Trajectory", "Target Position Trajectory Generated by Profile",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1048575, 1048575, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		144,	2,	"Present Input Voltage", "Current Input Voltage",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 255, 0.1, "V" },
		
	{	DXL_REG_MEM_RAM, 		146,	1,	"Present Temperature", "Current Internal Temperature", 
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 100, 1.0, "°C" },
		
	{	DXL_REG_MEM_END, 		147, 0, "", "", 0, 0, 0, 0, 0, 0.0, "" }
	
};

// XM430-W350 actuator registers
dxl_registers_struct_type dxl_reg_XM430_W350[] =	{
	{	DXL_REG_MEM_EEPROM, 0,		2,	"XM430-W350", "Model number", 
		DXL_REG_ACCESS_R, 1020,
		DXL_REG_TYPE_UINT16, 0, 65535, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 2,		4,	"Model Information", "Model Information",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT32, 0, 0, 0.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 6,		1,	"Version of Firmware", "Firmware Version",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 7,		1,	"ID,Dynamixel", "ID",
		DXL_REG_ACCESS_RW, 1,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 8,		1,	"Baud Rate", "Communication Baud Rate",
		DXL_REG_ACCESS_RW, 1,
		DXL_REG_TYPE_UINT8, 0, 7, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 9,		1,	"Return Delay Time", "Response Delay Time",
		DXL_REG_ACCESS_RW, 250,
		DXL_REG_TYPE_UINT8, 0, 254, 2.0, "us" },
		
	{	DXL_REG_MEM_EEPROM, 11,		1,	"Operating Mode", "Operating Mode",
		DXL_REG_ACCESS_RW, 3,
		DXL_REG_TYPE_UINT8, 0, 16, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 13,		1,	"Protocol Version", "Protocol Version",
		DXL_REG_ACCESS_RW, 2,
		DXL_REG_TYPE_UINT8, 1, 2, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 20,		4,	"Homing Offset", "Home Position Offset",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_SINT32, -1044479, 1044479, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 24,		4,	"Moving Threshold", "Velocity Threshold for Movement Detection",
		DXL_REG_ACCESS_RW, 10,
		DXL_REG_TYPE_SINT32, -1044479, 1044479, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 31,		1,	"Temperature Limit", "Maximum Internal Temperature Limit",
		DXL_REG_ACCESS_RW, 80,
		DXL_REG_TYPE_UINT8, 0, 100, 1.0, "°C" },
		
	{	DXL_REG_MEM_EEPROM, 32,		2,	"Max Voltage Limit", "Maximum Voltage Limit",
		DXL_REG_ACCESS_RW, 160,
		DXL_REG_TYPE_UINT16, 95, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 34,		2,	"Min Voltage Limit", "Minimum Voltage Limit",
		DXL_REG_ACCESS_RW, 95,
		DXL_REG_TYPE_UINT16, 95, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 36,		2,	"PWM Limit", "Maximum PWM Limit",
		DXL_REG_ACCESS_RW, 885,
		DXL_REG_TYPE_UINT16, 0, 885, 0.11299435028249, "%" },
		
	{	DXL_REG_MEM_EEPROM, 38,		2,	"Current Limit", "Maximum Current Limit",
		DXL_REG_ACCESS_RW, 1193,
		DXL_REG_TYPE_UINT16, 0, 1193, 2.69, "mA" },
		
	{	DXL_REG_MEM_EEPROM, 40,		4,	"Acceleration Limit", "Maximum Acceleration Limit",
		DXL_REG_ACCESS_RW, 32767,
		DXL_REG_TYPE_UINT32, 0, 32767, 0.0036, "RPM/min" },
		
	{	DXL_REG_MEM_EEPROM, 44,		4,	"Velocity Limit", "Maximum Velocity Limit",
		DXL_REG_ACCESS_RW, 480,
		DXL_REG_TYPE_UINT32, 0, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_EEPROM, 48,		4,	"Max Position Limit", "Maximum Position Limit",
		DXL_REG_ACCESS_RW, 4095,
		DXL_REG_TYPE_UINT32, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 52,		4,	"Min Position Limit", "Minimum Position Limit",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT32, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_EEPROM, 63,		1,	"Shutdown", "Shutdown Dynamixel",
		DXL_REG_ACCESS_RW, 52,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		64,		1,	"Torque Enable", "Motor Torque On/Off",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		65,		1,	"LED", "Status LED On/Off",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		68,		1,	"Status Return Level", "Select Types of Status Return",
		DXL_REG_ACCESS_RW, 2,
		DXL_REG_TYPE_UINT8, 0, 2, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		69,		1,	"Registered Instruction", "Check Reception of Instruction",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		70,		1,	"Hardware Error Status", "Hardware Error Status",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		76,		2,	"Velocity I Gain", "I Gain of Velocity",
		DXL_REG_ACCESS_RW, 1920,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		78,		2,	"Velocity P Gain", "P Gain of Velocity",
		DXL_REG_ACCESS_RW, 50,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		80,		2,	"Position D Gain", "D Gain of Position",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		82,		2,	"Position I Gain", "I Gain of Position",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		84,		2,	"Position P Gain", "P Gain of Position",
		DXL_REG_ACCESS_RW, 800,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		88,		2,	"Feedforward 2nd Gain", "2nd Gain of Feed-Forward",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		90,		2,	"Feedforward 1st Gain", "1st Gain of Feed-Forward",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		100,	2,	"Goal PWM", "Target PWM Value",
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -885, 885, 0.11299435028249, "%" },
		
	{	DXL_REG_MEM_RAM, 		102,	2,	"Goal Current", "Target Current Value",
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -1193, 1193, 2.69, "mA" },
		
	{	DXL_REG_MEM_RAM, 		104,	4,	"Goal Velocity", "Target Velocity Value",
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1023, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		108,	4,	"Profile Acceleration", "Acceleration Value of Profile",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT32, 0, 32767, 0.0036, "RPM/min" },
		
	{	DXL_REG_MEM_RAM, 		112,	4,	"Profile Velocity", "Velocity Value of Profile",
		DXL_REG_ACCESS_RW, 0,
		DXL_REG_TYPE_UINT32, 0, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		116,	4,	"Goal Position", "Target Position Value", 
		DXL_REG_ACCESS_RW, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1048575, 1048575, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		120,	2,	"Realtime Tick", "Count Time in millisecond", 
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 32767, 1.0, "ms" },
		
	{	DXL_REG_MEM_RAM, 		122,	1,	"Moving", "Movement Status",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		123,	1,	"Moving Status", "Detailed Information of Movement Status",
		DXL_REG_ACCESS_R, 0,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		124,	2,	"Present PWM", "Current PWM Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -885, 885, 0.11299435028249, "%" },
		
	{	DXL_REG_MEM_RAM, 		126,	2,	"Present Current", "Current Current Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT16, -1193, 1193, 2.69, "mA" },
		
	{	DXL_REG_MEM_RAM, 		128,	4,	"Present Velocity", "Current Velocity Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1023, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		132,	4,	"Present Position", "Current Position Value",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1048575, 1048575, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		136,	4,	"Velocity Trajectory", "Target Velocity Trajectory Generated by Profile",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT32, 0, 1023, 0.229, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		140,	4,	"Position Trajectory", "Target Position Trajectory Generated by Profile",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_SINT32, -1048575, 1048575, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		144,	2,	"Present Input Voltage", "Current Input Voltage",
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 255, 0.1, "V" },
		
	{	DXL_REG_MEM_RAM, 		146,	1,	"Present Temperature", "Current Internal Temperature", 
		DXL_REG_ACCESS_R, DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 100, 1.0, "°C" },
		
	{	DXL_REG_MEM_END, 		147, 0, "", "", 0, 0, 0, 0, 0, 0.0, "" }
};

// MX 12W actuator
dxl_registers_struct_type dxl_reg_MX12W[] =	{
	{	DXL_REG_MEM_EEPROM, 0,	2,	"MX12W", "Model number",
		DXL_REG_ACCESS_R,		360,
		DXL_REG_TYPE_UINT16, 0, 65535, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 2,	1,	"Version of Firmware", "Information on the version of firmware",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 3,	1,	"ID", "ID of Dynamixel",
		DXL_REG_ACCESS_RW,	1,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 4,	1,	"Baud Rate", "Baud Rate of Dynamixel",
		DXL_REG_ACCESS_RW,	1,
		DXL_REG_TYPE_UINT8, 0, 249, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 5,	1,	"Return Delay Time", "Return Delay Time",
		DXL_REG_ACCESS_RW,	250,
		DXL_REG_TYPE_UINT8, 0, 254, 2.0, "us" },
		
	{	DXL_REG_MEM_EEPROM, 6,	2,	"CW Angle Limit", "Clockwise Angle Limit",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 8,	2,	"CCW Angle Limit", "Counterclockwise Angle Limit",
		DXL_REG_ACCESS_RW,	4095,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 11,	1,	"The Highest Limit Temperature", "Internal Temperature Limit",
		DXL_REG_ACCESS_RW,	80,
		DXL_REG_TYPE_UINT8, 0, 80, 1.0, "°C" },
		
	{	DXL_REG_MEM_EEPROM, 12,	1,	"The Lowest Limit Voltage", "Lowest Voltage Limit",
		DXL_REG_ACCESS_RW,	60,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 13,	1,	"The Highest Limit Voltage", "Highest Voltage Limit",
		DXL_REG_ACCESS_RW,	160,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 14,	2,	"Max Torque", "Maximum Torque",
		DXL_REG_ACCESS_RW,	1023,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_EEPROM, 16,	1,	"Status Return Level", "Status Return Level",
		DXL_REG_ACCESS_RW,	2,
		DXL_REG_TYPE_UINT8, 0, 2, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 17,	1,	"Alarm LED", "LED for Alarm",
		DXL_REG_ACCESS_RW,	36,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 18,	2,	"Alarm Shutdown", "Shutdown for Alarm",
		DXL_REG_ACCESS_RW,	36,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 20,	2,	"Multi Turn Offset", "Multi-turn offset",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 22,	2,	"Resolution Divider", "Resolution divider",
		DXL_REG_ACCESS_RW,	1,
		DXL_REG_TYPE_UINT8, 1, 4, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		24,	1,	"Torque Enable", "Torque On/Off",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		25,	1,	"LED", "LED On/Off",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		26,	1,	"D Gain", "Derivative Gain",
		DXL_REG_ACCESS_RW,	8,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		27,	1,	"I Gain", "Integral Gain",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		28,	2,	"P Gain", "Proportional Gain",
		DXL_REG_ACCESS_RW,	8,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		30,	2,	"Goal Position", "Goal Position",
		DXL_REG_ACCESS_RW,	DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		32,	2,	"Goal Speed", "Goal Speed",
		DXL_REG_ACCESS_RW,	DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 2047, 0.114, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		34,	2,	"Torque Limit", "Torque Limit",
		DXL_REG_ACCESS_RW,	1023,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		36,	2,	"Present Position", "Current Position",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		38,	2,	"Present Speed", "Current Speed",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 2047, 0.114, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		40,	2,	"Present Torque", "Current Torque",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		42,	1,	"Present Voltage", "Current Supplied Voltage",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_RAM, 		43,	1,	"Present Temperature", "Current Temperature",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 80, 1.0, "°C" },
		
	{	DXL_REG_MEM_RAM, 		44,	1,	"Registered", "Instruction is registered",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		46,	1,	"Moving", "Device in motion",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		47,	1,	"Lock", "Locking EEPROM",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		48,	2,	"Punch", "Punch",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		73,	1,	"Goal Acceleration", "Goal Acceleration",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 8.583, "deg/s/s" },
		
	{	DXL_REG_MEM_END, 		74, 0, "", "", 0, 0, 0, 0, 0, 0.0, "" }
};

// MX 28 actuator
dxl_registers_struct_type dxl_reg_MX28[] =	{
	{	DXL_REG_MEM_EEPROM, 0,	2,	"MX28", "Model number",
		DXL_REG_ACCESS_R,		29,
		DXL_REG_TYPE_UINT16, 0, 65535, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 2,	1,	"Version of Firmware", "Information on the version of firmware",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 3,	1,	"ID", "ID of Dynamixel",
		DXL_REG_ACCESS_RW,	1,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 4,	1,	"Baud Rate", "Baud Rate of Dynamixel",
		DXL_REG_ACCESS_RW,	34,
		DXL_REG_TYPE_UINT8, 0, 249, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 5,	1,	"Return Delay Time", "Return Delay Time",
		DXL_REG_ACCESS_RW,	250,
		DXL_REG_TYPE_UINT8, 0, 254, 2.0, "us" },
		
	{	DXL_REG_MEM_EEPROM, 6,	2,	"CW Angle Limit", "Clockwise Angle Limit",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 8,	2,	"CCW Angle Limit", "Counterclockwise Angle Limit",
		DXL_REG_ACCESS_RW,	4095,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 11,	1,	"The Highest Limit Temperature", "Internal Temperature Limit",
		DXL_REG_ACCESS_RW,	80,
		DXL_REG_TYPE_UINT8, 0, 80, 1.0, "°C" },
		
	{	DXL_REG_MEM_EEPROM, 12,	1,	"The Lowest Limit Voltage", "Lowest Voltage Limit",
		DXL_REG_ACCESS_RW,	60,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 13,	1,	"The Highest Limit Voltage", "Highest Voltage Limit",
		DXL_REG_ACCESS_RW,	160,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 14,	2,	"Max Torque", "Maximum Torque",
		DXL_REG_ACCESS_RW,	1023,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_EEPROM, 16,	1,	"Status Return Level", "Status Return Level",
		DXL_REG_ACCESS_RW,	2,
		DXL_REG_TYPE_UINT8, 0, 2, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 17,	1,	"Alarm LED", "LED for Alarm",
		DXL_REG_ACCESS_RW,	36,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 18,	2,	"Alarm Shutdown", "Shutdown for Alarm",
		DXL_REG_ACCESS_RW,	36,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 20,	2,	"Multi Turn Offset", "Multi-turn offset",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 22,	2,	"Resolution Divider", "Resolution divider",
		DXL_REG_ACCESS_RW,	1,
		DXL_REG_TYPE_UINT8, 1, 4, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		24,	1,	"Torque Enable", "Torque On/Off",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		25,	1,	"LED", "LED On/Off",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		26,	1,	"D Gain", "Derivative Gain",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		27,	1,	"I Gain", "Integral Gain",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		28,	2,	"P Gain", "Proportional Gain",
		DXL_REG_ACCESS_RW,	32,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		30,	2,	"Goal Position", "Goal Position",
		DXL_REG_ACCESS_RW,	DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		32,	2,	"Goal Speed", "Goal Speed",
		DXL_REG_ACCESS_RW,	DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 2047, 0.114, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		34,	2,	"Torque Limit", "Torque Limit",
		DXL_REG_ACCESS_RW,	1023,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		36,	2,	"Present Position", "Current Position",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		38,	2,	"Present Speed", "Current Speed",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 2047, 0.114, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		40,	2,	"Present Torque", "Current Torque",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		42,	1,	"Present Voltage", "Current Supplied Voltage",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_RAM, 		43,	1,	"Present Temperature", "Current Temperature",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 80, 1.0, "°C" },
		
	{	DXL_REG_MEM_RAM, 		44,	1,	"Registered", "Instruction is registered",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		46,	1,	"Moving", "Device in motion",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		47,	1,	"Lock", "Locking EEPROM",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		48,	2,	"Punch", "Punch",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		73,	1,	"Goal Acceleration", "Goal Acceleration",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 8.583, "deg/s/s" },
		
	{	DXL_REG_MEM_END, 		74, 0, "", "", 0, 0, 0, 0, 0, 0.0, "" }
};

// MX 64 actuator
dxl_registers_struct_type dxl_reg_MX64[] =	{
	{	DXL_REG_MEM_EEPROM, 0,	2,	"MX64", "Model number",
		DXL_REG_ACCESS_R,		310,
		DXL_REG_TYPE_UINT16, 0, 65535, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 2,	1,	"Version of Firmware", "Information on the version of firmware",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 3,	1,	"ID", "ID of Dynamixel",
		DXL_REG_ACCESS_RW,	1,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 4,	1,	"Baud Rate", "Baud Rate of Dynamixel",
		DXL_REG_ACCESS_RW,	34,
		DXL_REG_TYPE_UINT8, 0, 249, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 5,	1,	"Return Delay Time", "Return Delay Time",
		DXL_REG_ACCESS_RW,	250,
		DXL_REG_TYPE_UINT8, 0, 254, 2.0, "us" },
		
	{	DXL_REG_MEM_EEPROM, 6,	2,	"CW Angle Limit", "Clockwise Angle Limit",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 8,	2,	"CCW Angle Limit", "Counterclockwise Angle Limit",
		DXL_REG_ACCESS_RW,	4095,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 11,	1,	"The Highest Limit Temperature", "Internal Temperature Limit",
		DXL_REG_ACCESS_RW,	80,
		DXL_REG_TYPE_UINT8, 0, 80, 1.0, "°C" },
		
	{	DXL_REG_MEM_EEPROM, 12,	1,	"The Lowest Limit Voltage", "Lowest Voltage Limit",
		DXL_REG_ACCESS_RW,	60,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 13,	1,	"The Highest Limit Voltage", "Highest Voltage Limit",
		DXL_REG_ACCESS_RW,	160,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_EEPROM, 14,	2,	"Max Torque", "Maximum Torque",
		DXL_REG_ACCESS_RW,	1023,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_EEPROM, 16,	1,	"Status Return Level", "Status Return Level",
		DXL_REG_ACCESS_RW,	2,
		DXL_REG_TYPE_UINT8, 0, 2, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 17,	1,	"Alarm LED", "LED for Alarm",
		DXL_REG_ACCESS_RW,	36,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 18,	2,	"Alarm Shutdown", "Shutdown for Alarm",
		DXL_REG_ACCESS_RW,	36,
		DXL_REG_TYPE_UINT8, 0, 255, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_EEPROM, 20,	2,	"Multi Turn Offset", "Multi-turn offset",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_SINT16, -28672, 28672, 1.0, "counts" },
		
	{	DXL_REG_MEM_EEPROM, 22,	2,	"Resolution Divider", "Resolution divider",
		DXL_REG_ACCESS_RW,	1,
		DXL_REG_TYPE_UINT8, 1, 4, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		24,	1,	"Torque Enable", "Torque On/Off",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		25,	1,	"LED", "LED On/Off",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		26,	1,	"D Gain", "Derivative Gain",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		27,	1,	"I Gain", "Integral Gain",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		28,	2,	"P Gain", "Proportional Gain",
		DXL_REG_ACCESS_RW,	32,
		DXL_REG_TYPE_UINT8, 0, 254, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		30,	2,	"Goal Position", "Goal Position",
		DXL_REG_ACCESS_RW,	DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		32,	2,	"Goal Speed", "Goal Speed",
		DXL_REG_ACCESS_RW,	DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 2047, 0.114, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		34,	2,	"Torque Limit", "Torque Limit",
		DXL_REG_ACCESS_RW,	1023,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		36,	2,	"Present Position", "Current Position",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 4095, 0.087890625, "deg" },
		
	{	DXL_REG_MEM_RAM, 		38,	2,	"Present Speed", "Current Speed",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 2047, 0.114, "RPM" },
		
	{	DXL_REG_MEM_RAM, 		40,	2,	"Present Torque", "Current Torque",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
		
	{	DXL_REG_MEM_RAM, 		42,	1,	"Present Voltage", "Current Supplied Voltage",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 50, 160, 0.1, "V" },
		
	{	DXL_REG_MEM_RAM, 		43,	1,	"Present Temperature", "Current Temperature",
		DXL_REG_ACCESS_R,		DXL_REG_UNDEFINED_INITIAL_VALUE,
		DXL_REG_TYPE_UINT8, 0, 80, 1.0, "°C" },
		
	{	DXL_REG_MEM_RAM, 		44,	1,	"Registered", "Instruction is registered",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		46,	1,	"Moving", "Device in motion",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		47,	1,	"Lock", "Locking EEPROM",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		48,	2,	"Punch", "Punch",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.1, "%" },
	
	{	DXL_REG_MEM_RAM, 		68,	2,	"Motor current", "Motor current",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.0045, "(X-9.2115) A" },
	
	{	DXL_REG_MEM_RAM, 		70,	1,	"Torque mode enable", "Torque control on/off",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 1, 1.0, DXL_REG_NO_UNIT },
		
	{	DXL_REG_MEM_RAM, 		71,	2,	"Goal torque", "Goal torque value",
		DXL_REG_ACCESS_R,		0,
		DXL_REG_TYPE_UINT16, 0, 1023, 0.0045, "(X-9.2115) A" },
		
	{	DXL_REG_MEM_RAM, 		73,	1,	"Goal Acceleration", "Goal Acceleration",
		DXL_REG_ACCESS_RW,	0,
		DXL_REG_TYPE_UINT8, 0, 254, 8.583, "deg/s/s" },
		
	{	DXL_REG_MEM_END, 		74, 0, "", "", 0, 0, 0, 0, 0, 0.0, "" }
};

// List of actuators register definitions
dxl_registers_struct_type* dxl_reg_list[] = {
	dxl_reg_XM430_W210,
	dxl_reg_XM430_W350,
	dxl_reg_MX12W,
	dxl_reg_MX28,
	dxl_reg_MX64
};
