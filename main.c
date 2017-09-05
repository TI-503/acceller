//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Out of Box
// Application Overview - This Application demonstrates Out of Box Experience 
//                        with CC32xx Launch Pad. It highlights the following 
//                        features:
//                     1. Easy Connection to CC3200 Launchpad
//                        - Direct Connection to LP by using CC3200 device in
//                          Access Point Mode(Default)
//                        - Connection using TI SmartConfigTechnology
//                     2. Easy access to CC3200 Using Internal HTTP server and 
//                        on-board webcontent
//                     3. Attractive Demos
//                        - Home Automation
//                        - Appliance Control
//                        - Security System
//                        - Thermostat
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Out_of_Box_Application
// or
// docs\examples\CC32xx_Out_of_Box_Application.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup oob
//! @{
//
//****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>

// Simplelink includes
#include "simplelink.h"
#include "netcfg.h"

// Driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "utils.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "pin.h"

// OS includes
#include "osi.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "i2c_if.h"
#include "common.h"

// App Includes
#include "device_status.h"
#include "smartconfig.h"
#include "tmp006drv.h"
#include "bma222drv.h"
#include "pinmux.h"

#define APPLICATION_NAME        "Acceleration TCP Server"
#define APPLICATION_VERSION     "1.1.1"

#define HOST_NAME               "www.jackh.cn"

//
// Values for below macros shall be modified for setting the 'Ping' properties
//
#define PING_INTERVAL       1000    /* In msecs */
#define PING_TIMEOUT        3000    /* In msecs */
#define PING_PKT_SIZE       20      /* In bytes */
#define NO_OF_ATTEMPTS      3

#define OSI_STACK_SIZE      1024

#define IP_ADDR             0xc0a80A6E /* 192.168.1.110 */
#define PORT_NUM            5001
#define BUF_SIZE            100
#define TCP_PACKET_COUNT    1

#define FOREVER             1
#define NO_OF_SAMPLES 		8

// Application specific status/error codes
typedef enum{
	// Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
	LAN_CONNECTION_FAILED = -0x7D0,
	INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
	DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

	SOCKET_CREATE_ERROR = -0x7D0,
	BIND_ERROR = SOCKET_CREATE_ERROR - 1,
	LISTEN_ERROR = BIND_ERROR -1,
	SOCKET_OPT_ERROR = LISTEN_ERROR -1,
	CONNECT_ERROR = SOCKET_OPT_ERROR -1,
	ACCEPT_ERROR = CONNECT_ERROR - 1,
	SEND_ERROR = ACCEPT_ERROR -1,
	RECV_ERROR = SEND_ERROR -1,
	SOCKET_CLOSE_ERROR = RECV_ERROR -1,

	STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

unsigned long  g_ulDestinationIp = IP_ADDR;
unsigned int   g_uiPortNum = PORT_NUM;
volatile unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;
unsigned char  g_ucConnectionStatus = 0;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;
char g_cBsdBuf[BUF_SIZE];

unsigned int connect_flag = 0;

float g_temp;
int g_iServerSockID;
int g_sockID;
int g_isShake;
int g_shakeEn;


signed char g_cAccX,g_cAccY,g_cAccZ;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
int BsdTcpClient(unsigned short usPort);
int BsdTcpServer(unsigned short usPort);
int IpAddressParser(char *ucCMD);
static long WlanConnect();
void WlanStationMode( void *pvParameters );
static void InitializeAppVariables();
static long ConfigureSimpleLinkToDefaultState();
void CollectTmp( void *pvParameters );
void CollectAcc( void *pvParameters );


#ifdef USE_FREERTOS
//*****************************************************************************
// FreeRTOS User Hook Functions enabled in FreeRTOSConfig.h
//*****************************************************************************

//*****************************************************************************
//
//! \brief Application defined hook (or callback) function - assert
//!
//! \param[in]  pcFile - Pointer to the File Name
//! \param[in]  ulLine - Line Number
//!
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
	//Handle Assert here
	while(1)
	{
	}
}

void DelayMs(float time)
{
	unsigned long ulCount;
	ulCount=80000*time/6;
	UtilsDelay((unsigned long)ulCount);
}

void doubleShake(float src_time, float dest_time)
{
	g_shakeEn = 1;
	float delta = 0.004;
	if(src_time<dest_time)
	{

	}
	else
	{
		delta = - delta;
	}
	//Report("delta: %f\r\n",delta);
	float start = src_time;
	float start_4 = (float)((float)3.2 - src_time);
	int loop = (int) ((dest_time - src_time)/delta+1);
	int i=0;
	for( i=0;i<loop;i++)
	{
		//Report("start: %f, %f\r\n",start,start_4);
		GPIOPinWrite(GPIOA0_BASE, 0x8, 0x8);
		//DelayMs(start);
		MAP_UtilsDelay((unsigned long)80000*start/6);
		GPIOPinWrite(GPIOA0_BASE, 0x8, 0);
		MAP_UtilsDelay((unsigned long)80000*(20-start)/6);
		//DelayMs(20.0-start);


		GPIOPinWrite(GPIOA1_BASE, 0x20,0x20);
		//DelayMs(start_4);
		MAP_UtilsDelay((unsigned long)80000*start_4/6);
		GPIOPinWrite(GPIOA1_BASE, 0x20,0);
		//DelayMs(20-start_4);
		MAP_UtilsDelay((unsigned long)80000*(20-start_4)/6);

		start += delta;
		start_4 -= delta;
	}
}

void noShake()
{
	g_shakeEn = 0;
	int i = 0;
	for(i=0;i<20;i++)
	{
		GPIOPinWrite(GPIOA0_BASE, 0x8, 0x8);
		GPIOPinWrite(GPIOA1_BASE, 0x20,0x20);
		//DelayMs(1.6);
		MAP_UtilsDelay((unsigned long)80000*1.6/6);
		GPIOPinWrite(GPIOA0_BASE, 0x8, 0);
		GPIOPinWrite(GPIOA1_BASE, 0x20,0);
		//DelayMs(20-1.6);
		MAP_UtilsDelay((unsigned long)80000*(20-1.6)/6);

	}
}
//*****************************************************************************
//
//! \brief Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook( void)
{
	//Handle Idle Hook for Profiling, Power Management etc
}

//*****************************************************************************
//
//! \brief Application defined malloc failed hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook()
{
	//Handle Memory Allocation Errors
	while(1)
	{
	}
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook( OsiTaskHandle *pxTask,
		signed char *pcTaskName)
{
	//Handle FreeRTOS Stack Overflow
	while(1)
	{
	}
}
#endif //USE_FREERTOS


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
	switch(pWlanEvent->Event)
	{
	case SL_WLAN_CONNECT_EVENT:
	{
		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

		//
		// Information about the connected AP (like name, MAC etc) will be
		// available in 'slWlanConnectAsyncResponse_t'-Applications
		// can use it if required
		//
		//  slWlanConnectAsyncResponse_t *pEventData = NULL;
		// pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
		//

		// Copy new connection SSID and BSSID to global parameters
		memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
				STAandP2PModeWlanConnected.ssid_name,
				pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
		memcpy(g_ucConnectionBSSID,
				pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
				SL_BSSID_LENGTH);

		UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
				"BSSID: %x:%x:%x:%x:%x:%x\n\r",
				g_ucConnectionSSID,g_ucConnectionBSSID[0],
				g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
				g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
				g_ucConnectionBSSID[5]);
	}
	break;

	case SL_WLAN_DISCONNECT_EVENT:
	{
		slWlanConnectAsyncResponse_t*  pEventData = NULL;

		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
		CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

		pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

		// If the user has initiated 'Disconnect' request,
		//'reason_code' is SL_USER_INITIATED_DISCONNECTION
		if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
		{
			UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
					"BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
					g_ucConnectionSSID,g_ucConnectionBSSID[0],
					g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
					g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
					g_ucConnectionBSSID[5]);
		}
		else
		{
			UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
					"BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
					g_ucConnectionSSID,g_ucConnectionBSSID[0],
					g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
					g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
					g_ucConnectionBSSID[5]);
		}
		memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
		memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
	}
	break;

	default:
	{
		UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
				pWlanEvent->Event);
	}
	break;
	}
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
	switch(pNetAppEvent->Event)
	{
	case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
	{
		SlIpV4AcquiredAsync_t *pEventData = NULL;

		SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

		//Ip Acquired Event Data
		pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

		//Gateway IP address
		g_ulGatewayIP = pEventData->gateway;

		UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
				"Gateway=%d.%d.%d.%d\n\r",
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
				SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
	}
	break;

	default:
	{
		UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
				pNetAppEvent->Event);
	}
	break;
	}
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
		SlHttpServerResponse_t *pHttpResponse)
{
	// Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
	//
	// Most of the general errors are not FATAL are are to be handled
	// appropriately by the application
	//
	UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
			pDevEvent->EventData.deviceEvent.status,
			pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
	//
	// This application doesn't work w/ socket - Events are not expected
	//
	switch( pSock->Event )
	{
	case SL_SOCKET_TX_FAILED_EVENT:
		switch( pSock->socketAsyncEvent.SockTxFailData.status)
		{
		case SL_ECLOSE:
			UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
					"failed to transmit all queued packets\n\n",
					pSock->socketAsyncEvent.SockTxFailData.sd);
			break;
		default:
			UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
					"(%d) \n\n",
					pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
			break;
		}
		break;

		default:
			UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
			break;
	}

}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! Send Thread
//!
//! This function
//!        1. Function for reading the user input for UDP RX/TX
//!
//!  \return 0 : Success, -ve : failure
//
//*****************************************************************************
void InitTcpServer( void *pvParameters )
{

	while(!connect_flag);

	UART_PRINT("Default settings: SSID Name: %s, PORT = %d, Packet Count = %d, "
			"Destination IP: %d.%d.%d.%d\n\r",
			SSID_NAME, g_uiPortNum, g_ulPacketCount,
			SL_IPV4_BYTE(g_ulDestinationIp,3),
			SL_IPV4_BYTE(g_ulDestinationIp,2),
			SL_IPV4_BYTE(g_ulDestinationIp,1),
			SL_IPV4_BYTE(g_ulDestinationIp,0));



	BsdTcpServer(g_uiPortNum);


	return;

}


//****************************************************************************
//
//! \brief Opening a TCP client side socket and sending data
//!
//! This function opens a TCP socket and tries to connect to a Server IP_ADDR
//!    waiting on port PORT_NUM.
//!    If the socket connection is successful then the function will send 1000
//! TCP packets to the server.
//!
//! \param[in]      port number on which the server will be listening on
//!
//! \return    0 on success, -1 on Error.
//
//****************************************************************************
int BsdTcpClient(unsigned short usPort)
{
	int             iCounter;

	SlSockAddrIn_t  sAddr;
	int             iAddrSize;
	int             iSockID;
	int             iStatus;


	// filling the buffer
	for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
	{
		g_cBsdBuf[iCounter] = (char)(iCounter % 10);
	}

	//filling the TCP server socket address
	sAddr.sin_family = SL_AF_INET;
	sAddr.sin_port = sl_Htons((unsigned short)usPort);
	sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

	iAddrSize = sizeof(SlSockAddrIn_t);

	// creating a TCP socket
	iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
	if( iSockID < 0 )
	{
		ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
	}

	// connecting to TCP server
	iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
	if( iStatus < 0 )
	{
		// error
		sl_Close(iSockID);
		ASSERT_ON_ERROR(CONNECT_ERROR);
	}

	g_sockID = iSockID;
	//iStatus = sl_Close(iSockID);
	//closing the socket after sending 1000 packets
	//ASSERT_ON_ERROR(iStatus);

	return SUCCESS;
}

void sendPacket()
{
	SlSockAddrIn_t  sAddr;
	int             iAddrSize;
	int             iStatus;
	int             iNewSockID;	// client socket id

	long            lLoopCount = 0;
	int             iTestBufLen;
	int				iCounter;
	char			RotateMsg[6];
	iTestBufLen = BUF_SIZE;
	iAddrSize = sizeof(SlSockAddrIn_t);

	iNewSockID = SL_EAGAIN;


	// filling the buffer
	//	for (iCounter = 0; iCounter<BUF_SIZE; iCounter++)
	//	{
	//		g_cBsdBuf[iCounter] = 0;
	//	}
	memset(g_cBsdBuf,0,sizeof(g_cBsdBuf));
	//delaySec(10);
	Report("================================1\n\rWaiting for connection on 5001...\n\r");
	// waiting for an incoming TCP connection
	while (iNewSockID < 0)
	{
		// if shake  then shake
		// else than back to mid_pos
		if (g_isShake == 1) // to left
		{
			doubleShake(1.6,1.5);
			doubleShake(1.5,1.6);
			g_isShake = 2;
		}
		else if(g_isShake == 2)
		{
			doubleShake(1.6,1.7);
			doubleShake(1.7,1.6);
			g_isShake = 1;
		}
		else
		{
			if(g_shakeEn == 1)
			{
				noShake();
			}
		}

		// accepts a connection form a TCP client, if there is any
		// otherwise returns SL_EAGAIN
		iNewSockID = sl_Accept(g_iServerSockID, (struct SlSockAddr_t *)&sAddr,
				(SlSocklen_t*)&iAddrSize);
		if (iNewSockID == SL_EAGAIN)
		{
			MAP_UtilsDelay(10000);
		}
		else if (iNewSockID < 0)
		{
			// error
			sl_Close(iNewSockID);
			//sl_Close(iSockID);
			ASSERT_ON_ERROR(ACCEPT_ERROR);
		}
		//Report("%d,%d\r\n",g_isShake ,g_shakeEn);

	}

	Report("Accept a new connectiong from %d.%d.%d.%d:%d\n\r",
			SL_IPV4_BYTE(sAddr.sin_addr.s_addr, 0),
			SL_IPV4_BYTE(sAddr.sin_addr.s_addr, 1),
			SL_IPV4_BYTE(sAddr.sin_addr.s_addr, 2),
			SL_IPV4_BYTE(sAddr.sin_addr.s_addr, 3),
			sl_Htons(sAddr.sin_port));

	// waits for 1 packets from the connected TCP client otherwise skip.
	SlSockNonblocking_t enableOption;
	enableOption.NonblockingEnabled = 1;
	sl_SetSockOpt(iNewSockID,SL_SOL_SOCKET,SL_SO_NONBLOCKING, (_u8 *)&enableOption,sizeof(enableOption));


	// recv data
	iStatus = sl_Recv(iNewSockID, g_cBsdBuf, iTestBufLen, 0);
	if (iStatus <= 0)
	{
		// error
		sl_Close(iNewSockID);
		//sl_Close(iSockID);
		ASSERT_ON_ERROR(RECV_ERROR);
	}
	Report("Recieved data %s\n\r",g_cBsdBuf);

	if(strcmp(g_cBsdBuf,"getdata") == 0)
	{
		int first = 0;

		while(1)
		{
			if(g_isShake ==0)
			{
				if(g_shakeEn == 1)
				{
					noShake();
				}
			}
			else if (g_isShake == 1) // to left
			{
				doubleShake(1.6,1.5);
				doubleShake(1.5,1.6);
				g_isShake = 2;
			}
			else if(g_isShake == 2)
			{
				doubleShake(1.6,1.7);
				doubleShake(1.7,1.6);
				g_isShake = 1;
			}
			// else // g_isShake == 0

			//Get the TMP and Acc value
			CollectTmp(NULL);
			osi_Sleep(200);
			CollectAcc(NULL);
			if(first != 0)
			{
				// 收数据
				// recv data
				memset(g_cBsdBuf,0,sizeof(g_cBsdBuf));
				iStatus = sl_Recv(iNewSockID, g_cBsdBuf, iTestBufLen, 0);
				if (iStatus <= 0)
				{
					// error
					sl_Close(iNewSockID);
					//sl_Close(iSockID);
					ASSERT_ON_ERROR(RECV_ERROR);
				}
				Report("Recieved data %s\n\r",g_cBsdBuf);
				if(strcmp(g_cBsdBuf,"shake") == 0)
				{
					g_isShake = 1;
				}
				else if(strcmp(g_cBsdBuf,"noshake") == 0)
				{
					g_isShake = 0;
					noShake();
				}
			}
			first = 1;
			// send data
			sprintf(g_cBsdBuf, "Temp is %.2f F AccX=%d,AccY=%d,AccZ=%d\n\r", g_temp, g_cAccX, g_cAccY, g_cAccZ);
			UART_PRINT("socket - Temp is %.2f F AccX=%d,AccY=%d,AccZ=%d\n\r", g_temp, g_cAccX, g_cAccY, g_cAccZ);
			UART_PRINT("%s", g_cBsdBuf);
			// sending packet
			iStatus = sl_Send(iNewSockID, g_cBsdBuf, iTestBufLen, 0 );
			osi_Sleep(50);
			if (iStatus <= 0)
			{
				// error
				sl_Close(iNewSockID);

				Report("error send\r\n");
				//break;
				ASSERT_ON_ERROR(SEND_ERROR);

			}
			Report("Sent packets successfully\n\r");
			Report("%d, %d\r\n", g_isShake, g_shakeEn);
		}

	}
	else if(strcmp(g_cBsdBuf,"shake") == 0)
	{
		g_isShake = 1;
	}
	else if(strcmp(g_cBsdBuf,"noshake") == 0)
	{
		g_isShake = 0;
	}
	else
	{
		UART_PRINT("Command Error\r\n");
		sprintf(g_cBsdBuf, "Command Error\r\n");
		sl_Send(iNewSockID, g_cBsdBuf, 60, 0);
	}

	sl_Close(iNewSockID);
	Report("Connection closed.\n\r");
	Report("================================\n\r");
}

//****************************************************************************
//
//! \brief Opening a TCP server side socket and receiving data
//!
//! This function opens a TCP socket in Listen mode and waits for an incoming
//!    TCP connection.
//! If a socket connection is established then the function will try to read
//!    1000 TCP packets from the connected client.
//!
//! \param[in] port number on which the server will be listening on
//!
//! \return     0 on success, -1 on error.
//!
//! \note   This function will wait for an incoming connection till
//!                     one is established
//
//****************************************************************************
int BsdTcpServer(unsigned short usPort)
{
	SlSockAddrIn_t  sAddr;
	SlSockAddrIn_t  sLocalAddr;
	int             iCounter;
	int             iAddrSize;
	int             iSockID;
	int             iStatus;
	int             iNewSockID;
	long            lLoopCount = 0;
	long            lNonBlocking = 1;
	int             iTestBufLen;

	// filling the buffer
	for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
	{
		g_cBsdBuf[iCounter] = (char)(iCounter % 10);
	}

	iTestBufLen  = BUF_SIZE;

	//filling the TCP server socket address
	sLocalAddr.sin_family = SL_AF_INET;
	sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
	sLocalAddr.sin_addr.s_addr = 0;

	// creating a TCP socket
	iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
	if( iSockID < 0 )
	{
		// error
		ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
	}
	g_iServerSockID = iSockID;

	iAddrSize = sizeof(SlSockAddrIn_t);

	// binding the TCP socket to the TCP server address
	iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
	if( iStatus < 0 )
	{
		// error
		sl_Close(iSockID);
		ASSERT_ON_ERROR(BIND_ERROR);
	}

	// putting the socket for listening to the incoming TCP connection
	iStatus = sl_Listen(iSockID, 0);
	if( iStatus < 0 )
	{
		sl_Close(iSockID);
		ASSERT_ON_ERROR(LISTEN_ERROR);
	}

	// setting socket option to make the socket as non blocking
	iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
			&lNonBlocking, sizeof(lNonBlocking));
	if( iStatus < 0 )
	{
		sl_Close(iSockID);
		ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
	}

	return SUCCESS;
}

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
	g_ulStatus = 0;
	g_ulPingPacketsRecv = 0;
	g_ulGatewayIP = 0;
	memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
	memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************

static long ConfigureSimpleLinkToDefaultState()
{
	SlVersionFull   ver = {0};
	_WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

	unsigned char ucVal = 1;
	unsigned char ucConfigOpt = 0;
	unsigned char ucConfigLen = 0;
	unsigned char ucPower = 0;

	long lRetVal = -1;
	long lMode = -1;

	lMode = sl_Start(0, 0, 0);
	ASSERT_ON_ERROR(lMode);

	// If the device is not in station-mode, try configuring it in station-mode
	if (ROLE_STA != lMode)
	{
		if (ROLE_AP == lMode)
		{
			// If the device is in AP mode, we need to wait for this event
			// before doing anything
			while(!IS_IP_ACQUIRED(g_ulStatus))
			{
#ifndef SL_PLATFORM_MULTI_THREADED
				_SlNonOsMainLoopTask();
#endif
			}
		}

		// Switch to STA role and restart
		lRetVal = sl_WlanSetMode(ROLE_STA);
		ASSERT_ON_ERROR(lRetVal);

		lRetVal = sl_Stop(0xFF);
		ASSERT_ON_ERROR(lRetVal);

		lRetVal = sl_Start(0, 0, 0);
		ASSERT_ON_ERROR(lRetVal);

		// Check if the device is in station again
		if (ROLE_STA != lRetVal)
		{
			// We don't want to proceed if the device is not coming up in STA-mode
			ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
		}
	}

	// Get the device's version-information
	ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
	ucConfigLen = sizeof(ver);
	lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
			&ucConfigLen, (unsigned char *)(&ver));
	ASSERT_ON_ERROR(lRetVal);

	UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
	UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
			ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
			ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
			ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
			ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
			ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

	// Set connection policy to Auto + SmartConfig
	//      (Device's default connection policy)
	lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
			SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Remove all profiles
	lRetVal = sl_WlanProfileDel(0xFF);
	ASSERT_ON_ERROR(lRetVal);



	//
	// Device in station-mode. Disconnect previous connection if any
	// The function returns 0 if 'Disconnected done', negative number if already
	// disconnected Wait for 'disconnection' event if 0 is returned, Ignore
	// other return-codes
	//
	lRetVal = sl_WlanDisconnect();
	if(0 == lRetVal)
	{
		// Wait
		while(IS_CONNECTED(g_ulStatus))
		{
#ifndef SL_PLATFORM_MULTI_THREADED
			_SlNonOsMainLoopTask();
#endif
		}
	}

	// Enable DHCP client
	lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
	ASSERT_ON_ERROR(lRetVal);

	// Disable scan
	ucConfigOpt = SL_SCAN_POLICY(0);
	lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Set Tx power level for station mode
	// Number between 0-15, as dB offset from max power - 0 will set max power
	ucPower = 0;
	lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
			WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
	ASSERT_ON_ERROR(lRetVal);

	// Set PM policy to normal
	lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Unregister mDNS services
	lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Remove  all 64 filters (8*8)
	memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
	lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
			sizeof(_WlanRxFilterOperationCommandBuff_t));
	ASSERT_ON_ERROR(lRetVal);

	lRetVal = sl_Stop(SL_STOP_TIMEOUT);
	ASSERT_ON_ERROR(lRetVal);

	InitializeAppVariables();

	return lRetVal; // Success
}

//****************************************************************************
//
//!    \brief Parse the input IP address from the user
//!
//!    \param[in]                     ucCMD (char pointer to input string)
//!
//!    \return                        0 : if correct IP, -1 : incorrect IP
//
//****************************************************************************
int IpAddressParser(char *ucCMD)
{
	volatile int i=0;
	unsigned int uiUserInputData;
	unsigned long ulUserIpAddress = 0;
	char *ucInpString;
	ucInpString = strtok(ucCMD, ".");
	uiUserInputData = (int)strtoul(ucInpString,0,10);
	while(i<4)
	{
		//
		// Check Whether IP is valid
		//
		if((ucInpString != NULL) && (uiUserInputData < 256))
		{
			ulUserIpAddress |= uiUserInputData;
			if(i < 3)
				ulUserIpAddress = ulUserIpAddress << 8;
			ucInpString=strtok(NULL,".");
			uiUserInputData = (int)strtoul(ucInpString,0,10);
			i++;
		}
		else
		{
			return -1;
		}
	}
	g_ulDestinationIp = ulUserIpAddress;
	return SUCCESS;
}

//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  None
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{
	SlSecParams_t secParams = {0};
	long lRetVal = 0;
	unsigned char policyVal;

	secParams.Key = (signed char*)SECURITY_KEY;
	secParams.KeyLen = strlen(SECURITY_KEY);
	secParams.Type = SECURITY_TYPE;

	//	锟斤拷锟絇rofile
	lRetVal = sl_WlanProfileAdd(SSID_NAME,strlen(SSID_NAME),0,&secParams,0,0,0);
	ASSERT_ON_ERROR(lRetVal);

	//Set the Connection Policy
	lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,SL_CONNECTION_POLICY(1,1,0,0,0),&policyVal, 1);
	ASSERT_ON_ERROR(lRetVal);

	// Connect to the AP
	//	lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
	//	ASSERT_ON_ERROR(lRetVal);

	// Wait for WLAN Event
	while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
	{
		// Toggle LEDs to Indicate Connection Progress
		GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
		MAP_UtilsDelay(800000);
		GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
		MAP_UtilsDelay(800000);
	}

	return SUCCESS;

}

//****************************************************************************
//
//! \brief Start simplelink, connect to the ap and run the ping test
//!
//! This function starts the simplelink, connect to the ap and start the ping
//! test on the default gateway for the ap
//!
//! \param[in]  pvParameters - Pointer to the list of parameters that
//!             can bepassed to the task while creating it
//!
//! \return  None
//
//****************************************************************************
void WlanStationMode( void *pvParameters )
{
	long lRetVal = -1;
	InitializeAppVariables();

	//
	// Following function configure the device to default state by cleaning
	// the persistent settings stored in NVMEM (viz. connection profiles &
	// policies, power policy etc)
	//
	// Applications may choose to skip this step if the developer is sure
	// that the device is in its default state at start of applicaton
	//
	// Note that all profiles and persistent settings that were done on the
	// device will be lost
	//
	lRetVal = ConfigureSimpleLinkToDefaultState();
	if(lRetVal < 0)
	{
		if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
		{
			UART_PRINT("Failed to configure the device in its default state\n\r");
		}

		LOOP_FOREVER();
	}

	UART_PRINT("Device is configured in default state \n\r");

	//
	// Assumption is that the device is configured in station mode already
	// and it is in its default state
	//
	lRetVal = sl_Start(0, 0, 0);
	if (lRetVal < 0 || ROLE_STA != lRetVal)
	{
		UART_PRINT("Failed to start the device \n\r");
		LOOP_FOREVER();
	}

	UART_PRINT("Device started as STATION \n\r");

	//
	//Connecting to WLAN AP
	//
	lRetVal = WlanConnect();
	if(lRetVal < 0)
	{
		UART_PRINT("Failed to establish connection w/ an AP \n\r");
		LOOP_FOREVER();
	}

	UART_PRINT("Connection established w/ AP and IP is aquired \n\r");

	connect_flag = 1;

	InitSendService();
}

void InitSendService()
{
	InitTcpServer(NULL);

	//LOOP_FOREVER();
	while(FOREVER)
	{
		sendPacket();
	}
}
//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

	UART_PRINT("\n\n\n\r");
	UART_PRINT("\t\t *************************************************\n\r");
	UART_PRINT("\t\t           CC3200 %s Application       \n\r", AppName);
	UART_PRINT("\t\t *************************************************\n\r");
	UART_PRINT("\n\n\n\r");
}
//*****************************************************************************
//
//! \brief  Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
	// In case of TI-RTOS vector table is initialize by OS itself
#ifndef USE_TIRTOS
	//
	// Set vector table base
	//
#if defined(ccs)
	MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
	MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif //USE_TIRTOS

	//
	// Enable Processor
	//
	MAP_IntMasterEnable();
	MAP_IntEnable(FAULT_SYSTICK);

	PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! \brief  Collect Data from TMP 006
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void CollectTmp( void *pvParameters )
{
	//
	// Initialize Array index for multiple execution
	float fCurrentTemp;
	TMP006DrvGetTemp(&fCurrentTemp);
	g_temp = fCurrentTemp;
}

//*****************************************************************************
//
//! \brief  Collect Data from BMA
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void CollectAcc( void *pvParameters )
{
	//
	// Initialize Array index for multiple execution
	//signed char cAccX,cAccY,cAccZ;
	BMA222Read(&g_cAccX, &g_cAccY, &g_cAccZ);

}

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
	long lRetVal = -1;
	//
	// Board Initialization
	//
	BoardInit();

	//
	// configure the GPIO pins for LEDs,UART
	//
	PinMuxConfig();
	PinConfigSet(PIN_58,PIN_STRENGTH_2MA|PIN_STRENGTH_4MA,PIN_TYPE_STD_PD);

	//
	// Configure the UART
	//
#ifndef NOTERM
	InitTerm();
#endif  //NOTERM

	//
	// Display Application Banner
	//
	DisplayBanner(APPLICATION_NAME);

	//
	// Configure all 3 LEDs
	//
	//GPIO_IF_LedConfigure(LED1|LED2|LED3);

	// switch off all LEDs
	//GPIO_IF_LedOff(MCU_ALL_LED_IND);

	lRetVal = I2C_IF_Open(I2C_MASTER_MODE_FST);
	if(lRetVal < 0)
	{
		return;
	}

	//Init Temprature Sensor
	lRetVal = TMP006DrvOpen();
	if(lRetVal < 0)
	{
		Report("wrong %d \n", lRetVal);
		return;
	}

	//Init Accelerometer Sensor
	lRetVal = BMA222Open();
	if(lRetVal < 0)
	{
		ERR_PRINT(lRetVal);
		LOOP_FOREVER();
	}

	// 舵机初始化
	g_isShake = 0;
	g_shakeEn = 1;
	//
	// Start the SimpleLink Host
	//
	lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
	if(lRetVal < 0)
	{
		ERR_PRINT(lRetVal);
		LOOP_FOREVER();
	}

	//
	// Start the WlanStationMode task
	//
	lRetVal = osi_TaskCreate( WlanStationMode, \
			(const signed char*)"Wlan Station Task", \
			OSI_STACK_SIZE, NULL, 2, NULL );
	if(lRetVal < 0)
	{
		ERR_PRINT(lRetVal);
		LOOP_FOREVER();
	}

	//	//
	//	// Start the DuoJi task
	//	//
	//	lRetVal = osi_TaskCreate( DuoJiTask, \
	//			(const signed char*)"DuoJi Task", \
	//			OSI_STACK_SIZE, NULL, 1, NULL );
	//	if(lRetVal < 0)
	//	{
	//		ERR_PRINT(lRetVal);
	//		LOOP_FOREVER();
	//	}

	//
	// Start the task scheduler
	//

	//    while(1)
	//    {
	//    	temp = TMP006DrvGetTemp(&g_temp);
	//    	BMA222Read(g_cAccX, g_cAccY, g_cAccZ);
	//    	UART_PRINT("UART - Temp is %.2f F\n\r AccX=%d,AccY=%d,AccZ=%d\n\r",  g_temp, g_cAccX, g_cAccY, g_cAccZ);
	//    }


	osi_start();

}
