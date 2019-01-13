/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "stdlib.h"
#include "string.h"
//#include "DHT11.h"
#include <stdio.h>

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
typedef unsigned int  uint;
uint8 count=0;
uint8 cmd;
uint8 send[10];
uint8 linked = 0;
unsigned char a[80];


void Coord_UartInit(void);
void Delay_MS(uint x);
void Delay_MS(uint x)
{
  uint i,j;
  for(i=0;i<x;i++)
  MicroWait(1000);
}
// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[ENDDEVICE_MAX_CLUSTERS] =
{
  ENDDEVIC1_MSG //Modified by user,区分每个终端的簇列表，第x个终端，则更改为ENDDEVICx_MSG
}; 
const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr; //广播
afAddrType_t SampleApp_Flash_DstAddr;    //组播
afAddrType_t SampleApp_P2P_DstAddr;      //点播

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint8 afRxData[4]={0};
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
//void Delay_10us(void);
//void Delay_ms(uint16 Time);
void callback(uint8 port, uint8 event);
void Delay_ms(unsigned int ms);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  Coord_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
 
 
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播 
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
  
  
  Delay_ms(500);
  HalUARTWrite(0, "AT+CIPMUX=1\r\n", 13);
  Delay_ms(200);
  HalUARTWrite(0, "AT+CIPSERVER=1,8080\r\n", 21);
  Delay_ms(200);
 
#if defined ( LCD_SUPPORTED )
  
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{ 
  
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter
 
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD) )
          {
            
          }
          break;
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Discard unknown events4即
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
 (void)shift;  // Intentionally unreferenced parameter
 if ( keys & HAL_KEY_SW_6 )
  {
   
  }

  if ( keys & HAL_KEY_SW_1 )
  {
  } 
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;

  uint8 i;
  for (i=0;i<3;i++)
    afRxData[i] = pkt->cmd.Data[i];
  HalUARTWrite(0, "AT+CIPSEND=0,3\r\n", 15);
  Delay_MS(10);
  HalUARTWrite(0,  afRxData, 3); //输出接收到的数据	
  
}
/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */



/*********************************************************************
 * @fn      SampleApp_Send_P2P_Message
 *
 * @brief   point to point.
 *
 * @param   none
 *
 * @return  none
 */

//以下为协调器显示wifi模块的调试信息，可用可不用
/*********************************************************************
*********************************************************************/
void callback(uint8 port, uint8 event)
{
 unsigned char a1[17];  //将接收的数据分为四个16字节大小的数组，在lcd四行显示
 unsigned char a2[17]; unsigned char a3[17];
 unsigned char a4[17];
 unsigned char a5[]="  +IPD"; //判断是否为wifi模块收到数据并串口发送给zigbee
 unsigned char a6[]="  +IPD,0,4:"; //判断是否为wifi模块收到数据并串口发送给zigbee
 unsigned char a7[]="Link"; //判断是否为wifi模块收到数据并串口发送给zigbee
 unsigned char a8[]="Unlink"; //判断是否为wifi模块收到数据并串口发送给zigbee
 unsigned char i=0;           //循环标志
 unsigned char j=0;           //循环标志
 unsigned char k=0;           //实际有效数据起始标志位
 unsigned char temp[50];      //提取出的有效数据暂存数组
 uint8 n=0;
 if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))) 
 {  //必须要判断event，此处确保字执行串口回调函数之前，dma已经把串口缓存区的数据完全读到
  HalUARTRead(0, a, 80); //从dma缓冲区读取数据
 
   for(i=0;i<80;i++)      //将数据内的换行回车以空格代替，便于显示
   {
     if(a[i]==0x0A||a[i]==0x0D) a[i]=' ';
   }
   if((strncmp(a8,a,6))==0)
   {
      Delay_MS(100);
       linked = 0;
      
   }
   else  if((strncmp(a7,a,4))==0)
   {
       Delay_MS(100);
       linked = 1;
   }
   else if ((strncmp(a6,a,7))==0) //比较数据头，拦截一种wifi转发给zigbee的控制指令
   {   
       for(j=0;j<4;j++)
       {
         send[j] = a[11+j];
       }
       AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       ENDDEVIC1_MSG,
                       4,
                       send,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS );  
   
   }   
   
   osal_memset(a, 0, 80); //清空a，为下次数据接收准备
 } 
}
/***************************************************************************************************/
void Coord_UartInit ()
{
  halUARTCfg_t uartConfig;

  /* Initialize APP ID */
  //App_TaskID = 0;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = MT_UART_DEFAULT_OVERFLOW;
  uartConfig.flowControlThreshold = MT_UART_DEFAULT_THRESHOLD;
  uartConfig.rx.maxBufSize        = MT_UART_DEFAULT_MAX_RX_BUFF;
  uartConfig.tx.maxBufSize        = MT_UART_DEFAULT_MAX_TX_BUFF;
  uartConfig.idleTimeout          = MT_UART_DEFAULT_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  uartConfig.callBackFunc         = callback;
#elif defined (ZAPP_P1) || defined (ZAPP_P2)
  uartConfig.callBackFunc         = callback;
#else
  uartConfig.callBackFunc         = callback;
#endif

  /* Start UART */
#if defined (MT_UART_DEFAULT_PORT)
  HalUARTOpen (MT_UART_DEFAULT_PORT, &uartConfig);
#else
  /* Silence IAR compiler warning */
  (void)uartConfig;
#endif

  /* Initialize for ZApp */
#if defined (ZAPP_P1) || defined (ZAPP_P2)
  /* Default max bytes that ZAPP can take */
  MT_UartMaxZAppBufLen  = 1;
  MT_UartZAppRxStatus   = MT_UART_ZAPP_RX_READY;
#endif

}

void Delay_ms(unsigned int ms)
{
  unsigned int i;
  while(ms != 0)
  {  
   MicroWait(1000);  
    ms--;  
  }     
}
