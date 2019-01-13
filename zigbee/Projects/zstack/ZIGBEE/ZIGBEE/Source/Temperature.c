/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).

**************************************************************************************************/

#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include <string.h>
#include <stdio.h>
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
#include "math.h"


#define DATA_PIN P0_4            //定义P0.4口为继电器的控制端
// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[ENDDEVICE_MAX_CLUSTERS] =
{
  ENDDEVIC1_MSG //Modified by user,区分每个终端的簇列表，第x个终端，则更改为ENDDEVICx_MSG
}; 
const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,             
  SAMPLEAPP_PROFID,              
  SAMPLEAPP_DEVICEID,             
  SAMPLEAPP_FLAGS,                
  SAMPLEAPP_DEVICE_VERSION,       
  ENDDEVICE_MAX_CLUSTERS,          
  (cId_t *)SampleApp_ClusterList,  
  ENDDEVICE_MAX_CLUSTERS,         
  (cId_t *)SampleApp_ClusterList   
};

endPointDesc_t SampleApp_epDesc;


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;  
                          
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  

afAddrType_t SampleApp_P2P_DstAddr;      //点播地址，即终端向协调器发送数据地址
uint8 ok;
float  ADC;
uint8 temperature;
uint8 str_t[4];
unsigned short  NUM;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void spi_master(void);
uint8 SampleApp_Send_PERIDOIC_CMD(void);     //定时串口发送函数
void ADCinital(void);
int get_temperature_Message(void);
void delayMS(unsigned int ms);
/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
 
  ADCinital();
  osal_start_timerEx( SampleApp_TaskID, PERIDOIC_CMD, PERIODIC_CMD_TIME );
  
   P0SEL &= ~0x10;               //设置P0.4口为普通IO
  P0DIR |= 0x10;                //设置P0.4口为输出,控制继电器
  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播地址初始化
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器，协调器地址固定为0X0000，因此无需改动


  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;
 
  afRegister( &SampleApp_epDesc );
 
  RegisterForKeys( SampleApp_TaskID );

  
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
 * @return  none  osal_start_timerEx( SampleApp_TaskID, PERIDOIC_CMD, PERIODIC_CMD_TIME );
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
         
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( 1 )
          {
           //只有终端设备开启，2秒温度换算任务
          // osal_start_timerEx( SampleApp_TaskID, PERIDOIC_CMD, PERIODIC_CMD_TIME );
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

  if ( events & PERIDOIC_CMD )//2S定时时间到，调用函数SampleApp_Send_PERIDOIC_CMD计算温度值
    {
      SampleApp_Send_PERIDOIC_CMD();
      osal_start_timerEx( SampleApp_TaskID, PERIDOIC_CMD, PERIODIC_CMD_TIME );
   //启动定时事件，周期为PERIODIC_CMD_TIME
    }
  return 0;
}

 
/*********************************************************************
*********************************************************************/
uint8 SampleApp_Send_PERIDOIC_CMD(void)//计算温度值
{
  if(1)            
    {                                   
             get_temperature_Message();
             str_t[0] = '1' ;//终端1温度编号
             str_t[1] = (uint8)temperature;   
             if(temperature >= 35)
             {
               str_t[2] = 1;
               DATA_PIN = 0;        //继电器吸合
             } 
             else 
             {
               str_t[2] = 0;
               if (temperature <= 25)
               {
                 DATA_PIN = 1;        //继电器断开
               }
             }

             //sprintf(&str_t[2], "%3.2f", RTD_Temperature);//将温度浮点数转化为对应字符串
             
             AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,//无线发送信息到协调器
                                 ENDDEVIC1_MSG, 
                                 3,            //发送数据长度,根据用户数据长度改变
                                 str_t,
                                 &SampleApp_TransID,
                                 AF_DISCV_ROUTE,AF_DEFAULT_RADIUS);
 
    }
   return 0;  
}
void ADCinital(void)
{
  ADCH&=0X00;//清EOC标志
  P0DIR &= ~0x40;
  //将 P1.1 定义为外设功能
  P0SEL |= 0x40;
  ADCCON3=0XB6;//单次转换，参考电压为电源电压，对P0.6采样，精度12位            
  ADCCON1=0X30;//停止AD
  ADCCON1|=0X40;//开始AD
}
int get_temperature_Message()
{
 
  
  
   while ( !ADCIF ) ;
        
   NUM = ADCL;
   NUM |= ((uint16)ADCH) << 8 ;//这里注意一下
   NUM >>= 4;
  
  ADCinital();
  if (NUM&0x8000) NUM=0;
  if (NUM>4000) NUM=0;
  ADC=(float)((float)NUM*3.30/2048);  
  temperature = ADC*10/33*150-50;
}
void delayMS(unsigned int ms)
{
  unsigned int i;
  while(ms != 0)
  {  
   MicroWait(1000);  
    ms--;  
  }     
}