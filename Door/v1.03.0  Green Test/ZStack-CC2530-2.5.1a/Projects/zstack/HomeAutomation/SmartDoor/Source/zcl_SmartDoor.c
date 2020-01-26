/**************************************************************************************************
Filename:       zcl_sampleSwitch.c
Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
Revision:       $Revision: 19453 $


Description:    Zigbee Cluster Library - sample device application.


Copyright 2006-2009 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
This device will be like a Switch device.  This application is not
intended to be a Switch device, but will use the device description
to implement this sample code.
*********************************************************************/

/*********************************************************************
* INCLUDES
*/
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#include "zcl_SmartDoor.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "zcl_door_control.h"
#include "zcl_device_info.h"


/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/
/*********************************************************************
* TYPEDEFS
*/



#define JOINNWKCYCLETIME_20S    0
#define JOINNWKCYCLETIME_1MIN   1
#define JOINNWKCYCLETIME_10MIN  2
#define JOINNWKCYCLETIME_30MIN  3
#define JOINNWKCYCLETIME_1H     4
#define JOINNWKCYCLETIME_2H     5

#define JOINNWKCYCLEEVENTTIME_20S   (20000+(osal_rand() & 0x00FF))
#define JOINNWKCYCLEEVENTTIME_60S   (60000+(osal_rand() & 0x00FF))

#define JOINNWKCYCLE_ACCOUNT 4

/*********************************************************************
* GLOBAL VARIABLES
*/
byte zclSmartDoor_TaskID;
devStates_t zclDevice_NwkState;
uint8 zclDevice_RejoinCounter = 0;
int8 zclDevice_JoinNetworkOk = 12;
int8 zclDevice_InitDone = 0;
uint8 zcl_Led_Blink = 0;
uint8 zcl_Flag = 0;//0表示当前无门磁变化 1表示关门 2表示开门
uint8 zcl_onoffFlag = 1; //1表示检测到门磁开关 0表示正在执行开或关信息上报

uint8 AppDEviceJoinNwkState = 0;
uint8 u8_AppDEviceJoinNwkTimeComing = 0;
uint16 u16_AppDeviceJoinNwkCycleTimeCount = 0;
uint8 u8_ZDApp_StopJoiningCycleFlag = 0; //0:star  1:stop

extern uint8 g_u8HeartBeatCount;//心跳包计数

uint8 g_u8sendDeviceInfoFlag = 0;//0x00默认发送；0x01收到服务器缺少设备信息的回执
uint8 ReJoinNetFlagInFlash[1] = {0xFF};//0x01:表示复位加入网络；0x02:表示断电重启
/*********************************************************************
* GLOBAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
//static afAddrType_t zclSmartDoor_DstAddr;
uint8 g_RestCount = 0;
/*********************************************************************
* LOCAL FUNCTIONS
*/
static void zclSmartDoor_HandleKeys( byte shift, byte keys );
static void zclSmartDoor_BasicResetCB( void );
static void zclSmartDoor_IdentifyCB( zclIdentify_t *pCmd );
static void zclSmartDoor_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSmartDoor0_OnOffCB(uint8 cmd );
static void zclSmartDoor_OnOffCB(uint8 s, uint8 cmd );
static void zclSmartDoor_ProcessIdentifyTimeChange( void );

// Functions to process ZCL Foundation incoming Command/Response messages 
static void zclSmartDoor_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSmartDoor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSmartDoor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSmartDoor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSmartDoor_ProcessInDiscRspCmd( zclIncomingMsg_t *pInMsg );
#endif

/*********************************************************************
* ZCL General Profile Callback table
*/
static zclGeneral_AppCallbacks_t zclSmartDoor0_CmdCallbacks =
{
  zclSmartDoor_BasicResetCB,              // Basic Cluster Reset command
  zclSmartDoor_IdentifyCB,                // Identify command  
  zclSmartDoor_IdentifyQueryRspCB,        // Identify Query Response command
  zclSmartDoor0_OnOffCB,                   // On/Off cluster command
  NULL,                                     // Level Control Move to Level command
  NULL,                                     // Level Control Move command
  NULL,                                     // Level Control Step command
  NULL,                                     // Group Response commands
  NULL,                                     // Scene Store Request command
  NULL,                                     // Scene Recall Request command
  NULL,                                     // Scene Response command
  NULL,                                     // Alarm (Response) command
  NULL,                                     // RSSI Location commands
  NULL,                                     // RSSI Location Response commands
};

/*********************************************************************
* @fn          zclSmartDoor_Init
*
* @brief       Initialization function for the zclGeneral layer.
*
* @param       none
*
* @return      none
*/
void zclSmartDoor_Init( byte task_id )
{
  zclSmartDoor_TaskID = task_id;
  zclDevice_NwkState = DEV_INIT;
  zclDevice_InitDone = 0;
  zclDevice_RejoinCounter = 0;
  
  
  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSmartDoor_SimpleDesc );
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMRTDOOR_ENDPOINT, &zclSmartDoor0_CmdCallbacks );
  
  // Register the application's attribute list
  zcl_registerAttrList( SAMRTDOOR_ENDPOINT, SMARTDOOR_MAX_ATTRIBUTES, zclSmartDoor_Attrs );
  
  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSmartDoor_TaskID );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSmartDoor_TaskID );
  
  zclDeviceInfoInit();
  zclSwitchControlInit();
}

/*********************************************************************
* @fn          zclSample_event_loop
*
* @brief       Event Loop Processor for zclGeneral.
*
* @param       none
*
* @return      none
*/
uint8 DeviceRehoinEventFlag = 0;
uint16 zclSmartDoor_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSmartDoor_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
      case ZCL_INCOMING_MSG:
        // Incoming ZCL Foundation command/response messages
        zclSmartDoor_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
        break;
        
      case KEY_CHANGE:
        zclSmartDoor_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;
      case ZDO_STATE_CHANGE:
        zclDevice_NwkState = (devStates_t)(MSGpkt->hdr.status);
        if (zclDevice_NwkState == DEV_END_DEVICE ) {
          osal_start_timerEx(zclSmartDoor_TaskID, 
                             ZCL_DEVICE_INFO_SEND_EVENT,
                             ZCL_DEVICE_SEND_INIF_TIME_1S);
          osal_start_timerEx(zclSmartDoor_TaskID, 
                             ZCL_DEVICE_JOIN_NETWORK_OK_EVENT,
                             200);
          
          zclDevice_JoinNetworkOk = 12;//LED
          zclDevice_RejoinCounter= 0;
          u8_ZDApp_StopJoiningCycleFlag = 0;
          u8_AppDEviceJoinNwkTimeComing = 0;
          DeviceRehoinEventFlag = 0;
          AppDEviceJoinNwkState = JOINNWKCYCLETIME_20S;
          u16_AppDeviceJoinNwkCycleTimeCount= 0;
          
          pwrmgr_attribute.pwrmgr_device =PWRMGR_BATTERY;
          pwrmgr_attribute.pwrmgr_task_state = 0; 
        } 
        else if( zclDevice_NwkState == DEV_NWK_ORPHAN )
        {
          
        }
        else if (zclDevice_NwkState == DEV_NWK_DISC) 
        {
          if(u8_ZDApp_StopJoiningCycleFlag == 0)
          {
            zclDevice_RejoinCounter++;
            AppDevNwkDiscCyCle();
          }
        }
        break;
      default:
        break;
      }
      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }
    
    // return unprocessed events
    // return (events ^ SYS_EVENT_MSG);
  }
  
  if (events & ZCL_DEVICE_REJOIN_EVENT) {
    if(zclDevice_NwkState != DEV_END_DEVICE)
    {
      AppDEviceJoinNwk(); 
    }
  }
  
  if ( events & SMARTDOOR_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSmartDoor_IdentifyTime > 0 )
      zclSmartDoor_IdentifyTime--;
    zclSmartDoor_ProcessIdentifyTimeChange();
    
    //  return ( events ^ SmartDoor_IDENTIFY_TIMEOUT_EVT );
  }
  
  if (events & ZCL_DEVICE_INIT_DONE_EVENT) 
  {
    zclDevice_InitDone = 1;
    LED_RED_OFF();
    zclSwitchControlTouchPanelEnable(); 
    //   return (events ^ ZCL_DEVICE_INIT_DONE_EVENT);
  }
  
  if (events & ZCL_DEVICE_INFO_SEND_EVENT) 
  {
    zclSendDeviceInfo();
    //     return (events ^ ZCL_DEVICE_INFO_SEND_EVENT);
  }
  
  if (events & ZCL_DEVICE_HEARTBEAT_EVENT) 
  {
    g_u8HeartBeatCount++;
    if(g_u8HeartBeatCount >= 2)
    {
      g_u8HeartBeatCount=0;
      zclSendHeartbeat();
    }
    osal_start_timerEx(zclSmartDoor_TaskID, 
                       ZCL_DEVICE_HEARTBEAT_EVENT,
                       ZCL_HEARTBEAT_PERIOD);
    //    return (events ^ ZCL_DEVICE_HEARTBEAT_EVENT);
  }
  
  if (events & ZCL_DEVICE_JOIN_NETWORK_OK_EVENT) 
  {
    if (zclDevice_JoinNetworkOk-- % 2) {
      LED_RED_OFF(); 
    } else {
      LED_RED_ON();
    }
    
    if (zclDevice_JoinNetworkOk <= 0) 
    {
      LED_RED_OFF();
    } else {
      osal_start_timerEx(zclSmartDoor_TaskID, 
                         ZCL_DEVICE_JOIN_NETWORK_OK_EVENT,
                         500);
    }
    //return events ^ IAS_ZONE_IR_JOIN_OK_EVENT;
  }

  if (events & ZCL_DEVICE_SWITCH0_TOUCHPANEL_EVENT) //门磁
  {
    if( zcl_Flag == 0)
    {
      if( P0_1 == 0 )/*反转上升沿和下降沿*/
      { 
        PICTL &= ~(0x1 << 0);
        zcl_Flag = 1;//关门低电平
      }
      else if( P0_1 == 1 )
      {
        PICTL |= (0x1 << 0);
        zcl_Flag = 2;//开门高电平
      }
      
      if( zcl_Flag & 1 )//关门
      {
        if( (zclSmartDoor_OnOff == SWITCH_ON) && (zcl_onoffFlag == 1) )
        {
          zcl_onoffFlag = 0;
          LED_RED_ON();/*LED灯闪一次*/
          zclSmartDoor_OnOff = SWITCH_OFF;
          
          zclSwitchReportEvent(SAMRTDOOR_ENDPOINT, &zclSmartDoor_OnOff); 
          osal_start_timerEx(zclSmartDoor_TaskID, 
                             ZCL_DEVICE_LEN_RED_OFF_EVENT,
                             100);
          
        }
        osal_start_timerEx(zclSmartDoor_TaskID, 
                           ZCL_DEVICE_SWITCH0_ABLE_EVENT,
                           200);/*开中断*/
        
      }
      
      if( zcl_Flag & 2 )//开门
      {
        if( (zclSmartDoor_OnOff == SWITCH_OFF) && (zcl_onoffFlag == 1) )
        {
          zcl_onoffFlag = 0;
          LED_RED_ON();
          zclSmartDoor_OnOff = SWITCH_ON;
          
          zclSwitchReportEvent(SAMRTDOOR_ENDPOINT, &zclSmartDoor_OnOff); 
          osal_start_timerEx(zclSmartDoor_TaskID, 
                             ZCL_DEVICE_LEN_RED_OFF_EVENT,
                             100);
          
        }
        osal_start_timerEx(zclSmartDoor_TaskID, 
                           ZCL_DEVICE_SWITCH0_ABLE_EVENT,
                           200);
        
      }
    }
  }
  
  if (events & ZCL_DEVICE_SWITCH0_ABLE_EVENT) 
  {
    zcl_Flag = 0;
    zcl_onoffFlag = 1;
    zclSwitch0TouchAble();
    //   return (events ^ ZCL_DEVICE_SWITCH0_ABLE_EVENT);
  }
  
  if (events & ZCL_DEVICE_RESET_EVENT) 
  {
    if(zcl_Led_Blink > 30)
    {
      zclFactoryReset(TRUE);
    }
    else
    {
      if(zcl_Led_Blink % 2 == 0)
      {
        LED_RED_OFF();
      } else {
        LED_RED_ON();
      }
      zcl_Led_Blink++;
      osal_start_timerEx(zclSmartDoor_TaskID, 
                         ZCL_DEVICE_RESET_EVENT,
                         200);
    }
    //    return (events ^ ZCL_DEVICE_RESET_EVENT);
  }
  
  if(events & ZCL_DEVICE_SWITCH_RESET_EVENT)
  {
    zclSwitch0TouchPanelEvent();
    //   return (events ^ ZCL_DEVICE_SWITCH_RESET_EVENT);
  }
  
  if(events & ZCL_DEVICE_RESET_1_EVENT)
  {
    if(P1_0 == 0)
    {
      g_RestCount++;
      if(g_RestCount>=3)
      {
        osal_start_timerEx(zclSmartDoor_TaskID, 
                           ZCL_DEVICE_SWITCH_RESET_EVENT,
                           100);
      }
      else
      {
        osal_start_timerEx(zclSmartDoor_TaskID, 
                           ZCL_DEVICE_RESET_1_EVENT,
                           100);
      }
    }
    else
    {
      P1IEN |= BV(0);
      g_RestCount=0;
    }
  }
  if(events & ZCL_DEVICE_LEN_RED_OFF_EVENT)
  {
    LED_RED_OFF();
    //return (events ^ ZCL_DEVICE_LEN_RED_OFF_EVENT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
* @fn      zclSmartDoor_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_4
*                 HAL_KEY_SW_3
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void zclSmartDoor_HandleKeys( byte shift, byte keys )
{
  // zAddrType_t dstAddr;
  
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_2 )
  {
  }
  
  if ( keys & HAL_KEY_SW_3 )
  {
  }
  
  if ( keys & HAL_KEY_SW_4 )
  {
  }
}

/*********************************************************************
* @fn      zclSmartDoor_ProcessIdentifyTimeChange
*
* @brief   Called to process any change to the IdentifyTime attribute.
*
* @param   none
*
* @return  none
*/
static void zclSmartDoor_ProcessIdentifyTimeChange( void )
{
  if ( zclSmartDoor_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSmartDoor_TaskID, SMARTDOOR_IDENTIFY_TIMEOUT_EVT, 1000 );
  }
  else
  {
    osal_stop_timerEx( zclSmartDoor_TaskID, SMARTDOOR_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
* @fn      zclSmartDoor_BasicResetCB
*
* @brief   Callback from the ZCL General Cluster Library
*          to set all the Basic Cluster attributes to default values.
*
* @param   none
*
* @return  none
*/
static void zclSmartDoor_BasicResetCB( void )
{
  // Reset all attributes to default values
}

/*********************************************************************
* @fn      zclSmartDoor_IdentifyCB
*
* @brief   Callback from the ZCL General Cluster Library when
*          it received an Identity Command for this application.
*
* @param   srcAddr - source address and endpoint of the response message
* @param   identifyTime - the number of seconds to identify yourself
*
* @return  none
*/
static void zclSmartDoor_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSmartDoor_IdentifyTime = pCmd->identifyTime;
  zclSmartDoor_ProcessIdentifyTimeChange();
}

/*********************************************************************
* @fn      zclSmartDoor_IdentifyQueryRspCB
*
* @brief   Callback from the ZCL General Cluster Library when
*          it received an Identity Query Response Command for this application.
*
* @param   srcAddr - requestor's address
* @param   timeout - number of seconds to identify yourself (valid for query response)
*
* @return  none
*/
static void zclSmartDoor_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  // Query Response (with timeout value)
  (void)pRsp;
}


static void zclSmartDoor0_OnOffCB(uint8 cmd )
{
  zclSmartDoor_OnOffCB(0, cmd);
}

/*********************************************************************
* @fn      zclSmartDoor_OnOffCB
*
* @brief   Callback from the ZCL General Cluster Library when
*          it received an On/Off Command for this application.
*
* @param   cmd - COMMAND_ON, COMMAND_OFF or COMMAND_TOGGLE
*
* @return  none
*/
static void zclSmartDoor_OnOffCB(uint8 switch0, uint8 cmd )
{
  
}


/****************************************************************************** 
* 
*  Functions for processing ZCL Foundation incoming Command/Response messages
*
*****************************************************************************/

/*********************************************************************
* @fn      zclSmartDoor_ProcessIncomingMsg
*
* @brief   Process ZCL Foundation incoming message
*
* @param   pInMsg - pointer to the received message
*
* @return  none
*/
static void zclSmartDoor_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
  case ZCL_CMD_READ_RSP:
    zclSmartDoor_ProcessInReadRspCmd( pInMsg );
    break;
#endif
#ifdef ZCL_WRITE    
  case ZCL_CMD_WRITE_RSP:
    zclSmartDoor_ProcessInWriteRspCmd( pInMsg );
    break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
  case ZCL_CMD_CONFIG_REPORT:
    //zclSmartDoor_ProcessInConfigReportCmd( pInMsg );
    break;
    
  case ZCL_CMD_CONFIG_REPORT_RSP:
    //zclSmartDoor_ProcessInConfigReportRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_READ_REPORT_CFG:
    //zclSmartDoor_ProcessInReadReportCfgCmd( pInMsg );
    break;
    
  case ZCL_CMD_READ_REPORT_CFG_RSP:
    //zclSmartDoor_ProcessInReadReportCfgRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_REPORT:
    //zclSmartDoor_ProcessInReportCmd( pInMsg );
    break;
#endif   
  case ZCL_CMD_DEFAULT_RSP:
    zclSmartDoor_ProcessInDefaultRspCmd( pInMsg );
    break;
#ifdef ZCL_DISCOVER     
  case ZCL_CMD_DISCOVER_RSP:
    zclSmartDoor_ProcessInDiscRspCmd( pInMsg );
    break;
#endif  
  default:
    break;
  }
  
  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
* @fn      zclSmartDoor_ProcessInReadRspCmd
*
* @brief   Process the "Profile" Read Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSmartDoor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;
  
  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes 
    // attempt and, for each successfull request, the value of the requested 
    // attribute
  }
  
  return TRUE; 
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
* @fn      zclSmartDoor_ProcessInWriteRspCmd
*
* @brief   Process the "Profile" Write Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSmartDoor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;
  
  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < writeRspCmd->numAttr; i++)
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }
  
  return TRUE; 
}
#endif // ZCL_WRITE

/*********************************************************************
* @fn      zclSmartDoor_ProcessInDefaultRspCmd
*
* @brief   Process the "Profile" Default Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSmartDoor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  
  // Device is notified of the Default Response command.
  (void)pInMsg;
  
  return TRUE; 
}

#ifdef ZCL_DISCOVER
/*********************************************************************
* @fn      zclSmartDoor_ProcessInDiscRspCmd
*
* @brief   Process the "Profile" Discover Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSmartDoor_ProcessInDiscRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverRspCmd_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return TRUE;
}
#endif // ZCL_DISCOVER


/****************************************************************************
****************************************************************************/

void AppDEviceJoinNwk()
{
  if(u8_AppDEviceJoinNwkTimeComing == 1)
  {
    u8_AppDEviceJoinNwkTimeComing = 0;
    if(zclDevice_NwkState != DEV_END_DEVICE)
    {
      pwrmgr_attribute.pwrmgr_device =PWRMGR_ALWAYS_ON;
      pwrmgr_attribute.pwrmgr_task_state = 1;
    }
    u8_ZDApp_StopJoiningCycleFlag = 0;
    
//    if((_NIB.nwkLogicalChannel >= 0x0B)&&(_NIB.nwkLogicalChannel <= 0x1A))//已经组网
//    {
//      ZDOInitDevice(0);
//    }
    
      if(FALSE == ZDApp_StartJoiningCycle())
      {
        ZDApp_StartJoiningCycle();    
      }
    
  }
  else
  {
    switch(AppDEviceJoinNwkState)
    {
    case JOINNWKCYCLETIME_20S: 
      { 
        u8_AppDEviceJoinNwkTimeComing = 1;
      }
      break;
    case JOINNWKCYCLETIME_1MIN:
      {
        u8_AppDEviceJoinNwkTimeComing = 1;
      }
      break;
    case JOINNWKCYCLETIME_10MIN:
      {
        if((u16_AppDeviceJoinNwkCycleTimeCount++) == 10)
        {
          u8_AppDEviceJoinNwkTimeComing = 1;
          u16_AppDeviceJoinNwkCycleTimeCount = 0;
        }
      }
      break;
    case JOINNWKCYCLETIME_30MIN:
      {
        if((u16_AppDeviceJoinNwkCycleTimeCount++) == 30)
        {
          u8_AppDEviceJoinNwkTimeComing = 1;
          u16_AppDeviceJoinNwkCycleTimeCount = 0;
        }
      }
      break;
    case JOINNWKCYCLETIME_1H:
      {
        if((u16_AppDeviceJoinNwkCycleTimeCount++) == 60)
        {
          u8_AppDEviceJoinNwkTimeComing = 1;
          u16_AppDeviceJoinNwkCycleTimeCount = 0;
        }
      }
      break;
    default:
      {
        AppDEviceJoinNwkState = JOINNWKCYCLETIME_20S;
        u8_AppDEviceJoinNwkTimeComing = 1;
        u16_AppDeviceJoinNwkCycleTimeCount = 0;
      }
      break;
      
    }
    
    if(AppDEviceJoinNwkState == JOINNWKCYCLETIME_20S)
    {
      osal_start_timerEx(zclSmartDoor_TaskID, 
                         ZCL_DEVICE_REJOIN_EVENT,
                         JOINNWKCYCLEEVENTTIME_20S);
    }
    else
    {	
      osal_start_timerEx(zclSmartDoor_TaskID, 
                         ZCL_DEVICE_REJOIN_EVENT,
                         JOINNWKCYCLEEVENTTIME_60S);    
    }
  }
}

void AppDevNwkDiscCyCle()
{
  DeviceRehoinEventFlag = 0;
  
  if((_NIB.nwkLogicalChannel >= 0x0B)&&(_NIB.nwkLogicalChannel <= 0x1A))//已经组网
  {
    if (zclDevice_RejoinCounter <= JOINNWKCYCLE_ACCOUNT*3) 
    {  
      if(zclDevice_RejoinCounter%JOINNWKCYCLE_ACCOUNT == 0)
      {
        DeviceRehoinEventFlag = 1;
        AppDEviceJoinNwkState = JOINNWKCYCLETIME_20S;
      }
      
    }
    else if(zclDevice_RejoinCounter >JOINNWKCYCLE_ACCOUNT*3 && zclDevice_RejoinCounter <= JOINNWKCYCLE_ACCOUNT*6)
    {
      if(zclDevice_RejoinCounter%JOINNWKCYCLE_ACCOUNT == 0)
      {
        DeviceRehoinEventFlag = 1;
        AppDEviceJoinNwkState = JOINNWKCYCLETIME_1MIN;
      }
    }
    else if(zclDevice_RejoinCounter >JOINNWKCYCLE_ACCOUNT*6 && zclDevice_RejoinCounter <= JOINNWKCYCLE_ACCOUNT*9)
    {
      if(zclDevice_RejoinCounter%JOINNWKCYCLE_ACCOUNT == 0)
      {
        DeviceRehoinEventFlag = 1;
        AppDEviceJoinNwkState = JOINNWKCYCLETIME_10MIN;
      }
    }
    else if(zclDevice_RejoinCounter >JOINNWKCYCLE_ACCOUNT*9 && zclDevice_RejoinCounter <= JOINNWKCYCLE_ACCOUNT*12)
    {
      if(zclDevice_RejoinCounter%JOINNWKCYCLE_ACCOUNT == 0)
      {
        DeviceRehoinEventFlag = 1;
        AppDEviceJoinNwkState = JOINNWKCYCLETIME_30MIN;
      }
    }
    else if(zclDevice_RejoinCounter >JOINNWKCYCLE_ACCOUNT*12 && zclDevice_RejoinCounter <= JOINNWKCYCLE_ACCOUNT*15)
    {
      if(zclDevice_RejoinCounter%JOINNWKCYCLE_ACCOUNT == 0)
      {
        DeviceRehoinEventFlag = 1;
        AppDEviceJoinNwkState = JOINNWKCYCLETIME_1H;
        zclDevice_RejoinCounter = JOINNWKCYCLE_ACCOUNT*12;
      }
    }
  }
  else//新设备未组网
  {
    if (zclDevice_RejoinCounter <= 24) 
    {
      if(zclDevice_RejoinCounter%4 == 0)
      {
        DeviceRehoinEventFlag = 1;
        AppDEviceJoinNwkState = JOINNWKCYCLETIME_20S;
      }
    }
    else
    {
      pwrmgr_attribute.pwrmgr_device =PWRMGR_BATTERY;
      pwrmgr_attribute.pwrmgr_task_state = 0;
      DeviceRehoinEventFlag = 0; 
      if(FALSE == ZDApp_StopJoiningCycle())
      {
        ZDApp_StopJoiningCycle();
      }
    }
  }
  
  if(DeviceRehoinEventFlag == 1)
  {
    if(FALSE == ZDApp_StopJoiningCycle())
    {
      ZDApp_StopJoiningCycle();
    }
    
    u8_ZDApp_StopJoiningCycleFlag = 1;
    
    DeviceRehoinEventFlag = 0;
    u8_AppDEviceJoinNwkTimeComing = 0;
    
    pwrmgr_attribute.pwrmgr_device =PWRMGR_BATTERY;
    pwrmgr_attribute.pwrmgr_task_state = 0;  
    
    osal_start_timerEx(zclSmartDoor_TaskID, 
                       ZCL_DEVICE_REJOIN_EVENT,
                       500); 
  }
  
}
