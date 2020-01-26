#include "zcl.h"
#include "ZDObject.h"
#include "zcl_device_info.h"
#include "zcl_SmartDoor.h"
#include "OnBoard.h"
#include "ZGlobals.h"
#include "ZComDef.h"

#include "zcl_door_control.h"

extern byte zclSmartDoor_TaskID;
#define ZCL_TASK_ID zclSmartDoor_TaskID

afAddrType_t zclIASZoneMotionIR_Coord_nwkAddr;
static uint16 gDeviceInfoSendStatus = 0;
uint8 g_u8AdcIsBusy = 0;

uint8 softeVision[3] = {0x01,0x08,0x00};//软件版本号 v1.0.0
uint8 hardVision[2] = {0x00,0x00};//硬件北版本号
uint8 g_u8HeartBeatCount = 0;//心跳包计数
extern uint8 g_u8sendDeviceInfoFlag;

void zclDeviceInfoInit(void)
{
  gDeviceInfoSendStatus = 0;
  zclIASZoneMotionIR_Coord_nwkAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclIASZoneMotionIR_Coord_nwkAddr.addr.shortAddr = 0x0000;
  zclIASZoneMotionIR_Coord_nwkAddr.endPoint = 0xF0;
}

static void zclActiveEPRsp(void)
{
  uint8 pBuf[64];
  byte cnt = 0;
  zAddrType_t srcAddr;
  
  srcAddr.addrMode = (afAddrMode_t)Addr16Bit;
  srcAddr.addr.shortAddr = 0x0000;
  
  cnt = afNumEndPoints() - 1;
  afEndPoints( (uint8 *)pBuf, true );
  ZDP_ActiveEPRsp(0x0, &srcAddr, ZDP_SUCCESS,
                  NLME_GetShortAddr(), 
                  cnt, (uint8 *)pBuf, 0);
  
  gDeviceInfoSendStatus++;
  if(g_u8sendDeviceInfoFlag == 0)
  {
    
    osal_start_timerEx(ZCL_TASK_ID, 
                       ZCL_DEVICE_INFO_SEND_EVENT,
                       ZCL_DEVICE_SEND_INFO_TIME_1S);
  }
  else if(g_u8sendDeviceInfoFlag == 1)
  {
    osal_start_timerEx(ZCL_TASK_ID, 
                       ZCL_DEVICE_INFO_SEND_EVENT,
                       ZCL_DEVICE_SEND_INFO_TIME_200MS);
  }
}

static void zclSimpleDescRsp(uint16 endpoint)
{
  zdoIncomingMsg_t *imMsg;
  uint16 nwkAddr = 0;
  
  nwkAddr = NLME_GetShortAddr();
  
  imMsg = (zdoIncomingMsg_t *)osal_msg_allocate( sizeof( zdoIncomingMsg_t ) + 3 );
  imMsg->srcAddr.addrMode = (afAddrMode_t)Addr16Bit;
  imMsg->srcAddr.addr.shortAddr = 0x0000;
  imMsg->TransSeq = (uint8)0x00;
  imMsg->asdu = (byte*)(((byte*)imMsg) + sizeof( zdoIncomingMsg_t ));
  imMsg->asdu[0] = LO_UINT16( nwkAddr );
  imMsg->asdu[1] = HI_UINT16( nwkAddr );
  imMsg->asdu[2] = endpoint;
  
  ZDO_ProcessSimpleDescReq(imMsg);
  osal_msg_deallocate((uint8 *)imMsg);
  
  gDeviceInfoSendStatus++;
  if(g_u8sendDeviceInfoFlag == 0)
  {
    osal_start_timerEx(ZCL_TASK_ID, 
                       ZCL_DEVICE_INFO_SEND_EVENT,
                       ZCL_DEVICE_SEND_INFO_TIME_1S);
  }
  else if(g_u8sendDeviceInfoFlag == 1)
  {
    osal_start_timerEx(ZCL_TASK_ID, 
                       ZCL_DEVICE_INFO_SEND_EVENT,
                       ZCL_DEVICE_SEND_INFO_TIME_200MS);
  }
}

static endPointDesc_t zclHeartbeatDesc = {
  ZCL_HEARTBEAT_ENDPOINT,
  &ZCL_TASK_ID,
  (SimpleDescriptionFormat_t *)&zclSmartDoor_SimpleDesc,
  (afNetworkLatencyReq_t)0
};

static uint8 zclHeartbeatCounter = 0;

afStatus_t zclSendHeartbeat(void)
{
  afStatus_t stat;
  uint8 SendDataBuf[6]={0,0,0,0,0,0};
  
  SendDataBuf[0] = DeviceAppBatteryDetect();
  SendDataBuf[1] = softeVision[0];
  SendDataBuf[2] = softeVision[1];
  SendDataBuf[3] = softeVision[2];
  SendDataBuf[4] = hardVision[0];
  SendDataBuf[5] = hardVision[1];
  
  stat = AF_DataRequest(&zclIASZoneMotionIR_Coord_nwkAddr, 
                        &zclHeartbeatDesc,
                        ZCL_HEARTBEAT_CLUSTERID,
                        sizeof(SendDataBuf),
                        SendDataBuf,
                        (uint8 *)&zclHeartbeatCounter,
                        0,
                        AF_DEFAULT_RADIUS);
  return stat;
}

void zclSendDeviceInfo(void)
{
  switch (gDeviceInfoSendStatus)
  {
  case 0:
    zclActiveEPRsp();
    break;
  case 1:
    zclSimpleDescRsp(SAMRTDOOR_ENDPOINT);
    break;
  default:
    {
      gDeviceInfoSendStatus = 0;
      
      osal_start_timerEx(ZCL_TASK_ID, 
                         ZCL_DEVICE_HEARTBEAT_EVENT,
                         ZCL_HEARTBEAT_PERIOD);
      
      g_u8HeartBeatCount = 1;
      g_u8sendDeviceInfoFlag =0;//信息发送结束
    }
    break;
  }
}


void zclFactoryReset(uint8 DataCleanFlag)
{
  static uint8 hasDoReset = 0;
  if (hasDoReset)
    return;
  
  hasDoReset = 1;
  if(DataCleanFlag)
  {
    zgWriteStartupOptions(ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE);
  }
  SystemResetSoft();
}

/**************************************************************************
*低电量检测低于2.6v报警
*****************************************************************************/
uint8 DeviceAppBatteryDetect()
{
  int16 value= 0,value1 = 0;
  uint8 i = 0;
  uint16 VddValue = 0;
  uint8 u8BatterAlarmValue = 0;
  if(g_u8AdcIsBusy == 0)
  {
    g_u8AdcIsBusy = 1;
    uint8 tmpADDCON3 = ADCCON3;
    for(i = 0;i < 8;i++)
    {
      ADCCON3 = 0x0F;
      while (!(ADCCON1 & 0x80));
      value = (int8)(ADCH)<<8;
      value1 = (int8)(ADCL);
      value1+=value;
      VddValue = value1;
      VddValue += VddValue;
    }
    
    VddValue = VddValue>>3; 
    ADCCON3 = tmpADDCON3;
    VddValue = value1>>8;
    VddValue = VddValue*115*3;
    VddValue = VddValue>>7;
    
    if(VddValue < BETTERYALARMVALUE)
    {
      u8BatterAlarmValue = 1;//报警
    }
    else
    {
      u8BatterAlarmValue = 0;
    }
    g_u8AdcIsBusy = 0;
  }
  return u8BatterAlarmValue;
}