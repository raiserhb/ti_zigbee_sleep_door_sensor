#include "zcl_door_control.h"
#include "zcl_SmartDoor.h"
#include "zcl.h"
#include "ZDObject.h"
#include "zcl_device_info.h"

extern byte zclSmartDoor_TaskID;
static afAddrType_t zcl_Coord_nwkAddr;
static uint8 zcl_SeqNum = 0;
static uint8 zcl_factoryReset = 0;
extern uint8 zcl_Led_Blink;
extern uint8 zcl_Flag;
extern devStates_t zclDevice_NwkState;

void zclSwitchControlInit(void)
{
  zclRESETInit();
  
  P1SEL &= ~BV(0); //RED
  P1DIR |= BV(0);

  LED_RED_ON();
  
  ///end init for relay control////////
  osal_start_timerEx(zclSmartDoor_TaskID, 
                     ZCL_DEVICE_INIT_DONE_EVENT,
                     1000);
  
  zcl_Coord_nwkAddr.addrMode = afAddr16Bit;
  zcl_Coord_nwkAddr.addr.shortAddr = 0x0000;
  zcl_Coord_nwkAddr.endPoint = 0xF0;
}

void zclSwitchControlTouchPanelEnable(void)/*门磁检测*/
{
  P0SEL &= ~BV(1);
  P0DIR &= ~BV(1);
  P0INP |= BV(1);
  P0IEN |= BV(1);
  
  PICTL |= (0x1 << 0); //P0 down touch
  IEN1 |= (0x1 << 5);//p0 port
  
  P0IFG = 0;
  P0IF = 0;
  //  //P0_4 = 0;
}

void zclRESETInit(void)/*复位*/
{
  P1SEL &= ~BV(0);
  P1DIR &= ~BV(0);
  P2INP &= ~BV(5);
  P1INP &= ~BV(0);
  P1IEN |= BV(0);
  
  PICTL |= (0x1 << 1); //P0 down touch
  IEN2 |= (0x1 << 4);//p0 port
  
  P1IFG = 0;
  P1IF = 0; 
}

void zclSwitchReportEvent(uint16 endpoint, uint8 *zclOnOffSwitch_OnOff)
{
  zclReportCmd_t *pReportCmd;
  
  pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
  if ( pReportCmd != NULL )
  {
    pReportCmd->numAttr = 1;
    pReportCmd->attrList[0].attrID = ATTRID_SS_IAS_ZONE_STATUS;   //0x0002
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_BITMAP16;
    pReportCmd->attrList[0].attrData = (void *)(zclOnOffSwitch_OnOff);
    
    if( ZSuccess != zcl_SendReportCmd( endpoint, 
                      &zcl_Coord_nwkAddr,
                      ZCL_CLUSTER_ID_SS_IAS_ZONE,
                      pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, 
                      zcl_SeqNum++ ))
    {
    }
    
    osal_mem_free( pReportCmd );
  }
}

void zcldointerruptevents(void)
{

}
void zclSwitch0TouchAble(void)
{
  zcl_Flag = 0;
  P0IFG = 0;
  P0IF = 0;
  P1IFG = 0;
  P1IF = 0;
  P0IEN |= BV(1);
  P0IEN |= BV(7);
  P1IEN |= BV(0);
}

/*复位处理函数*/
void zclSwitch0TouchPanelEvent(void)
{
  if (P1_0 == 0 ) 
  {
    if (zcl_factoryReset > ZCL_FACTORY_RESET_TIMECOUNT) 
    {
      
      osal_start_timerEx(zclSmartDoor_TaskID, 
                         ZCL_DEVICE_RESET_EVENT,
                         ZCL_FACTORY_RESET_HINT);
      zcl_factoryReset = 0;
      zcl_Led_Blink = 0;
    }
    else
    {
      LED_RED_ON();
      zcl_factoryReset++;
      osal_start_timerEx(zclSmartDoor_TaskID, 
                         ZCL_DEVICE_SWITCH_RESET_EVENT,
                         500);
      
    }
  }
  else 
  {
    LED_RED_OFF();
    zcl_factoryReset = 0;
    P0IEN |= BV(1);
    P1IEN |= BV(0);
  }
  
}

HAL_ISR_FUNCTION(zclSwitchControlPort0Isr, P0INT_VECTOR)
{
  HAL_ENTER_ISR();

  if ((P0IFG & 0x02) && (zcl_Flag == 0))  //P0_1   CTIN
  {
    P0IEN &= ~BV(1);
    osal_start_timerEx(zclSmartDoor_TaskID, 
                       ZCL_DEVICE_SWITCH0_TOUCHPANEL_EVENT,
                       1000);

  }
  
  P0IFG = 0;
  P0IF = 0;
  
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}

HAL_ISR_FUNCTION(zclSwitchControlPort1Isr, P1INT_VECTOR)
{
  HAL_ENTER_ISR();
  
  if (P1IFG & 0x01)  //P1_0  RET
  {
      P1IEN &= ~BV(0);
      zcl_factoryReset = 0;
      osal_start_timerEx(zclSmartDoor_TaskID, 
                         ZCL_DEVICE_RESET_1_EVENT,
                         100);
  }
  
  P1IFG = 0;
  P1IF = 0;
  
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}