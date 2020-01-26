#ifndef __ZCL_SWITCH_CONTROL_H__
#define __ZCL_SWITCH_CONTROL_H__
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"

#include "zcl_SmartDoor.h"

#include "onboard.h"

#define LED_RED_OFF()      st(P1_0 = 1;)
#define LED_RED_ON()       st(P1_0 = 0;)

extern void zclSwitchControlInit(void);
extern void zclSwitch0TouchAble(void);
extern void zclSwitchControlTouchPanelEnable(void);
extern void zclSwitch0TouchPanelEvent(void);
extern void zclSendDeviceInfo(void);
extern void zclSwitchReportEvent(uint16 endpoint, uint8 *zclOnOffSwitch_OnOff);
extern void zclDoorOnOff(void);
extern void zclRESETInit(void);
extern void zcldointerruptevents(void);
#endif