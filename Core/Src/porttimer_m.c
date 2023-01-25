#include "port.h"
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0

/* ----------------------- User defenitions ---------------------------------*/
TIM_HandleTypeDef *timM;
static uint16_t timeout = 0;
extern volatile uint16_t counterM;

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit( USHORT usTimeOut50us, void *dHTIM )
{
	timM = (TIM_HandleTypeDef *)dHTIM;
	timeout = usTimeOut50us;
    return TRUE;
}

void vMBMasterPortTimersT35Enable()
{
    vMBMasterSetCurTimerMode(MB_TMODE_T35);
	counterM=0;
	HAL_TIM_Base_Start_IT(timM);
}

void vMBMasterPortTimersConvertDelayEnable()
{
	vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);

}

void vMBMasterPortTimersRespondTimeoutEnable()
{
	vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);

}

void vMBMasterPortTimersDisable()
{
	HAL_TIM_Base_Stop_IT(timM);
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == tim->Instance)
//	{
//		if((++counter) >= timeout)
//			pxMBMasterPortCBTimerExpired();
//		if (htim->Instance == TIM2)
//			{
//		    HAL_IncTick();
//			}
//	}
//}

#endif
