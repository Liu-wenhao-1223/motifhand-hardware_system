/*
 * DWT.c
 *
 *  Created on: Mar 19, 2025
 *      Author: ASTRAYRMORS
 */
/**
  ******************************************************************************
  * @file    dwt.c
  * @brief   This file includes the utilities for DWT
  ******************************************************************************
  */
#include "dwt.h"

static uint32_t SysCClk, start;

/**
  * @brief  Initialize DWT
  */
void DwtInit(void)
{
	SysCClk 		= (SystemCoreClock / 1000000);	// Calculate in us
	DWT_LAR			|= DWT_LAR_UNLOCK;
	DEM_CR			|= (uint32_t)DEM_CR_TRCENA;
	DWT_CYCCNT		= (uint32_t)0u;					// Reset the clock counter
	DWT_CR			|= (uint32_t)DWT_CR_CYCCNTENA;
}

/**
  * @brief  Start DWT Counter
  */
void DwtStart(void)
{
	start = DWT_CYCCNT;
}

/**
  * @brief  Calculate Interval Base On Previous Start Time
  * @retval Interval in us
  */
float DwtInterval(void)
{
	return (float)(DWT_CYCCNT - start) / SysCClk;
}

/**
  * @brief  Function to delay in microsecond
  * @param	usec	Period in microsecond
  */
inline void DwtDelay_us(uint32_t usec)
{
	start = DWT_CYCCNT;
	while(((DWT_CYCCNT - start) / SysCClk) < usec) {};
}

/**
  * @brief  Function to delay in millisecond
  * @param	msec	Period in millisecond
  */
inline void DwtDelay_ms(uint32_t msec)
{
	start = DWT_CYCCNT;
	while(((DWT_CYCCNT - start) / SysCClk) < (msec * 1000)) {};
}


//inline void BMM350_DwtDelay_us(uint32_t usec, void *intf_ptr)
//{
//	(void)intf_ptr;
//	start = DWT_CYCCNT;
//	while(((DWT_CYCCNT - start) / SysCClk) < usec) {};
//}

//inline void DwtDelay_ns(uint32_t nsec)
//{
//	uint32_t cycles = (uint32_t)((nsec * (float)SysCClk) / 1000.0f);  // 计算纳秒对应的周期
//	start = DWT_CYCCNT;
//	while ((DWT_CYCCNT - start) < cycles);
//}

//#include "DWT.h"
//
//
//uint8_t times_t;
//
//void bsp_InitDWT(void)
//{
//	DEM_CR         |= (unsigned int)DEM_CR_TRCENA;   /* Enable Cortex-M4's DWT CYCCNT reg.  */
//	DWT_CYCCNT      = (unsigned int)0u;
//	DWT_CR         |= (unsigned int)DWT_CR_CYCCNTENA;
//}
//
//
//void bsp_DelayUS(uint32_t _ulDelayTime)
//{
//    uint32_t tCnt, tDelayCnt;
//	uint32_t tStart;
//
//	tStart = DWT_CYCCNT;                                     /* 刚进入时的计数器值 */
//	tCnt = 0;
//	tDelayCnt = _ulDelayTime * (SystemCoreClock / 1000000);	 /* 需要的节拍数 */
//
//	while(tCnt < tDelayCnt)
//	{
//		tCnt = DWT_CYCCNT - tStart; /* 求减过程中，如果发生第一次32位计数器重新计数，依然可以正确计算 */
//	}
//}
//
//void bsp_DelayMS(uint32_t _ulDelayTime)
//{
//	bsp_DelayUS(1000*_ulDelayTime);
//}
//
///* Sleep implementation */
//void inv_imu_sleep_us(uint32_t us)
//{
//	bsp_DelayUS(us);
//}
//
//uint32_t inv_imu_get_time_us(void)
//{
//
//
//	return OS_TS_GET()/73 + times_t*58835168;
//}
//
//uint32_t OS_TS_GET(void)
//
//{
//	uint32_t _get_ts;
//
//	uint32_t _ts;
//
//	static uint32_t _ts_bak;    /* 时间戳备份 */
//
////	_get_ts = DWT_CYCCNT / 73;
//	_get_ts = DWT_CYCCNT;
//
//	if(_get_ts < _ts_bak)
//		{
//			/* 做溢出修正 */
//			_ts = 0XFFFFFFFF - _ts_bak + _get_ts;
////			_ts = 58835168 - _ts_bak + _get_ts;
//			/* 加上上次数据 即可求出本次时间差*/
//			_ts = _ts+_ts_bak;
//		}
//	else
//		{
//			/* 正常情况 */
//			_ts = _get_ts;
//		}
//
//		_ts_bak = _ts;
//
//		return _ts;
//}

