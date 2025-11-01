/*
 * WS2812.c
 *
 *  Created on: Mar 24, 2025
 *      Author: ASTRAYRMORS
 */
#include "WS2812.h"
//#include "delay.h"
#include "cmsis_gcc.h"
#include "DWT.h"
#include "tim.h"

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];

volatile int datasentflag = 1;

#define LED_NUM    1
#define BITS_PER_LED 24
//#define PWM_HIGH    168  // 0.7us / 1.25us * 300 ≈ 168
//#define PWM_LOW     84  // 0.35us / 1.25us * 300 ≈ 84

#define LED_ONE  192    // 0.8us / 1.25us * 300 ≈ 192 PWM_HIGH
#define LED_ZERO 108    // 0.4us / 1.25us * 300 ≈ 108 PWM_LOW


uint16_t pwm_buffer[LED_NUM * BITS_PER_LED + 50];  // 加 50 是为了 reset 码（空白期）
#define PWM_BUF_LEN (LED_NUM * 24 + 280)
uint16_t dma_buffer[24+50];
uint32_t pwmData[24+300];




void WS2812_Send(uint32_t color)
{
	uint16_t index = 0;

	for (int i=23; i>=0; i--)
	{
		if (color&(1<<i))
		{
			pwmData[index++] = LED_ONE;
		}
		else pwmData[index++] = LED_ZERO;
//		index++;
	}

    // 280us reset delay
	for (int i = 0; i < 300; i++) {
	    pwmData[index] = 0;
	    index++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, (uint32_t *)pwmData, index);

	while (!datasentflag){};
	datasentflag = 0;

}




