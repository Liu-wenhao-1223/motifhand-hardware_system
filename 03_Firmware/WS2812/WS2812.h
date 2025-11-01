/*
 * WS2812.h
 *
 *  Created on: Mar 24, 2025
 *      Author: ASTRAYRMORS
 */

#ifndef WS2812_WS2812_H_
#define WS2812_WS2812_H_

#include "main.h"

#define		RGB_LED_HIGH	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define 	RGB_LED_LOW		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)

#define MAX_LED 1
#define USE_BRIGHTNESS 1
#define PI 3.14159265

// Colours
#define  dark_red    0x001000
#define  mid_red     0x005000
#define  light_red   0x00ff00

#define  dark_blue   0x000010
#define  mid_blue    0x000050
#define  light_blue  0x0000ff

#define  dark_green  0x100000
#define  mid_green   0x500000
#define  light_green 0xff0000

#define  PINLAN     0x2b73af



void WS2812_Send(uint32_t color);


#endif /* WS2812_WS2812_H_ */
