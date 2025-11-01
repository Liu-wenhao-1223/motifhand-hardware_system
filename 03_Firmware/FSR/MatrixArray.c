/*
 * MatrixArray.c
 *
 *  Created on: Mar 30, 2025
 *      Author: Wenhao Liu
 */
#include "MatrixArray.h"

uint8_t row_idx = 0;
uint8_t col_idx = 0;

GPIO_TypeDef* FSR_GPIO[6] = {GPIOC, GPIOA, GPIOC, GPIOC, GPIOA, GPIOA};

uint16_t FSR_Y_Pin[6] = {
    FSR_Y1_Pin,  // GPIOC
    FSR_Y2_Pin,  // GPIOA
    FSR_Y3_Pin,  // GPIOC
    FSR_Y4_Pin,  // GPIOC
    FSR_Y5_Pin,  // GPIOA
    FSR_Y6_Pin   // GPIOA
};

void Set_Switch_Channel(uint8_t col)
{
	HAL_GPIO_WritePin(GPIOC, Switch_A_Pin, (col & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Switch_B_Pin, (col & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Switch_C_Pin, (col & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Set_IO_Value(uint8_t row)
{
	for (int i = 0; i < 6; i++) {
//		HAL_GPIO_WritePin(FSR_GPIO[i], FSR_Y_Pin[i], (i == row) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		if (i == row) {
			HAL_GPIO_WritePin(FSR_GPIO[i], FSR_Y_Pin[i], GPIO_PIN_SET); 
		}
		else {
			HAL_GPIO_WritePin(FSR_GPIO[i], FSR_Y_Pin[i], GPIO_PIN_RESET);
		}
	}
}
