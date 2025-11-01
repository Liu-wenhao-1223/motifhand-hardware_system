/*
 * RS485.h
 *
 *  Created on: Apr 6, 2025
 *      Author: ASTRAYRMORS
 */

#ifndef RS485_RS485_H_
#define RS485_RS485_H_

#include "main.h"
#include "ICM42688P_SPI.h"
#include "bmm350_defs.h"
#include <string.h>
#include <stdio.h>

#define BOARDS_NUM 3+1  // the number of boards on each finger

//#define RS485_RX_BUFFER_SIZE 100
//#define RS485_TX_BUFFER_SIZE 100
#define RS485_RX_BUFFER_SIZE 1200  // 1 + 1 + 2 + 4 * 192 + 1 = 743
#define ID_LEN 3

#define NUM_FSR_X_VALUES 6
#define NUM_FSR_Y_VALUES 6


#pragma pack(1)
typedef struct {
//	uint8_t board_head;
	struct {
		uint8_t id;
	} ID;
	struct {
		float x; /**< Raw int16_t value from the x axis */
		float y; /**< Raw int16_t value from the y axis */
		float z; /**< Raw int16_t value from the z axis */
	} IMU_Acc;
	struct {
		float x; /**< Raw int16_t value from the x axis */
		float y; /**< Raw int16_t value from the y axis */
		float z; /**< Raw int16_t value from the z axis */
	} IMU_Gyro;
	struct {
	    float x; /*! Compensated mag X data */
	    float y; /*! Compensated mag Y data */
	    float z; /*! Compensated mag Z data */
	    float temperature; /*! Temperature */
	} Bmm_350;
	uint16_t FSR_36[NUM_FSR_X_VALUES][NUM_FSR_Y_VALUES];
//	float IMU[3];
} ModuleData_f;
#pragma pack()


void Fill_Buffer_With_Data(ModuleData_f* data);
void Receive_Data_Process(uint8_t* rx_buffer);

void RS485_DMA_Send_usart6(void);

#endif /* RS485_RS485_H_ */
