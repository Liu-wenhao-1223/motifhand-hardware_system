/*
 * RS485.c
 *
 *  Created on: Apr 6, 2025
 *      Author: ASTRAYRMORS
 */

#include "RS485.h"
//#include "gpio.h"

ModuleData_f small_boards[BOARDS_NUM];
ModuleData_f current_small_board;

#define RS485_TX_BUFFER_SIZE sizeof(small_boards)


uint8_t rs485_rx_buffer[RS485_RX_BUFFER_SIZE];
uint8_t rs485_rx_id_buffer[10];  // 3
uint8_t rs485_tx_buffer[RS485_TX_BUFFER_SIZE];

extern UART_HandleTypeDef huart1;
extern icm42688RawData_t stAccData;
extern icm42688RawData_t stGyroData;
extern icm42688RawData_t stAccData_calib;
extern icm42688RawData_t stGyroData_calib;
//extern int16_t ICM_Temp;

uint8_t board_head1 = 0xA1; // The head of the board
uint8_t board_head2; // The head of the board
uint8_t board_tail = 0x55; // The tail of the board

extern uint8_t board_id_slave;
extern char board_id[8]; // The id of the board, {finger}-{num} "0-1"
//int finger_set, num_set;

//#define RS485_TX_BUFFER_SIZE (NUM_FSR_VALUES * sizeof(float))
extern struct bmm350_mag_temp_data mag_temp_data;
extern struct bmm350_mag_temp_data mag_calibrated_data;
extern float FSR_values[NUM_FSR_X_VALUES][NUM_FSR_Y_VALUES];
extern float FSR_values_proc[NUM_FSR_X_VALUES][NUM_FSR_Y_VALUES];
extern uint8_t tx_flag;


void Fill_Buffer_With_Data(ModuleData_f* data)
{
	// Clean Buffer
	data->ID.id = board_id_slave;
	data->IMU_Acc.x = stAccData_calib.x;
	data->IMU_Acc.y = stAccData_calib.y;
	data->IMU_Acc.z = stAccData_calib.z;
	data->IMU_Gyro.x = stGyroData_calib.x;
	data->IMU_Gyro.y = stGyroData_calib.y;
	data->IMU_Gyro.z = stGyroData_calib.z;
	data->Bmm_350.x = mag_calibrated_data.x;
	data->Bmm_350.y = mag_calibrated_data.y;
	data->Bmm_350.z = mag_calibrated_data.z;
	data->Bmm_350.temperature = mag_temp_data.temperature;

//	cnm.FSR_36 = FSR_values;
	memcpy(data->FSR_36, FSR_values_proc, sizeof(FSR_values_proc));
}

void Receive_Data_Process(uint8_t* rx_buffer)
{
	uint8_t pre_head = rx_buffer[0];
	uint8_t pre_head_num = rx_buffer[1];
	uint16_t payload_len = rx_buffer[2] | (rx_buffer[3] << 8);
	uint8_t pre_tail = rx_buffer[1 + 1 + 2 + payload_len];
	ModuleData_f pre_small_boards_buffer[BOARDS_NUM];

	if (pre_head == board_head1 && payload_len == sizeof(small_boards) && pre_tail == board_tail)
	{

		if (pre_head_num < board_head2)
		{
			memcpy(pre_small_boards_buffer, &rx_buffer[4], payload_len);

			for(int i = 0; i <= pre_head_num; i++)
			{
				memcpy(&small_boards[i], &pre_small_boards_buffer[i], sizeof(ModuleData_f));
			}
		}
	}
}


void RS485_DMA_Send(void)
{
//	Fill_Buffer_With_less_Data();

	uint8_t tx_buffer[1000];

	uint16_t offset = 0;
	uint16_t payload_len = sizeof(small_boards);

	tx_buffer[offset++] = board_head1;
	tx_buffer[offset++] = board_head2;

	tx_buffer[offset++] = (payload_len & 0xFF);
	tx_buffer[offset++] = (payload_len >> 8) & 0xFF;  // 高字节


	memcpy(&tx_buffer[offset], &small_boards, payload_len);
	offset += payload_len;

	tx_buffer[offset++] = board_tail;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart1, tx_buffer, offset);
}

void RS485_DMA_Receive(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rs485_rx_id_buffer, ID_LEN);
}

void RS485_DMA_Send_usart6(void)
{
//	Fill_Buffer_With_less_Data();

//	uint8_t tx_buffer[4000];
	uint8_t tx_buffer[4000];
//	uint8_t tx_buffer[1000];
	uint16_t offset = 0;
	uint16_t payload_len = sizeof(all_boards);
//	uint16_t payload_len = sizeof(finger_2);
//	uint16_t payload_len = sizeof(small_board_1);

	tx_buffer[offset++] = board_head1;
	tx_buffer[offset++] = board_head2;

	//	tx_buffer[offset++] = (uint8_t)payload_len;

	tx_buffer[offset++] = (payload_len & 0xFF);
	tx_buffer[offset++] = (payload_len >> 8) & 0xFF;  // 高字节

	memcpy(&tx_buffer[offset], &all_boards, payload_len);
//	memcpy(&tx_buffer[offset], &finger_2, payload_len);
//	memcpy(&tx_buffer[offset], &small_board_1, payload_len);
	offset += payload_len;

	tx_buffer[offset++] = board_tail;

	HAL_UART_Transmit_DMA(&huart6, tx_buffer, offset);
}