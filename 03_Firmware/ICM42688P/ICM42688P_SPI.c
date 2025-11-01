/*
 * ICM42688P.c
 *
 *  Created on: Mar 19, 2025
 *      Author: Wenhao Liu
 */

#include "ICM42688P_SPI.h"
//#include "DWT.h"

float accSensitivity;   //加速度的最小分辨率 mg/LSB
float gyroSensitivity;    //陀螺仪的最小分辨率
float g1 = 9.80665f;

//icm42688RawData_t stAccData;
//icm42688RawData_t stGyroData;

uint8_t hal_Spi_ReadWriteByte(uint8_t txdata,SPI_HandleTypeDef* hspi)
{
    uint8_t rxdata = 0;
    HAL_SPI_TransmitReceive(hspi, &txdata, &rxdata, 1, 5);
//    HAL_SPI_TransmitReceive_IT(hspi, &txdata, &rxdata, 1);
    return rxdata;
}

static void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len, SPI_HandleTypeDef* hspi)
{
    uint8_t i = 0;

    for(i = 0; i < len; i ++)
    {
        *pBuffer = hal_Spi_ReadWriteByte(*pBuffer,hspi);
        pBuffer++;
    }

}

static uint8_t icm42688_read_reg(uint8_t reg, SPI_HandleTypeDef* hspi)
{
    uint8_t regval = 0;


//    if(hspi == &hspi1){
//    ICM_SPI1_CS_LOW();
//    }
    SPI_IMU_CS_LOW();
    reg |= 0x80;
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1, hspi);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&regval, 1, hspi);
//    if(hspi == &hspi1){
//    ICM_SPI1_CS_HIGH();
//    }
    SPI_IMU_CS_HIGH();

    return regval;
}

static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len, SPI_HandleTypeDef* hspi)
{

    reg |= 0x80;
//    if(hspi == &hspi1){
//    ICM_SPI2_CS_LOW();
//    }
    SPI_IMU_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1, hspi);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(buf, len, hspi);
//    if(hspi == &hspi1){
//    ICM_SPI2_CS_HIGH();
//    }
    SPI_IMU_CS_HIGH();

}

static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value, SPI_HandleTypeDef* hspi)
{

//	 if(hspi == &hspi1){
//	 ICM_SPI2_CS_LOW();
//	 }
	SPI_IMU_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1, hspi);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&value, 1, hspi);
//    if(hspi == &hspi1){
//    ICM_SPI2_CS_HIGH();
//    }
    SPI_IMU_CS_HIGH();
	return value;

}


float bsp_Icm42688GetAres(uint8_t Ascale)
{
	//float accSensitivity;
    switch(Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2*g1 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4*g1 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8*g1 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16*g1 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
	//float gyroSensitivity;
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}

uint8_t reg_val = 0;
int8_t bsp_Icm42688RegCfg(SPI_HandleTypeDef* hspi)
{

    /* 读取 who am i 寄存器 */
//    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0, hspi); //设置bank 0区域寄存器
    reg_val = icm42688_read_reg(ICM42688_WHO_AM_I, hspi);
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0, hspi); //设置bank 0区域寄存器
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x01, hspi); //软复位传感器

//    ICM42688DelayMs(50);
    HAL_Delay(50);
//    rt_kprintf("%x\n",reg_val);

    if(reg_val == ICM42688_ID)
    {
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 1, hspi); //设置bank 1区域寄存器
        icm42688_write_reg(ICM42688_INTF_CONFIG4, 0x02, hspi); //设置为4线SPI通信  0x02
        icm42688_write_reg(ICM42688_INTF_CONFIG5, 0x04, hspi);  //0x04

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0, hspi); //设置bank 0区域寄存器
        icm42688_write_reg(ICM42688_FIFO_CONFIG, 0x40, hspi);//Stream-to-FIFO Mode(page63)

        icm42688_write_reg(ICM42688_SENSOR_CONFIG0, 0x80, hspi);

		icm42688_write_reg(ICM42688_REG_BANK_SEL, 1, hspi); //设置bank 1区域寄存器
        icm42688_write_reg(ICM42688_GYRO_CONFIG_STATIC2, 0xA0, hspi);
//		ICM42688DelayMs(5);
//        DWT_Delay_Us(200);
        HAL_Delay(2);

		icm42688_write_reg(ICM42688_REG_BANK_SEL, 2, hspi); //设置bank 1区域寄存器
        icm42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC2, 0x03, hspi);
//		ICM42688DelayMs(5);
//        DWT_Delay_Us(200);
        HAL_Delay(2);

		icm42688_write_reg(ICM42688_REG_BANK_SEL, 0, hspi); //设置bank 0区域寄存器
        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0, hspi);
        icm42688_write_reg(ICM42688_INT_SOURCE0, 0x00, hspi);
        icm42688_write_reg(ICM42688_FIFO_CONFIG2, 0x00, hspi); // watermark
        icm42688_write_reg(ICM42688_FIFO_CONFIG3, 0x02, hspi); // watermark
        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val, hspi);
        icm42688_write_reg(ICM42688_FIFO_CONFIG1, 0x63, hspi); // Enable the accel and gyro to the FIFO

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
        icm42688_write_reg(ICM42688_INT_CONFIG, 0x36, hspi);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0, hspi);
        reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val, hspi);

        bsp_Icm42688GetAres(AFS_8G);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
        reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG0, hspi);//page74
        reg_val |= (AFS_8G << 5);   //量程 ±8g
        reg_val |= (AODR_8000Hz);     //输出速率 32000HZ
        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val, hspi);
//        ICM42688DelayMs(5);
//        DWT_Delay_Us(200);
        HAL_Delay(2);

        bsp_Icm42688GetGres(GFS_1000DPS);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
        reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0, hspi);//page73
        reg_val |= (GFS_1000DPS << 5);   //量程 ±1000dps  500
        reg_val |= (GODR_8000Hz);     //输出速率 32000HZ
        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val, hspi);
//		ICM42688DelayMs(5);
//		DWT_Delay_Us(200);
        HAL_Delay(2);

		icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
		icm42688_write_reg(ICM42688_GYRO_CONFIG1, 0x1A, hspi);   //0x12 0x16 0x1A
//		ICM42688DelayMs(5);
//		DWT_Delay_Us(200);
		HAL_Delay(2);

		icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
		icm42688_write_reg(ICM42688_ACCEL_CONFIG1, 0x14, hspi);  // 0x04  0x0D  0x14
//		ICM42688DelayMs(5);
//		DWT_Delay_Us(200);
		HAL_Delay(2);

		//icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
		icm42688_write_reg(ICM42688_GYRO_ACCEL_CONFIG0, 0x42, hspi);
//		ICM42688DelayMs(5); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作
//		DWT_Delay_Us(200);
		HAL_Delay(2);
         //自测功能可以不用开
//		icm42688_write_reg(ICM42688_REG_BANK_SEL, 0, hspi); //设置bank 1区域寄存器
//      icm42688_write_reg(ICM42688_SELF_TEST_CONFIG, 0x8F, hspi);
//		//icm42688_write_reg(ICM42688_SELF_TEST_CONFIG, 0x80, hspi);
//		ICM42688DelayMs(1);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00, hspi);
        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0, hspi); //读取PWR—MGMT0当前寄存器的值(page72)
//        reg_val |= (1 << 5);//不使能温度测量
        reg_val &= ~(1 << 5); // 使能温度测量
        reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val, hspi);
//        ICM42688DelayMs(5); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作
//        DWT_Delay_Us(200);
        HAL_Delay(2);
		return 0;
    }
    return -1;
}

int8_t bsp_Icm42688Init(SPI_HandleTypeDef* hspi)
{
    return(bsp_Icm42688RegCfg(hspi));

}


//这里最好不要连续读
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData, SPI_HandleTypeDef* hspi)
{

	uint8_t buffer_accel[6] = {0};


	icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer_accel, 6, hspi);

	accData->x = (int16_t)((buffer_accel[0] << 8) | buffer_accel[1]);
	accData->y = (int16_t)((buffer_accel[2] << 8) | buffer_accel[3]);
	accData->z = (int16_t)((buffer_accel[4] << 8) | buffer_accel[5]);


	accData->x *= accSensitivity;
	accData->y *= accSensitivity;
	accData->z *= accSensitivity;

	return 0;
}


//这里最好不要连续读
int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData, SPI_HandleTypeDef* hspi)
{
//	last_ICM42688_val Last_ICM42688_val;
//	short Temp[2]={0};
    uint8_t buffer_gyro[6] = {0};

	icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer_gyro, 6, hspi);

	GyroData->x = (int16_t)((buffer_gyro[0] << 8) | buffer_gyro[1]);
	GyroData->y = (int16_t)((buffer_gyro[2] << 8) | buffer_gyro[3]);
	GyroData->z = (int16_t)((buffer_gyro[4] << 8) | buffer_gyro[5]);

	GyroData->x *= gyroSensitivity;
	GyroData->y *= gyroSensitivity;
	GyroData->z *= gyroSensitivity;

    return 0;
}
