#include "BNO055.h"

/******************Basic Configrations****************/

void BNO055_setOperationMode (I2C_HandleTypeDef *hi2c , uint8_t opMode)
{
	uint8_t page;
	uint8_t memoryVal;
	uint8_t mask = 0b1111;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page =0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 0);
	memoryVal |= opMode << 0;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,OPR_MODE,1,&memoryVal,1,10);
	HAL_Delay (19);
}

void BNO055_setPowerMode (I2C_HandleTypeDef *hi2c , uint8_t powerMode)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b11;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
			BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PWR_MODE ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 0);
	memoryVal |= powerMode << 0;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PWR_MODE,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{		
		BNO055_setOperationMode (hi2c , mode);
	}
}

/************************Accelerometer Configrations*********************/
void BNO055_ACC_set_G_range(I2C_HandleTypeDef *hi2c ,uint8_t gRange)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b11;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,ACC_CONFIG ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 0);
	memoryVal |= gRange << 0;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,ACC_CONFIG,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

void BNO055_ACC_set_bandwidth (I2C_HandleTypeDef *hi2c ,uint8_t bandwidth)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b111;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , BNO055_ADDRESS ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,ACC_CONFIG ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 2);
	memoryVal |= bandwidth << 2;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,ACC_CONFIG,1,&memoryVal,1,10);

	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

void BNO055_ACC_set_operationMode (I2C_HandleTypeDef *hi2c ,uint8_t accOpMode)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b111;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , BNO055_ADDRESS ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,ACC_CONFIG ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 5);
	memoryVal |= accOpMode << 5;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,ACC_CONFIG,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

/***********************Gyroscope Configrations***************************/
void BNO055_GYRO_set_range (I2C_HandleTypeDef *hi2c ,uint8_t range)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b111;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,GYR_CONFIG_0 ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 0);
	memoryVal |= range << 0;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,GYR_CONFIG_0,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

void BNO055_GYRO_set_bandwidth (I2C_HandleTypeDef *hi2c ,uint8_t bandwidth)
{	
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b111;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,GYR_CONFIG_0 ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 3);
	memoryVal |= bandwidth << 3;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,GYR_CONFIG_0,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

void BNO055_GYRO_set_operationMode (I2C_HandleTypeDef *hi2c ,uint8_t gyroOpMode)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b111;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,GYR_CONFIG_1 ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 0);
	memoryVal |= gyroOpMode << 0;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,GYR_CONFIG_1,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}	
}

/*******************Magnetometer Configrations****************************/
void BNO055_MAG_set_dataOutputRate (I2C_HandleTypeDef *hi2c ,uint8_t dataOutputRate)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b111;
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);

	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,MAG_CONFIG ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 0);
	memoryVal |= dataOutputRate << 0;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,MAG_CONFIG,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

void BNO055_MAG_set_operationMode (I2C_HandleTypeDef *hi2c ,uint8_t magOpMode)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b11;
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,MAG_CONFIG ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 3);
	memoryVal |= magOpMode << 3;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,MAG_CONFIG,1,&memoryVal,1,10);
	
	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

void BNO055_MAG_set_powerMode (I2C_HandleTypeDef *hi2c ,uint8_t magPowerMode)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b11;
	
	HAL_I2C_Mem_Read (hi2c , BNO055_ADDRESS ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
	mode &= (modeMask<<0)>>0;
	if (mode != OPERATION_MODE_CONFIG)
	{
		BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
	}
	
	page = 0x01;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);	
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,MAG_CONFIG ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 5);
	memoryVal |= magPowerMode << 5;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,MAG_CONFIG,1,&memoryVal,1,10);

	if (mode != OPERATION_MODE_CONFIG)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
		BNO055_setOperationMode (hi2c , mode);	
	}
}

/**********************Unit Selection**********************/
void BNO055_UNIT_select (I2C_HandleTypeDef *hi2c ,uint8_t accUnit,uint8_t gyroUnit,uint8_t eulUnit,uint8_t tempUnit,uint8_t unitFormat)
{
	uint8_t page;
	uint8_t mode;
	uint8_t modeMask = 0b111;
	uint8_t memoryVal;
	uint8_t mask = 0b10010111;	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{	
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);
	}
	
		HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,OPR_MODE ,1 ,&mode,1,10);
		mode &= (modeMask<<0)>>0;
		if (mode != OPERATION_MODE_CONFIG)
		{
			BNO055_setOperationMode (hi2c , OPERATION_MODE_CONFIG);
		}	
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,UNIT_SEL ,1 ,&memoryVal,1,10);
	memoryVal &= (~mask << 0);
	memoryVal |= (accUnit<<0)|(gyroUnit<<1)|(eulUnit<<2)|(tempUnit<<4)|(unitFormat<<7) << 0;
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,UNIT_SEL,1,&memoryVal,1,10);

	if (mode != OPERATION_MODE_CONFIG)
	{		
		BNO055_setOperationMode (hi2c , mode);

	}
}

/***********Getting Data*********/
void BNO055_ACC_get_readings (I2C_HandleTypeDef *hi2c ,float *acceleration)
{
	uint8_t page;
	uint8_t memoryVal[6];
	uint8_t reg;
	int16_t data[3];
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);		
	}

	for (uint8_t i = 0;i<6;i++)
	{
		reg = ACC_DATA_X_LSB + i;
		HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,reg ,1 ,&memoryVal[i],1,10);
	}
	
	for (uint8_t j = 0;j<3;j++)
	{
		data[j] = (int16_t) (memoryVal[2*j]|(memoryVal[2*j+1]<<8));
		acceleration[j] = (float) data[j] /100.0;
	}
}

void BNO055_GYRO_get_readings (I2C_HandleTypeDef *hi2c ,float *angularVelocity)
{
	uint8_t page;
	uint8_t memoryVal[6];
	uint8_t reg;
	int16_t data[3];
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);		
	}

	for (uint8_t i = 0;i<6;i++)
	{
		reg = GYR_DATA_X_LSB + i;
		HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,reg ,1 ,&memoryVal[i],1,10);
	}
	
	for (uint8_t j = 0;j<3;j++)
	{
		data[j] = (int16_t) (memoryVal[2*j]|(memoryVal[2*j+1]<<8));
		angularVelocity[j] = (float) data[j] / 16.0;
	}	
}

void BNO055_MAG_get_readings (I2C_HandleTypeDef *hi2c ,float *magneticField)
{
	uint8_t page;
	uint8_t memoryVal[6];
	uint8_t reg;
	int16_t data[3];
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);		
	}

	for (uint8_t i = 0;i<6;i++)
	{
		reg = MAG_DATA_X_LSB + i;
		HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,reg ,1 ,&memoryVal[i],1,10);
	}
	
	for (uint8_t j = 0;j<3;j++)
	{
		data[j] = (int16_t) (memoryVal[2*j]|(memoryVal[2*j+1]<<8));
		magneticField[j] = (float) data[j] / 16.0;
	}	
}

void BNO055_EUL_get_readings (I2C_HandleTypeDef *hi2c ,float *eulerAngles)
{
	uint8_t page;
	uint8_t memoryVal[6];
	uint8_t reg;
	int16_t data[3];
	
	HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);		
	}

	for (uint8_t i = 0;i<6;i++)
	{
		reg = EUL_HEADING_LSB + i;
		HAL_I2C_Mem_Read (hi2c , (BNO055_ADDRESS) ,reg ,1 ,&memoryVal[i],1,10);
	}
	
	for (uint8_t j = 0;j<3;j++)
	{
		data[j] = (int16_t) (memoryVal[2*j]|(memoryVal[2*j+1]<<8));
		eulerAngles[j] = (float) data[j] / 16.0;
	}	
}

void BNO055_default (I2C_HandleTypeDef *hi2c)
{
	uint8_t reset = 0x20;
	
	HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,0x3F,1,&reset,1,100);
	
	BNO055_setPowerMode(hi2c , POWER_MODE_NORMAL);
	BNO055_setOperationMode (hi2c,OPERATION_MODE_CONFIG);
	
	BNO055_ACC_set_G_range (hi2c , ACC_RANGE_4G);
  BNO055_ACC_set_bandwidth (hi2c ,ACC_BANDWIDTH_62_5HZ);
	BNO055_ACC_set_operationMode (hi2c ,ACC_OPERATION_MODE_NORMAL);
	
	BNO055_GYRO_set_range (hi2c ,GYRO_RANGE_2000DPS);
	BNO055_GYRO_set_bandwidth (hi2c ,GYRO_BANDWIDTH_32HZ);
	BNO055_GYRO_set_operationMode (hi2c ,GYRO_OPERATION_MODE_NORMAL);
	
	BNO055_MAG_set_dataOutputRate (hi2c,MAG_DATA_OUTPUT_RATE_20HZ);
	BNO055_MAG_set_operationMode (hi2c,MAG_OPERATION_MODE_REGULAR);
	BNO055_MAG_set_powerMode (hi2c,MAG_POWER_MODE_FORCEMODE);
	
	BNO055_UNIT_select (hi2c ,ACC_UNIT_M_S2,GYRO_UNIT_DPS,EUL_UNIT_DEGREE, TEMP_UNIT_C ,UNIT_FORMAT_WINDOWS);
	
	BNO055_setOperationMode (hi2c,OPERATION_MODE_AMG);
}

void BNO055_NDOF_Init(I2C_HandleTypeDef *hi2c)
{
	BNO055_setPowerMode(hi2c , POWER_MODE_NORMAL);
	BNO055_setOperationMode (hi2c,OPERATION_MODE_CONFIG);
	
	BNO055_ACC_set_G_range (hi2c , ACC_RANGE_4G);
	
	BNO055_UNIT_select (hi2c ,ACC_UNIT_M_S2,GYRO_UNIT_DPS,EUL_UNIT_DEGREE, TEMP_UNIT_C ,UNIT_FORMAT_WINDOWS);
	
	BNO055_setOperationMode (hi2c,OPERATION_MODE_NDOF);
}

void BNO055_calibration_state(I2C_HandleTypeDef *hi2c , uint8_t state)
{
	uint8_t page;

	HAL_I2C_Mem_Read (hi2c,BNO055_ADDRESS,PAGE_ID ,1 ,&page,1,10);
	if (page == 0x01)
	{
		page = 0x00;
		HAL_I2C_Mem_Write (hi2c,BNO055_ADDRESS,PAGE_ID,1,&page,1,10);		
	}
	
	HAL_I2C_Mem_Read (hi2c ,BNO055_ADDRESS,CALIB_STAT ,1 ,&state,1,10);
}