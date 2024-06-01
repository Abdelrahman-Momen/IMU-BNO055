#ifndef BNO055_H_
#define BNO055_H_
//Includes
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

//Bitwise operations
//#define CLEAR_BIT(REG,NUM) REG&=~(1<<NUM)
//#define SET_BIT(REG,NUM) REG|=(1<<NUM)
#define TOGGLE_BIT(REG,NUM) REG^=(1<<NUM)
//#define READ_BIT(REG,NUM) ((REG&(1<<NUM))>>NUM)
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/******* Device address*****/
#define BNO055_ADDRESS 0x52

/*********Registers Page0**********/

#define MAG_RADIUS_MSB	0x6A
#define MAG_RADIUS_LSB	0x69

#define ACC_RADIUS_MSB	0x68
#define ACC_RADIUS_LSB	0x67
	
#define GYR_OFFSET_Z_MSB	0x66
#define GYR_OFFSET_Z_LSB	0x65

#define GYR_OFFSET_Y_MSB	0x64
#define GYR_OFFSET_Y_LSB	0x63
	
#define GYR_OFFSET_X_MSB	0x62
#define GYR_OFFSET_X_LSB	0x61
	
#define MAG_OFFSET_Z_MSB	0x60
#define MAG_OFFSET_Z_LSB	0x5F

#define MAG_OFFSET_Y_MSB	0x5E
#define MAG_OFFSET_Y_LSB	0x5D
	
#define MAG_OFFSET_X_MSB	0x5C
#define MAG_OFFSET_X_LSB	0x5B
	
#define ACC_OFFSET_Z_MSB	0x5A
#define ACC_OFFSET_Z_LSB	0x59

#define ACC_OFFSET_Y_MSB	0x58
#define ACC_OFFSET_Y_LSB	0x57
	
#define ACC_OFFSET_X_MSB	0x56
#define ACC_OFFSET_X_LSB	0x55
	
#define AXIS_MAP_SIGN 0x42
#define AXIS_MAP_CONFIG 0x41
	
#define TEMP_SOURCE 0x40
	
#define SYS_TRIGGER 0x3F
	
#define PWR_MODE 0x3E	
#define OPR_MODE 0x3D
	
#define UNIT_SEL 0x3B
	
#define SYS_ERR 0x3A
#define SYS_STATUS 0x39
#define SYS_CLK_STATUS 0x38

#define INT_STA 0x37
	
#define ST_RESULT 0x36
#define CALIB_STAT 0x35
	
#define TEMP 0x34

#define GRV_DATA_Z_MSB	0x33
#define GRV_DATA_Z_LSB	0x32

#define GRV_DATA_Y_MSB	0x31
#define GRV_DATA_Y_LSB	0x30
	
#define GRV_DATA_X_MSB	0x2F
#define GRV_DATA_X_LSB	0x2E
	
#define LIA_DATA_Z_MSB	0x2D
#define LIA_DATA_Z_LSB	0x2C

#define LIA_DATA_Y_MSB	0x2B
#define LIA_DATA_Y_LSB	0x2A
	
#define LIA_DATA_X_MSB	0x29
#define LIA_DATA_X_LSB	0x28
	
#define QUA_DATA_Z_MSB	0x27
#define QUA_DATA_Z_LSB	0x26

#define QUA_DATA_Y_MSB	 0x25
#define QUA_DATA_Y_LSB	 0x24
	
#define QUA_DATA_X_MSB	 0x23
#define QUA_DATA_X_LSB	 0x22
	
#define QUA_DATA_W_MSB	 0x21
#define QUA_DATA_W_LSB	 0x20

#define EUL_PITCH_MSB	 0x1F
#define EUL_PITCH_LSB	 0x1E
	
#define EUL_ROLL_MSB	 0x1D
#define EUL_ROLL_LSB	 0x1C
	
#define EUL_HEADING_MSB	 0x1B
#define EUL_HEADING_LSB	 0x1A

#define GYR_DATA_Z_MSB	 0x19
#define GYR_DATA_Z_LSB	 0x18

#define GYR_DATA_Y_MSB	 0x17
#define GYR_DATA_Y_LSB	 0x16
	
#define GYR_DATA_X_MSB	 0x15
#define GYR_DATA_X_LSB	 0x14
	
#define MAG_DATA_Z_MSB	 0x13
#define MAG_DATA_Z_LSB	 0x12

#define MAG_DATA_Y_MSB	 0x11
#define MAG_DATA_Y_LSB	 0x10
	
#define MAG_DATA_X_MSB	 0x0F
#define MAG_DATA_X_LSB	 0x0E
	
#define ACC_DATA_Z_MSB	 0x0D
#define ACC_DATA_Z_LSB	 0x0C

#define ACC_DATA_Y_MSB	 0x0B
#define ACC_DATA_Y_LSB	 0x0A
	
#define ACC_DATA_X_MSB	 0x09
#define ACC_DATA_X_LSB	 0x08

#define PAGE_ID	 0x07
#define BL_REV_ID	 0x06
	
#define SW_REV_ID_MSB	 0x05
#define SW_REV_ID_LSB	 0x04
	
#define GYR_ID	 0x03
#define MAG_ID	 0x02
#define ACC_ID	 0x01
#define CHIP_ID	 0x00

/*********Registers Page1**********/

#define GYR_AM_SET	 0x1F
#define GYR_AM_THRES	 0x1E
	
#define GYR_DUR_Z	 0x1D
#define GYR_HR_Z_SET	 0x1C
	
#define GYR_DUR_Y  0x1B
#define GYR_HR_Y_SET	 0x1A
	
#define GYR_DUR_X	 0x19
#define GYR_HR_X_SET	 0x18

#define GYR_INT_SETING	 0x17

#define ACC_NM_SET	 0x16
#define ACC_NM_THRES	 0x15

#define ACC_HG_SET	 0x14
#define ACC_HG_THRES	 0x13
	
#define ACC_INT_SETING	 0x12
	
#define ACC_AM_THRES	 0x11
	
#define INT_EN  0x10
#define INT_MSK  0x0F	
	
#define GYR_SLEEP_CONFIG  0x0D
#define ACC_SLEEP_CONFIG  0x0C
	
#define GYR_CONFIG_1  0x0B
#define GYR_CONFIG_0  0x0A

#define MAG_CONFIG  0x09
#define ACC_CONFIG  0x08

/***** Macro Defines***/

/** Power mode settings **/

#define	POWER_MODE_NORMAL  0b00
#define	POWER_MODE_LOW_POWER  0b01
#define	POWER_MODE_SUSPEND 0b10

/** Operation mode settings **/
#define  OPERATION_MODE_CONFIG 0b0000
#define  OPERATION_MODE_ACCONLY 0b0001
#define  OPERATION_MODE_MAGONLY 0b0010
#define  OPERATION_MODE_GYRONLY 0b0011
#define  OPERATION_MODE_ACCMAG 0b0100
#define  OPERATION_MODE_ACCGYRO 0b0101
#define  OPERATION_MODE_MAGGYRO 0b0110
#define  OPERATION_MODE_AMG 0b0111
#define  OPERATION_MODE_IMU 0b1000
#define  OPERATION_MODE_COMPASS 0b1001
#define  OPERATION_MODE_M4G 0b1010
#define  OPERATION_MODE_NDOF_FMC_OFF  0b1011
#define  OPERATION_MODE_NDOF 0b1100

/***** Axes ****/
#define	AXIS_X 0b00
#define	AXIS_Y 0b01
#define AXIS_Z 0b10
#define POSITIVE 0
#define NEGATIVE 1

/**** Accelerometer G ranges ****/
#define ACC_RANGE_2G 0b00
#define ACC_RANGE_4G 0b01
#define ACC_RANGE_8G 0b10
#define ACC_RANGE_16G 0b11

/**** Accelerometer bandwidth ****/ 
#define ACC_BANDWIDTH_7_81HZ 0b000
#define ACC_BANDWIDTH_15_63HZ 0b001
#define ACC_BANDWIDTH_31_25HZ 0b010
#define ACC_BANDWIDTH_62_5HZ 0b011
#define ACC_BANDWIDTH_125HZ 0b100
#define ACC_BANDWIDTH_250HZ 0b101
#define ACC_BANDWIDTH_500HZ 0b110
#define ACC_BANDWIDTH_1000HZ 0b111

/**** Accelerometer operation mode ****/ 
#define ACC_OPERATION_MODE_NORMAL 0b000
#define ACC_OPERATION_MODE_SUSPEND 0b001
#define ACC_OPERATION_MODE_LOWPOWER1 0b010
#define ACC_OPERATION_MODE_STANDBY 0b011
#define ACC_OPERATION_MODE_LOWPOWER2 0b100
#define ACC_OPERATION_MODE_DEEPSUSPEND 0b101

/**** Gyroscope ranges ****/
#define GYRO_RANGE_2000DPS 0b000
#define GYRO_RANGE_1000DPS 0b001
#define GYRO_RANGE_500DPS 0b010
#define GYRO_RANGE_250DPS 0b011
#define GYRO_RANGE_125DPS 0b100

/**** Gyroscope bandwidth ****/ 
#define GYRO_BANDWIDTH_523HZ 0b000
#define GYRO_BANDWIDTH_230HZ 0b001
#define GYRO_BANDWIDTH_116HZ 0b010
#define GYRO_BANDWIDTH_47HZ 0b011
#define GYRO_BANDWIDTH_23HZ 0b100
#define GYRO_BANDWIDTH_12HZ 0b101
#define GYRO_BANDWIDTH_64HZ 0b110
#define GYRO_BANDWIDTH_32HZ 0b111

/**** Gyroscope operation mode ****/ 
#define GYRO_OPERATION_MODE_NORMAL 0b000
#define GYRO_OPERATION_MODE_FASTPOWERUP 0b001
#define GYRO_OPERATION_MODE_DEEPSUSPEND 0b010
#define GYRO_OPERATION_MODE_SUSPEND 0b011
#define GYRO_OPERATION_MODE_ADVANCEDPOWERSAVE 0b100

/**** Magnetometer bandwidth ****/ 
#define MAG_DATA_OUTPUT_RATE_2HZ 0b000
#define MAG_DATA_OUTPUT_RATE_6HZ 0b001
#define MAG_DATA_OUTPUT_RATE_8HZ 0b010
#define MAG_DATA_OUTPUT_RATE_10HZ 0b011
#define MAG_DATA_OUTPUT_RATE_15HZ 0b100
#define MAG_DATA_OUTPUT_RATE_20HZ 0b101
#define MAG_DATA_OUTPUT_RATE_25HZ 0b110
#define MAG_DATA_OUTPUT_RATE_30HZ 0b111

/**** Magnetometer operation mode ****/ 
#define MAG_OPERATION_MODE_LOWPOWER 0b00
#define MAG_OPERATION_MODE_REGULAR 0b01
#define MAG_OPERATION_MODE_ENHANCEDREGULAR 0b10
#define MAG_OPERATION_MODE_HIGHACCURACY 0b11

/**** Magnetometer power mode ****/ 
#define MAG_POWER_MODE_NORMAL 0b00
#define MAG_POWER_MODE_SLEEP 0b01
#define MAG_POWER_MODE_SUSPEND 0b10
#define MAG_POWER_MODE_FORCEMODE 0b11

/**** Units ****/
#define ACC_UNIT_M_S2 0b0
#define ACC_UNIT_MG 0b1

#define GYRO_UNIT_DPS 0b0
#define GYRO_UNIT_RPS 0b1

#define EUL_UNIT_DEGREE 0b0
#define EUL_UNIT_RADIAN 0b1

#define TEMP_UNIT_C 0b0
#define TEMP_UNIT_F 0b1

#define UNIT_FORMAT_WINDOWS 0b0
#define UNIT_FORMAT_ANDROID 0b1

/** Function prototypes**/

/***Basic Configrations***/
void BNO055_setOperationMode (I2C_HandleTypeDef *hi2c ,uint8_t opMode);
void BNO055_setPowerMode (I2C_HandleTypeDef *hi2c ,uint8_t powerMode);
//void bno055_remapAxes ();

/**Accelerometer Configrations**/
void BNO055_ACC_set_G_range (I2C_HandleTypeDef *hi2c ,uint8_t gRange);
void BNO055_ACC_set_bandwidth (I2C_HandleTypeDef *hi2c ,uint8_t bandwidth);
void BNO055_ACC_set_operationMode (I2C_HandleTypeDef *hi2c ,uint8_t accOpMode);

/**Gyroscope Configrations**/
void BNO055_GYRO_set_range (I2C_HandleTypeDef *hi2c ,uint8_t range);
void BNO055_GYRO_set_bandwidth (I2C_HandleTypeDef *hi2c ,uint8_t bandwidth);
void BNO055_GYRO_set_operationMode (I2C_HandleTypeDef *hi2c ,uint8_t gyroOpMode);

/**Magnetometer Configrations**/
void BNO055_MAG_set_dataOutputRate (I2C_HandleTypeDef *hi2c ,uint8_t dataOutputRate);
void BNO055_MAG_set_operationMode (I2C_HandleTypeDef *hi2c ,uint8_t magOpMode);
void BNO055_MAG_set_powerMode (I2C_HandleTypeDef *hi2c ,uint8_t magPowerMode);

/***Unit Selection***/
void BNO055_UNIT_select (I2C_HandleTypeDef *hi2c ,uint8_t accUnit,uint8_t gyroUnit,uint8_t eulUnit,uint8_t tempUnit,uint8_t unitFormat);

/***********Getting Data*********/
void BNO055_ACC_get_readings (I2C_HandleTypeDef *hi2c ,float *acceleration);

void BNO055_GYRO_get_readings (I2C_HandleTypeDef *hi2c ,float *angularVelocity);

void BNO055_MAG_get_readings (I2C_HandleTypeDef *hi2c ,float *magneticField);

void BNO055_EUL_get_readings (I2C_HandleTypeDef *hi2c ,float *eulerAngles);

/**************default startup***********************/
void BNO055_default (I2C_HandleTypeDef *hi2c);

void BNO055_NDOF_Init(I2C_HandleTypeDef *hi2c);

/************ calibration state*********************/
void BNO055_calibration_state(I2C_HandleTypeDef *hi2c , uint8_t state);

#endif 