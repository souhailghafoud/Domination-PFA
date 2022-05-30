
/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>         // For uint8_t definition 
#include <stdbool.h>        // For true/false definition

/* I2C */
#include "i2c.h"

/* MPU6050 */
#include "mpu6050.h"



/********************************************* Constants **********************************************/

#define MPU6050_I2C_ADDR                        0x68

#define MPU6050_REG_SMPRT_DIV                   0x19
#define MPU6050_REG_CONFIG                      0x1A
#define MPU6050_REG_GYRO_CONFIG                 0x1B
#define MPU6050_REG_ACCEL_CONFIG                0x1C
#define MPU6050_REG_FIFO_EN                     0x23
#define MPU6050_REG_INT_ENABLE                  0x38
#define MPU6050_REG_INT_STATUS                  0x3A
#define MPU6050_REG_ACCEL_XOUT_H                0x3B
#define MPU6050_REG_ACCEL_XOUT_L                0x3C
#define MPU6050_REG_ACCEL_YOUT_H                0x3D
#define MPU6050_REG_ACCEL_YOUT_L                0x3E
#define MPU6050_REG_ACCEL_ZOUT_H                0x3F
#define MPU6050_REG_ACCEL_ZOUT_L                0x40
#define MPU6050_REG_TEMP_OUT_H                  0x41
#define MPU6050_REG_TEMP_OUT_L                  0x42
#define MPU6050_REG_GYRO_XOUT_H                 0x43
#define MPU6050_REG_GYRO_XOUT_L                 0x44
#define MPU6050_REG_GYRO_YOUT_H                 0x45
#define MPU6050_REG_GYRO_YOUT_L                 0x46
#define MPU6050_REG_GYRO_ZOUT_H                 0x47
#define MPU6050_REG_GYRO_ZOUT_L                 0x48
#define MPU6050_REG_USER_CTRL                   0x6A    // Bit 7 enable DMP, bit 3 reset DMP
#define MPU6050_REG_PWR_MGMT_1                  0x6B    // Device defaults to the SLEEP mode
#define MPU6050_REG_PWR_MGMT_2                  0x6C
#define MPU6050_REG_DMP_BANK                    0x6D    // Activates a specific bank in the DMP
#define MPU6050_REG_DMP_RW_PNT                  0x6E    // Set read/write pointer to a specific start address in specified DMP bank
#define MPU6050_REG_DMP_REG                     0x6F    // Register in DMP from which to read or to which to write
#define MPU6050_REG_DMP_REG_1                   0x70
#define MPU6050_REG_DMP_REG_2                   0x71
#define MPU6050_REG_FIFO_COUNTH                 0x72
#define MPU6050_REG_FIFO_COUNTL                 0x73
#define MPU6050_REG_FIFO_R_W                    0x74
#define MPU6050_REG_WHO_AM_I                    0x75



/*************************************** Structure Definitions ****************************************/

typedef struct {
	uint8_t DLPF_CFG		        : 2;
	uint8_t EXT_SYNC_SET	        : 3;
	uint8_t reserved_0		        : 2;
} mpu6050_config_t;


typedef struct {
	uint8_t reserved_0		        : 3;
	uint8_t FS_SEL			        : 2;
	uint8_t ZG_ST			        : 1;
	uint8_t YG_ST			        : 1;
	uint8_t XG_ST			        : 1;
} mpu6050_gyro_config_t;


typedef struct {
	uint8_t reserved_0		        : 3;
	uint8_t AFS_SEL			        : 2;
	uint8_t ZA_ST			        : 1;
	uint8_t YA_ST			        : 1;
	uint8_t XA_ST			        : 1;
} mpu6050_accel_config_t;


typedef struct {
	uint8_t SIG_COND_RESET			: 1;
	uint8_t I2C_MST_RESET			: 1;
	uint8_t FIFO_RESET				: 1;
	uint8_t reserved_1 				: 1;
	uint8_t	I2C_IF_DIS				: 1;
	uint8_t I2C_MST_EN				: 1;
	uint8_t FIFO_EN 				: 1;
	uint8_t reserved_0 				: 1;
} mpu6050_user_ctrl_t;


typedef struct {
	uint8_t CLKSEL					: 3;
	uint8_t TEMP_DIS				: 1;
	uint8_t reserved_0 				: 1;
	uint8_t CYCLE					: 1;
	uint8_t SLEEP					: 1;
	uint8_t DEVICE_RESET			: 1;
} mpu6050_pwr_mgmt_1_t;


typedef struct{
	uint8_t STBY_ZG				    : 1;
	uint8_t STBY_YG				    : 1;
	uint8_t STBY_XG				    : 1;
	uint8_t STBY_ZA				    : 1;
	uint8_t STBY_YA				    : 1;
	uint8_t STBY_XA				    : 1;
	uint8_t LP_WAKE_CTRL			: 2;
} mpu6050_pwr_mgmt_2_t;



/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used to write in MPU6050 registers.
 * 
 * @param[in] reg  :MPU6050 register to write.
 * @param[in,out] data  :data to write into the register.
 * @param[in] len  :Data length.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t *data, uint32_t len)
{
    return i2c_send_data(MPU6050_I2C_ADDR, &reg, data, len);
}


/*!
 * @brief This private function is used to read MPU6050 registers.
 * 
 * @param[in] reg  :MPU6050 register to read.
 * @param[in,out] data  :To store the content of the register.
 * @param[in] len  :Data length.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data, uint32_t len)
{
    return i2c_read_data(MPU6050_I2C_ADDR, &reg, data, len);
}



/***************************************** Public Functions *******************************************/

/*!
 * @brief This public function is used to initialize the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of mpu6050_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t mpu6050_init(mpu6050_t mpu6050)
{
    /* Check device ID */
    uint8_t who_am_i = 0;
    if (ESP_OK != mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i, 1)) {
        return ESP_FAIL;
    }
    if (MPU6050_I2C_ADDR != who_am_i) {
        return ESP_FAIL;
    }

    /* Set clock source to be PLL with x-axis gyroscope reference */
	mpu6050_pwr_mgmt_1_t pwr_mgmt_1 = {0};
    pwr_mgmt_1.CLKSEL = 1;
    if (ESP_OK != mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, (uint8_t *)&pwr_mgmt_1, sizeof(mpu6050_pwr_mgmt_1_t))) {
        return ESP_FAIL;
    }

    /* Disable FSYNC and set accel and gyro bandwidth to 44 and 42 Hz, respectively.
     * - Set sample rate at 1 kHz for accel and gyro.
     * - Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate.
     * */
	mpu6050_config_t config = {0};
    config.DLPF_CFG  = 3;
    if (ESP_OK != mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, (uint8_t *)&config, sizeof(mpu6050_config_t))) {
        return ESP_FAIL;
    }

    /* Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) */
    uint8_t smplrt_div = 0x04;  // Use a 200 Hz rate; the same rate set in CONFIG above
    if (ESP_OK != mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, &smplrt_div, 1)) {
        return ESP_FAIL;
    }

    /* Set gyroscope full scale range */
	mpu6050_gyro_config_t gyro_config = {0};
    gyro_config.FS_SEL = mpu6050.gyro_scale;
    if (ESP_OK != mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, (uint8_t *)&gyro_config, sizeof(mpu6050_gyro_config_t))) {
        return ESP_FAIL;
    }

    /* Set accelerometer full scale range */
	mpu6050_accel_config_t accel_config = {0};
    accel_config.AFS_SEL = mpu6050.accel_scale;
    if (ESP_OK != mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, (uint8_t *)&accel_config, sizeof(mpu6050_accel_config_t))) {
        return ESP_FAIL;
    }

    return ESP_OK;
}


/*!
 * @brief This public function to get the gyroscope resolution of the MPU6050 sensor.
 * 
 * @param[in,out] gyro_scale  :Enumeration instance of gyro_scale_t.
 *
 * @return The gyroscope resolution.
 */
float mpu6050_get_gyro_res(gyro_scale_t gyro_scale)
{
    switch (gyro_scale)
    {
        /* Possible gyro scales (and their register bit settings) are:
         * 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
         * Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
         * */
        case GYRO_FS_250DPS:
            return (32768.0f / 250.0f);
            break;
        case GYRO_FS_500DPS:
            return (32768.0f / 500.0f);
            break;
        case GYRO_FS_1000DPS:
            return (32768.0f / 1000.0f);
            break;
        case GYRO_FS_2000DPS:
            return (32768.0f / 2000.0f);
            break;
        default :
            return 0;
            break;
    }
}


/*!
 * @brief This public function to get the accelerometer resolution of the MPU6050 sensor.
 * 
 * @param[in,out] accel_scale  :Enumeration instance of accel_scale_t.
 *
 * @return The accelerometer resolution.
 */
float mpu6050_get_accel_res(accel_scale_t accel_scale)
{
    switch (accel_scale)
    {
        /* Possible accelerometer scales (and their register bit settings) are:
         * 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
         * Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
         * */
        case ACCEL_FS_2G:
            return (32768.0f / 2.0f);
            break;
        case ACCEL_FS_4G:
            return (32768.0f / 4.0f);
            break;
        case ACCEL_FS_8G:
            return (32768.0f / 8.0f);
            break;
        case ACCEL_FS_16G:
            return (32768.0f / 16.0f);
            break;
        default :
            return 0;
            break;
    }
}


/*!
 * @brief This public function is used to fetch new gyroscope data
 *        from the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of mpu6050_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t mpu6050_read_gyro(mpu6050_t *mpu6050)
{
    uint8_t raw_gyro[6] = {0};
    int16_t gyro_16b[3] = {0};

    /* Fetch raw 3-axis gyroscope data from MPU6050 sensor (High and low bits) */
    if (ESP_OK != mpu6050_read_reg(MPU6050_REG_GYRO_XOUT_H, raw_gyro, 6)) {
        return ESP_FAIL;
    }
        
    /* Convert high and low bits into full 16 bits data (x, y, z) */
    gyro_16b[AXIS_X] = (int16_t)(((int16_t)raw_gyro[0] << 8) | raw_gyro[1]);
    gyro_16b[AXIS_Y] = (int16_t)(((int16_t)raw_gyro[2] << 8) | raw_gyro[3]);
    gyro_16b[AXIS_Z] = (int16_t)(((int16_t)raw_gyro[4] << 8) | raw_gyro[5]);

    /* Scale 3-axis gyroscope data according to the current resolution (250dps, 500dps, 1000dps, 2000dps)*/
    for (uint8_t axis = 0; axis < 3; axis++) {
        mpu6050->gyro_data[axis] = ((float)gyro_16b[axis] / mpu6050_get_gyro_res(mpu6050->gyro_scale));
    }

    return ESP_OK;
}


/*!
 * @brief This public function is used to fetch new accelerometer data
 *        from the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of mpu6050_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t mpu6050_read_accel(mpu6050_t *mpu6050)
{
    uint8_t raw_accel[6] = {0};
    int16_t accel_16b[3] = {0};

    /* Fetch raw 3-axis accelerometer data from MPU6050 sensor (High and low bits) */
    if (ESP_OK != mpu6050_read_reg(MPU6050_REG_ACCEL_XOUT_H, raw_accel, 6)) {
        return ESP_FAIL;
    }
        
    /* Convert high and low bits into full 16 bits data (x, y, z) */
    accel_16b[AXIS_X] = (int16_t)(((int16_t)raw_accel[0] << 8) | raw_accel[1]);
    accel_16b[AXIS_Y] = (int16_t)(((int16_t)raw_accel[2] << 8) | raw_accel[3]);
    accel_16b[AXIS_Z] = (int16_t)(((int16_t)raw_accel[4] << 8) | raw_accel[5]);

    /* Scale 3-axis accelerometer data according to the current resolution (2g, 4g, 8g, 16g)*/
    for (uint8_t axis = 0; axis < 3; axis++) {
        mpu6050->accel_data[axis] = ((float)accel_16b[axis] / mpu6050_get_accel_res(mpu6050->accel_scale));
    }

    return ESP_OK;
}


/******************************************************************************************************/