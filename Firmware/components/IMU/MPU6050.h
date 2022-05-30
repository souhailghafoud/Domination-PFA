#ifndef _MPU6050_H_   /* Include guard */
#define _MPU6050_H_



/****************************************** Header Includes *******************************************/

/* ESP error */
#include "esp_err.h"



/********************************************* Constants **********************************************/

#define MPU6050_AXIS                            3



/************************************** Enumeration Definitions ***************************************/

/*!
 * @brief Accelerometer full scale range.
 */
typedef enum {
	ACCEL_FS_2G = 0,
	ACCEL_FS_4G,
	ACCEL_FS_8G,
	ACCEL_FS_16G
} accel_scale_t;


/*!
 * @brief Gyroscope full scale range.
 */
typedef enum {
	GYRO_FS_250DPS = 0,
	GYRO_FS_500DPS,
	GYRO_FS_1000DPS,
	GYRO_FS_2000DPS
} gyro_scale_t;


/*!
 * @brief 3-axis index.
 */
typedef enum {
    AXIS_X = 0,      // x-axis
    AXIS_Y,          // y-axis
    AXIS_Z           // z-axis
} mpu6050_axis_index_t;



/*************************************** Structure Definitions ****************************************/

typedef struct {
    float gyro_data[MPU6050_AXIS];
    float gyro_bias[MPU6050_AXIS];
    gyro_scale_t gyro_scale;
    float accel_data[MPU6050_AXIS];
    float accel_bias[MPU6050_AXIS];
    accel_scale_t accel_scale;
} mpu6050_t;



/*********************************** Function Prototype Declarations **********************************/

/*!
 * @brief This public function is used to initialize the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of mpu6050_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t mpu6050_init(mpu6050_t mpu6050);


/*!
 * @brief This public function to get the gyroscope resolution of the MPU6050 sensor.
 * 
 * @param[in,out] gyro_scale  :Enumeration instance of gyro_scale_t.
 *
 * @return The gyroscope resolution.
 */
float mpu6050_get_gyro_res(gyro_scale_t gyro_scale);


/*!
 * @brief This public function to get the accelerometer resolution of the MPU6050 sensor.
 * 
 * @param[in,out] accel_scale  :Enumeration instance of accel_scale_t.
 *
 * @return The accelerometer resolution.
 */
float mpu6050_get_accel_res(accel_scale_t accel_scale);


/*!
 * @brief This public function is used to fetch new gyroscope data
 *        from the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of mpu6050_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t mpu6050_read_gyro(mpu6050_t *mpu6050);


/*!
 * @brief This public function is used to fetch new accelerometer data
 *        from the MPU6050 sensor.
 * 
 * @param[in,out] mpu6050  :Structure instance of mpu6050_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t mpu6050_read_accel(mpu6050_t *mpu6050);



#endif  /* End of include guard */

/******************************************************************************************************/

