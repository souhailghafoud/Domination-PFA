#ifndef _IMU_H_   /* Include guard */
#define _IMU_H_



/****************************************** Header includes *******************************************/

/* ESP error */
#include "esp_err.h"

/* MPU6050 */
#include "mpu6050.h"



/********************************************* Constants **********************************************/

#define IMU_QUATERNIONS         4
#define IMU_EULER_ANGLES        3



/************************************** Enumeration Definitions ***************************************/

/*!
 * @brief Quaternion index.
 */
typedef enum {
    QUATERNION_X = 0,      // x-axis
    QUATERNION_Y,          // y-axis
    QUATERNION_Z,          // z-axis
    QUATERNION_W           // Angle of rotation
} quaternion_index_t;


/*!
 * @brief Euler angle index.
 */
typedef enum {
    YAW = 0,    // Angle between Sensor x-axis and Earth magnetic North
    PITCH,      // Angle between sensor x-axis and Earth ground plane
    ROLL,       // Angle between sensor y-axis and Earth ground plane
} euler_angle_index_t;



/*************************************** Structure Definitions ****************************************/

typedef struct {
    mpu6050_t mpu6050;
    float quaternion[IMU_QUATERNIONS];
    float euler_angle[IMU_EULER_ANGLES];
} imu_t;



/*********************************** Function Prototype Declarations **********************************/

/*!
 * @brief This public function is used to initialize the imu.
 * 
 * @param[in,out] imu  :Structure instance of imu_t.
 * @param[in] gyro_scale  :Enumeration instance of gyro_scale_t.
 * @param[in] accel_scale  :Enumeration instance of accel_scale_t.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t imu_init(imu_t *imu, gyro_scale_t gyro_scale, accel_scale_t accel_scale);


/*!
 * @brief This public function is used to get quaternions and euler angles.
 * 
 * @param[in,out] imu  :Structure instance of imu_t.
 * @param[in] delta_time_sec  :Delta time between now and last filtering.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t imu_get_filtered_data(imu_t *imu, float delta_time_sec);



#endif  /* End of include guard */

/******************************************************************************************************/

