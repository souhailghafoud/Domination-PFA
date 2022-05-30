
/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>         // For uint8_t definition 
#include <stdbool.h>        // For true/false definition

/* Math */
#include <math.h>           // For math functions

/* String */
#include <string.h>         // For memcpy

/* IMU */
#include "imu.h"



/********************************************* Constants **********************************************/

#define PI                      3.141592653589793238f



/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used to filter the accelerometer and gyroscope data
 *        into quaternions with the Madgwick algorithm.
 *
 * @param[in] deltat  :Integration interval for both filter schemes.
 * @param[in] q  :Quaternions.
 * @param[in] ax  :Accelerometer X-axis.
 * @param[in] ay  :Accelerometer Y-axis.
 * @param[in] az  :Accelerometer Z-axis.
 * @param[in] gyrox  :Gyroscope X-axis.
 * @param[in] gyroy  :Gyroscope Y-axis.
 * @param[in] gyroz  :Gyroscope Z-axis.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
static esp_err_t imu_madgwick_algorithm(float deltat,
                                        float q[4],
                                        const float accel_data[MPU6050_AXIS],
                                        const float gyro_data[MPU6050_AXIS])
{
    float ax = accel_data[AXIS_X];
    float ay = accel_data[AXIS_Y];
    float az = accel_data[AXIS_Z];
    float gyrox = gyro_data[AXIS_X];
    float gyroy = gyro_data[AXIS_Y];
    float gyroz = gyro_data[AXIS_Z];
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz;
    float gbiasx = 0.0f, gbiasy = 0.0f, gbiasz = 0.0f;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // parameters for 6 DoF sensor fusion calculations
    float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return ESP_FAIL; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
    
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

    return ESP_OK;
}



/***************************************** Public Functions *******************************************/

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
esp_err_t imu_init(imu_t *imu, gyro_scale_t gyro_scale, accel_scale_t accel_scale)
{
    imu->mpu6050.gyro_scale = GYRO_FS_500DPS;
    imu->mpu6050.accel_scale = ACCEL_FS_4G;

    /* Initialize MPU6050 sensor */
    if (ESP_OK != mpu6050_init(imu->mpu6050)) {
        return ESP_FAIL;
    }
    
    imu->quaternion[QUATERNION_X] = 1.0f;
    imu->quaternion[QUATERNION_Y] = 0.0f;
    imu->quaternion[QUATERNION_Z] = 0.0f;
    imu->quaternion[QUATERNION_W] = 0.0f;

    return ESP_OK;
}


/*!
 * @brief This public function is used to get quaternions and euler angles.
 * 
 * @param[in,out] imu  :Structure instance of imu_t.
 * @param[in] delta_time_sec  :Delta time between now and last filtering.
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t imu_get_filtered_data(imu_t *imu, float delta_time_sec)
{    
    /* Fetch new accelerometer data from MPU6050 */
    if (ESP_OK != mpu6050_read_accel(&imu->mpu6050)) {
        return ESP_FAIL;
    }
    
    /* Fetch new gyroscope data from MPU6050 */
    if (ESP_OK != mpu6050_read_gyro(&imu->mpu6050)) {
        return ESP_FAIL;
    }

    /* Get quaternions */
    if (ESP_OK != imu_madgwick_algorithm(delta_time_sec,
                                         imu->quaternion,
                                         imu->mpu6050.accel_data,
                                         imu->mpu6050.gyro_data)) {
        return ESP_FAIL;
    }

    /* short name local variable for readability */
    float q[IMU_QUATERNIONS] = {1.0f, 0.0f, 0.0f, 0.0f};
    memcpy(q, imu->quaternion, sizeof(imu->quaternion));

    /* Get Euler angles */
    imu->euler_angle[YAW]   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    imu->euler_angle[PITCH] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    imu->euler_angle[ROLL]  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    /* Convert rad to deg (1rad * 180/PI = 57.2958°) */
    imu->euler_angle[PITCH] *= 57.2958f;
    imu->euler_angle[YAW]   *= 57.2958f;
    imu->euler_angle[ROLL]  *= 57.2958f;

    return ESP_OK;
}




/******************************************************************************************************/