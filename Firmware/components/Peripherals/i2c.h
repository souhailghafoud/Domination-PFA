#ifndef I2C_H       /* Include guard */
#define I2C_H



/****************************************** Header includes *******************************************/

#include "esp_err.h"



/***************************** Function Prototype Declarations *****************************/

/*!
 * @brief This public function is used to initialize the I2C port
 *        as master.
 * 
 * @param[in] sda_io_num  :GPIO number for I2C data line
 * @param[in] scl_io_num  :GPIO number for I2C clock line
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t i2c_init(int sda_io_num, int scl_io_num);


/*!
 * @brief This public function is used to send data to a slave
 *        through the I2C port.
 *
 * @param[in] addr  :Slave address
 * @param[in] reg   :Register to write
 * @param[in] data  :Data to send
 * @param[in] len   :Data lenght
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t i2c_send_data(uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len);


/*!
 * @brief This public function is used to read data from a slave
 *        through the I2C port.
 *
 * @param[in] addr  :Slave address
 * @param[in] reg   :Register to read
 * @param[in] data  :Data read from reg
 * @param[in] len   :Data lenght
 *
 * @return Result of the function execution status.
 * @retval ESP_OK -> Success  /  ESP_FAIL -> Error
 */
esp_err_t i2c_read_data(uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len);



#endif  /* End of include guard */

/*********************************************************************************************/