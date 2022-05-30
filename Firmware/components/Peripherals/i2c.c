
/************************************* Header Includes *************************************/

/* std Lib */
#include <stdint.h>         // For uint8_t definition 
#include <stdbool.h>        // For true/false definition

/* FreeRTOS */
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* ESP32 drivers */
#include <driver/gpio.h>  // GPIO
#include <driver/i2c.h>   // I2C

/* I2C */
#include "i2c.h"



/**************************************** Constants ****************************************/

#define I2C_MASTER_PORT             0         // I2C master port number
#define I2C_ACK_VAL                 0         // I2C Acknowledge
#define I2C_NACK_VAL                1         // I2C No Acknowledge



/************************************* Static Variables ************************************/

static SemaphoreHandle_t s_i2c_mutex = NULL;    // I2C mutex Semaphore reference



/************************************ Macro Definitions ************************************/

#define I2C_LOCK()      do {                                                      \
                              if (!xSemaphoreTake(s_i2c_mutex, portMAX_DELAY)) {  \
                                  return ESP_ERR_TIMEOUT;                         \
                              }                                                   \
                        } while (0)

#define I2C_UNLOCK()    do {                                                      \
                              if (!xSemaphoreGive(s_i2c_mutex)) {                 \
                                  return ESP_FAIL;                                \
                              }                                                   \
                        } while (0)



/************************************ Public Functions ************************************/

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
esp_err_t i2c_init(int sda_io_num, int scl_io_num)
{
  i2c_config_t conf;

  s_i2c_mutex = xSemaphoreCreateMutex();
  if (s_i2c_mutex == NULL) {return ESP_FAIL;}

  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = sda_io_num;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = scl_io_num;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 800000;

  if (i2c_param_config(I2C_MASTER_PORT, &conf) != ESP_OK) {
    return ESP_FAIL;
  }

  if (i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0) != ESP_OK) {
    return ESP_FAIL;
  }

  return ESP_OK;
}


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
esp_err_t i2c_send_data(uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len)
{
  I2C_LOCK();

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  if (i2c_master_start(cmd) != ESP_OK) {
    i2c_cmd_link_delete(cmd);
    return ESP_FAIL;
  }

  if (i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true) != ESP_OK) {
    i2c_cmd_link_delete(cmd);
    return ESP_FAIL;
  }

  if (reg) {
    if (i2c_master_write_byte(cmd, *reg, true) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }
  }
  else {
    i2c_cmd_link_delete(cmd);
    return ESP_FAIL;
  }

  if (data) {
    if (i2c_master_write(cmd, data, len, true) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }
  }
  else {
    i2c_cmd_link_delete(cmd);
    return ESP_FAIL;
  }

  if (i2c_master_stop(cmd) != ESP_OK) {
    i2c_cmd_link_delete(cmd);
    return ESP_FAIL;
  }

  esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_RATE_MS);

  i2c_cmd_link_delete(cmd);

  I2C_UNLOCK();
  
  return err;
}


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
esp_err_t i2c_read_data(uint8_t addr, const uint8_t *reg, uint8_t *data, uint32_t len)
{
  if (len == 0) {
    return ESP_FAIL;
  }

  I2C_LOCK();

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  if (reg) {
    if (i2c_master_start(cmd) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }

    if (i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }

    if (i2c_master_write_byte(cmd, *reg, true) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }

    if (!data) {
      i2c_master_stop(cmd);
      i2c_cmd_link_delete(cmd);
    }
  }
  else {
    i2c_cmd_link_delete(cmd);
    return ESP_FAIL;
  }

  if (data) {
    if (i2c_master_start(cmd) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }

    if (i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }
    
    if (len > 1) {
      if (i2c_master_read(cmd, data, len-1, (i2c_ack_type_t)I2C_ACK_VAL) != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        return ESP_FAIL;
      }
    }

    if (i2c_master_read_byte(cmd, data + len-1, (i2c_ack_type_t)I2C_NACK_VAL) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }

    if (i2c_master_stop(cmd) != ESP_OK) {
      i2c_cmd_link_delete(cmd);
      return ESP_FAIL;
    }
  }
  else {
    i2c_cmd_link_delete(cmd);
    return ESP_FAIL;
  }

  esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_RATE_MS);

  i2c_cmd_link_delete(cmd);
  
  I2C_UNLOCK();

  return err;
}


/******************************************************************************************/