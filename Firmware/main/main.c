/*******************************************************************************************************
**
** @brief     eMMG Tech  -  PFA-v0.3.0 Firmware
**
** @copyright Copyright © 2022 eMMG Tech, All rights reserved.
** 
** @file	  main.cpp
** @author    Souhail Ghafoud
** @date	  April 29, 2022
** @version	  0.1.0
**
*******************************************************************************************************/



/****************************************** Header includes *******************************************/

/* std Lib */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/* ESP32 drivers */
#include "driver/gpio.h"    // GPIO
#include "driver/timer.h"   // Timer

/* I2C */
#include "i2c.h"

/* IMU */
#include "imu.h"



/********************************************* Constants **********************************************/

#define FIRMWARE_VERSION                "0.1.0"                     // Firmware version

#define PRO_CORE                        0                           // ESP32 Core 0
#define APP_CORE                        1                           // ESP32 Core 1

#define IMU_SAMPLING_TASK_STACK         (1024 * 3)                  // Stack size in bytes
#define IMU_SAMPLING_TASK_PRIORITY      (tskIDLE_PRIORITY + 3)      // Priority level
#define IMU_SAMPLING_TASK_CORE          APP_CORE                    // CPU core ID

#define COMMS_TASK_STACK                (1024 * 1)                  // Stack size in bytes
#define COMMS_STATUS_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)      // Priority level
#define COMMS_STATUS_TASK_CORE          APP_CORE                    // CPU core ID

#define DISPLAY_TASK_STACK              (1024 * 2)                  // Stack size in bytes
#define DISPLAY_STATUS_TASK_PRIORITY    (tskIDLE_PRIORITY + 2)      // Priority level
#define DISPLAY_STATUS_TASK_CORE        APP_CORE                    // CPU core ID

#define TASK_IMU_SAMPLING_BIT           ( 1 << 0 )                  // Task IMU sampling EventBit
#define TASK_COMMS_BIT                  ( 1 << 1 )                  // Task comms EventBit
#define TASK_DISPLAY_BIT                ( 1 << 2 )                  // Task display EventBit
#define ALL_SYNC_BITS                   ( TASK_IMU_SAMPLING_BIT  | \
                                          TASK_COMMS_BIT  | \
                                          TASK_DISPLAY_BIT)

#define DISPLAY_DATA_QUEUE_LEN          10                          // Display data Queue length
#define IMU_DATA_QUEUE_LEN              10                          // IMU data Queue length

#define IMU_SAMPLING_PERIOD_US          100000                      // IMU sampling period in us

#define TIMER_CLK_DIVIDER               80                          // Hardware timer clock divider

#define HIGH			                1
#define LOW 			                0

/* I2C pins */
#define PIN_I2C_SCL                     GPIO_NUM_22
#define PIN_I2C_SDA                     GPIO_NUM_21

/* SPI pins */
#define PIN_SPI_MOSI                    GPIO_NUM_23
#define PIN_SPI_MISO                    GPIO_NUM_19
#define PIN_SPI_CLK                     GPIO_NUM_18

/* TFT pins */
#define PIN_TFT_SS                      GPIO_NUM_5
#define PIN_TFT_CD                      GPIO_NUM_17
#define PIN_TFT_RESET                   GPIO_NUM_16

/* SD pins */
#define PIN_SD_SS                       GPIO_NUM_15



/************************************** Enumeration Definitions ***************************************/

/*!
 * @brief Team id.
 */
typedef enum {
    TEAM_A = 0,
    TEAM_B,
    TEAM_NONE
} team_id_t;


/*!
 * @brief Player life state.
 */
typedef enum {
    PLAYER_INGAME = 0,
    PLAYER_ELIMINATED    
} player_life_state_t;


/*!
 * @brief CP id.
 */
typedef enum {
    CP_A = 0,
    CP_B,
    CP_C
} cp_id_t;


/*!
 * @brief CP capture state.
 */
typedef enum {
    CP_LOST = 0,
    CP_CAPTURED,
    CP_CAPTURING    
} cp_capture_state_t;


/*!
 * @brief CP capture state.
 */
typedef enum {
    BAT_NOT_CHARGING = 0,
    BAT_CHARGING,
    BAT_CHARGED    
} bat_charging_state_t;



/*************************************** Structure Definitions ****************************************/

/*!
 * @brief Player information.
 */
typedef struct {
    player_life_state_t life_state;
    uint32_t eliminated_count;
} player_info_t;


/*!
 * @brief CP information.
 */
typedef struct {
    cp_id_t id;
    cp_capture_state_t capture_state;
    team_id_t dominant_team;
} cp_info_t;


/*!
 * @brief Battery information.
 */
typedef struct {
    uint8_t SOT;
    bat_chrgn_state_t chrgn_status;
} bat_info_t;


/*!
 * @brief Display data.
 */
typedef struct {
    player_info_t player_info;
    cp_info_t cp_info;
    bat_info_t bat_info;    
} display_data_t;



/***************************************** Static Variables *******************************************/

static EventGroupHandle_t s_sys_init_event_group = NULL;    // System init EventGroup handle

static QueueHandle_t s_imu_data_queue = NULL;               // IMU data Queue handle
static QueueHandle_t s_display_data_queue = NULL;           // Display data Queue handle

static SemaphoreHandle_t s_timer0_semaphore = NULL;         // Timer0 Semaphore handle



/*********************************************** ISRs *************************************************/


/*!
 * @brief This interrupt service routine is used to wake imu_sampling_task()
 *        when the timer reaches it's alarm value.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void IRAM_ATTR tg0_alarm_isr(void *arg)
{
    timer_spinlock_take(TIMER_GROUP_0);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;   // Higher priority task flag

    /* Retrieve the interrupt status from the 
     * timer that reported the interrupt.
     * */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
    
    if (timer_intr & TIMER_INTR_T0) {     // 100ms elapsed
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0); // Clear interrupt
        xSemaphoreGiveFromISR(s_timer0_semaphore, &xHigherPriorityTaskWoken);
    }
    else if (timer_intr & TIMER_INTR_T1) {
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1); // Clear interrupt
    }

    /* Wake up higher priority task immediately */
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }

    timer_spinlock_give(TIMER_GROUP_0);
}



/***************************************** Private Functions ******************************************/

/*!
 * @brief This private function is used to delay a task for a period
 *        of time in milliseconds.
 */
static void delay_ms(uint32_t period_ms)
{
    vTaskDelay((period_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}



/****************************************** App Core Tasks ********************************************/

/*!
 * @brief This internal task is used to get filtered IMU data and send 
 *        it to comms_task() through s_imu_data_queue every 100 ms.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void imu_sampling_task(void *arg)
{
    imu_t imu = {0};                    // Structure instance of imu_data_t
    timer_config_t timer_config = {};   // Structure instance of timer_config_t
    uint64_t timer_val_us = 0;          // To store timer 0 counter value
    uint64_t last_timer_val_us = 0;     // To store last timer 0 counter value
    float delta_time_sec = 0;           // To store delta time in seconds
    
    /* Create binary semaphores */
    s_timer0_semaphore = xSemaphoreCreateBinary();
    
    /* Create timer configuration */
    timer_config.divider = TIMER_CLK_DIVIDER;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_PAUSE;
    timer_config.alarm_en = TIMER_ALARM_EN;
    timer_config.auto_reload = TIMER_AUTORELOAD_DIS;

    /* Init Group0 Timer0 */
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);              // Init timer with config above
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0,                 // Set counter value to 0
                            0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0,                   // Set alarm value
                          IMU_SAMPLING_PERIOD_US);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);                      // Enable interupt
    timer_isr_register(TIMER_GROUP_0, TIMER_0, tg0_alarm_isr,       // Register ISR
                       NULL, ESP_INTR_FLAG_IRAM, NULL);

    /* Create IMU data queue */
    s_imu_data_queue = xQueueCreate(IMU_DATA_QUEUE_LEN, sizeof(imu_t));
    
    /* Initialize IMU */
    if (ESP_OK != imu_init(&imu, GYRO_FS_500DPS, ACCEL_FS_4G)) {
        printf("ERROR : IMU init failed\n\n");
    }

    /* Sync all tasks for initialization */
    xEventGroupSync(s_sys_init_event_group,
                    TASK_IMU_SAMPLING_BIT,
                    ALL_SYNC_BITS,
                    portMAX_DELAY);

    /* Start timer 0 */
    timer_start(TIMER_GROUP_0, TIMER_0);

	while (1) {
        /* Wait for alarm event from timer 0 */
        xSemaphoreTake(s_timer0_semaphore, portMAX_DELAY);
        
        /* Get Timer 0 counter value */
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_val_us);

        /* Set next alarm */
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0,
                              (timer_val_us + IMU_SAMPLING_PERIOD_US));
        timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);

        /* Update delta time */
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_val_us);
        delta_time_sec = ((float)(timer_val_us - last_timer_val_us) / 1000000.0f);
        last_timer_val_us = timer_val_us;

        /* Get new filtered data from IMU */
        if (ESP_OK != imu_get_filtered_data(&imu, delta_time_sec)) {
            printf("ERROR : IMU get filtered data failed\n\n");
        }

        /* Enqueue new IMU data */
        xQueueSend(s_imu_data_queue, &imu, 0);
	}
}


/*!
 * @brief This internal task is used to fetch filtered IMU data from
 *        imu_sampling_task() through s_imu_data_queue and send it to
 *        the UpperArm device via UART. Also, this task listens on the
 *        UART RX line for new info on the CPs and Player status, the
 *        battery charging status and its state-of-charge.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void comms_task(void *arg)
{
    imu_t imu = {0};                // Structure instance of imu_data_t
    display_data_t display = {0};   // Structure instance of display_data_t

	while (1) {
        /* Dequeue new IMU data */
        xQueueReceive(s_imu_data_queue, &imu, portMAX_DELAY);

        
        /* Enqueue new IMU data */
        xQueueSend(s_display_data_queue, &display, 0);
    }
}


/*!
 * @brief This internal task is used to display the CPs and Player status,
 *        the battery charging status and its state-of-charge received from
 *        comms_task() through s_comms_rx_data_queue.
 * 
 * @param[in] arg  :Not used.
 * 
 * @return Nothing.
 */
static void display_task(void *arg)
{
    display_data_t display = {0};       // Structure instance of display_data_t
    
    /* Create display data queue */
    s_display_data_queue = xQueueCreate(DISPLAY_DATA_QUEUE_LEN, sizeof(display_data_t));
    
	while (1) {
        /* Dequeue new data to display */
        xQueueReceive(s_display_data_queue, &display, portMAX_DELAY);

    }
}


/****************************************** Pro Core Tasks ********************************************/

void app_main(void)
{
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    printf("**********************************************\n");
    printf("eMMG Tech  -  PFA-v%s\n", FIRMWARE_VERSION);
    printf("**********************************************\n\n");
    
    /* Init I2C Peripheral */
    if (ESP_OK != i2c_init(PIN_I2C_SDA, PIN_I2C_SCL)) {
        printf("ERROR : I2C init failed\n\n");
    }

    /* Create system initialization EventGroup */
    s_sys_init_event_group = xEventGroupCreate();

    /* Create a task for IMU data sampling */
    xTaskCreatePinnedToCore(&imu_sampling_task,          
                            "IMU Sampling",
                            IMU_SAMPLING_TASK_STACK,
                            NULL,
                            IMU_SAMPLING_TASK_PRIORITY,
                            NULL, 
                            IMU_SAMPLING_TASK_CORE);
                            
    /* Create a task to communicate with the UpperArm device */
    xTaskCreatePinnedToCore(&comms_task,
                            "Comms",
                            COMMS_TASK_STACK,
                            NULL,
                            COMMS_TASK_PRIORITY,
                            NULL,
                            COMMS_TASK_CORE);
        
    /* Create a task to display info on the TFT */
    xTaskCreatePinnedToCore(&display_task,
                            "Display",
                            DISPLAY_TASK_STACK,
                            NULL,
                            DISPLAY_TASK_PRIORITY,
                            NULL,
                            DISPLAY_TASK_CORE);
}



/******************************************************************************************************/