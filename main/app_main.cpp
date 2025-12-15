
#include "Arduino.h"
#include <SPI.h>
#include <SimpleFOC.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "Motor_Haptic.h"

motor_info_t motor_info = {32, 33, 25, 11, FOC_ZERO_ELECTRIC_ANGLE};
MotorHaptic motorHaptic(motor_info, PIN_SPI_CS);

#define UPDATED_LEVER_ID        0x01111110
#define COMMAND_LEVER_ID        0x01111111

typedef enum {
    DISABLE_FUNCTION = 0,
    ENABLE_FUNCTION_1,
    ENABLE_FUNCTION_2,
    ENABLE_FUNCTION_3,
    ENABLE_FUNCTION_4,
    ENABLE_FUNCTION_5
} rx_task_function_t;

#define TX_GPIO_NUM             (gpio_num_t)13
#define RX_GPIO_NUM             (gpio_num_t)15
#define TX_TASK_PRIO            9       //Sending task priority
#define RX_TASK_PRIO            8       //Receiving task priority
#define EXAMPLE_TAG             "LEVER"

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//Filter all other IDs except MSG_ID
// static const twai_filter_config_t f_config = {.acceptance_code = (MSG_ID << 21),
//                                               .acceptance_mask = ~(TWAI_STD_ID_MASK << 21),
//                                               .single_filter = true
//                                              };

// Chỉ nhận 2 extended ID: 0x01111110 và 0x01111111
// Extended ID được shift left 3 bits trong hardware register
// acceptance_code: base ID (0x01111110) << 3
// acceptance_mask: mask out bit thay đổi. 0 = must match, 1 = don't care
//                  Bit cuối của ID khác nhau, nên mask bit đó = 1
static const twai_filter_config_t f_config = {
    .acceptance_code = (0x01111110 << 3),  // Base ID
    .acceptance_mask = (1 << 3),            // Don't care bit 0 của ID (bit 3 trong register)
    .single_filter = true
};
//Set to NO_ACK mode due to self testing with single module
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t rx_sem;
static bool function_5_enabled = false;
static float position_value = 0.0f;

/* --------------------------- Tasks and Functions -------------------------- */

// Chuyển 1 float thành 4 bytes (little-endian)
void float_to_bytes(float value, uint8_t* bytes) {
    union {
        float f;
        uint8_t b[4];
    } data;
    data.f = value;
    memcpy(bytes, data.b, 4);
}

// Chuyển 4 bytes thành 1 float (little-endian)
float bytes_to_float(const uint8_t* bytes) {
    union {
        float f;
        uint8_t b[4];
    } data;
    memcpy(data.b, bytes, 4);
    return data.f;
}

static void twai_transmit_task(void *arg)
{
    twai_message_t tx_msg = {
        // Message type and format settings
        .extd = 1,              // Extended Format message (29-bit ID)
        .rtr = 0,               // Send a data frame
        .ss = 1,                // Single shot
        .self = 0,              // Normal transmission (not loopback)
        .dlc_non_comp = 0,      // DLC is less than 8
        // Message ID and payload
        .identifier = UPDATED_LEVER_ID,
        .data_length_code = 8,
        .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    };

    while (1) {
        xSemaphoreTake(rx_sem, portMAX_DELAY);
        float_to_bytes(position_value, &tx_msg.data[0]);
        ESP_ERROR_CHECK(twai_transmit(&tx_msg, portMAX_DELAY));
        xSemaphoreGive(rx_sem);
        //ESP_LOGI(EXAMPLE_TAG, "twai_transmit_task running on core %d", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void twai_receive_task(void *arg)
{
    twai_message_t rx_message;
    twai_message_t tx_message;
    rx_task_function_t rx_func;

    while(1){
        //Receive message and print message data
        ESP_ERROR_CHECK(twai_receive(&rx_message, portMAX_DELAY));
        ESP_LOGI(EXAMPLE_TAG, "Msg received\tID 0x%lx\tData = %d", rx_message.identifier, rx_message.data[0]);
        if (rx_message.extd && rx_message.data_length_code == 8 && rx_message.identifier == COMMAND_LEVER_ID) {   
            printf("Processing received message...\n");

            // rx_func = (rx_task_function_t)rx_message.data[0];
            // xQueueSend(rx_task_queue, &rx_func, portMAX_DELAY);

            xSemaphoreTake(rx_sem, portMAX_DELAY);
            tx_message = rx_message;
            tx_message.data[0] = 0xFF;
            ESP_ERROR_CHECK(twai_transmit(&tx_message, portMAX_DELAY));
            xSemaphoreGive(rx_sem);
        }else if(rx_message.extd && rx_message.data_length_code == 8 && rx_message.identifier == UPDATED_LEVER_ID && function_5_enabled){   
            printf("Processing received message for position update...\n");
            
        }
    }
}

extern "C" void app_main() {

    rx_sem = xSemaphoreCreateBinary();
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_function_t));
    xSemaphoreGive(rx_sem);

    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");
    vTaskDelay(pdMS_TO_TICKS(1000));

    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, 1);

    // Tắt watchdog cho task này
    esp_task_wdt_deinit();
    
    // Đặt priority = 24 (thấp hơn Timer task = 25)
    // Timer task sẽ ưu tiên cao nhất, code này thứ 2
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 2);
    
    printf("Starting maximum speed...\n");
    printf("Task priority: %d (Timer task = 25 is highest)\n", 
           uxTaskPriorityGet(NULL));
  
  
    initArduino();
    motorHaptic.init();

    while (1) {
        motorHaptic.calibrate();
        motorHaptic.setup();
        while(1) {
            motorHaptic.loop();

            xSemaphoreTake(rx_sem, portMAX_DELAY); 
            position_value = motorHaptic.getPosition();
            xSemaphoreGive(rx_sem);
        }
    }
}

