
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
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_ota_ops.h"

#include "Motor_Haptic.h"
#include "utils.h"

motor_info_t motor_info = {25, 33, 32, 11, FOC_ZERO_ELECTRIC_ANGLE};
MotorHaptic motorHaptic(motor_info, PIN_SPI_CS);

// TWAI (CAN bus) definitions------------------------
// ID for left lever updates and commands
#define UPDATED_LEFT_LEVER_ID        0x01000000
#define COMMAND_LEFT_LEVER_ID        0x01000001
// ID for right lever updates and commands
#define UPDATED_RIGHT_LEVER_ID       0x02000000
#define COMMAND_RIGHT_LEVER_ID       0x02000001
// ID for azipod updates and commands
#define UPDATED_AZIPOD_ID            0x03000000
#define COMMAND_AZIPOD_ID            0x03000001
//---------------------------------------------------

#define TX_GPIO_NUM             (gpio_num_t)13
#define RX_GPIO_NUM             (gpio_num_t)15
#define ADC_PIN                 (gpio_num_t)36

#define EXAMPLE_TAG             "LEVER"

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NO_ACK);

static SemaphoreHandle_t rx_sem;
static SemaphoreHandle_t tx_sem;

static float position_value = 0.0f;
static float received_position_value = 0.0f;

static bool change_mode_requested = false;
static LoopMode current_mode = FUNCTION_MODE_DEFAULT;
static LeverType current_lever_type = LEFT_LEVER;
static uint32_t update_id = UPDATED_LEFT_LEVER_ID;
static uint32_t command_id = COMMAND_LEFT_LEVER_ID; 

#define BOOTLOADER_TRIGGER_BYTE_1   0xFF
#define BOOTLOADER_TRIGGER_BYTE_2   0x05
#define BOOTLOADER_TRIGGER_BYTE_3   0xFF
#define BOOTLOADER_TRIGGER_BYTE_4   0x50
static bool bootloader_triggered = false;

/* --------------------------- Tasks and Functions -------------------------- */

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
        .identifier = update_id,
        .data_length_code = 8,
        .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    };

    while (1) {
        xSemaphoreTake(rx_sem, portMAX_DELAY);
        float_to_bytes(position_value, &tx_msg.data[0]);

        xSemaphoreTake(tx_sem, portMAX_DELAY);
        tx_msg.data[4] = static_cast<uint8_t>(current_mode);
        tx_msg.data[5] = static_cast<uint8_t>(motorHaptic.getMotorState());
        xSemaphoreGive(tx_sem);

        if(!motorHaptic.function_demo_enabled){
            esp_err_t result = twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
        
            if (result == ESP_OK) {
                // Gửi thành công
            } else if (result == ESP_ERR_INVALID_STATE) {
                // Driver bị bus-off hoặc stopped, thử recover
                // ESP_LOGW(EXAMPLE_TAG, "TWAI in invalid state");
            } else if (result == ESP_ERR_TIMEOUT) {
                // TX queue đầy, bỏ qua
                ESP_LOGW(EXAMPLE_TAG, "TWAI TX timeout");
            } else {
                ESP_LOGE(EXAMPLE_TAG, "TWAI transmit error: 0x%x", result);
            }            
        }
        
        xSemaphoreGive(rx_sem);

        //ESP_LOGI(EXAMPLE_TAG, "twai_transmit_task running on core %d", xPortGetCoreID());

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void twai_receive_task(void *arg)
{
    twai_message_t rx_message;
    twai_message_t tx_message;

    while(1){
        //Receive message and print message data
        esp_err_t result = twai_receive(&rx_message, pdMS_TO_TICKS(1000));
        
        if (result != ESP_OK) {
            if (result == ESP_ERR_TIMEOUT) {
                // Timeout bình thường, tiếp tục loop
                continue;
            } else if (result == ESP_ERR_INVALID_STATE) {
                ESP_LOGW(EXAMPLE_TAG, "TWAI RX invalid state");
                continue;
            } else {
                ESP_LOGE(EXAMPLE_TAG, "TWAI receive error: 0x%x", result);
                continue;
            }
        }
        
        //ESP_LOGI(EXAMPLE_TAG, "Msg received\tID 0x%lx\tData = %d", rx_message.identifier, rx_message.data[0]);
        if (rx_message.extd && rx_message.data_length_code == 8 && rx_message.identifier == command_id) {   
            printf("Processing received message...\n");

            xSemaphoreTake(tx_sem, portMAX_DELAY);
            if(rx_message.data[0] == BOOTLOADER_TRIGGER_BYTE_1 &&
               rx_message.data[1] == BOOTLOADER_TRIGGER_BYTE_2 &&
               rx_message.data[2] == BOOTLOADER_TRIGGER_BYTE_3 &&
               rx_message.data[3] == BOOTLOADER_TRIGGER_BYTE_4){
                bootloader_triggered = true;
                xSemaphoreGive(tx_sem);
                continue;
            }
            uint8_t raw = rx_message.data[0];
            int8_t signed_raw = static_cast<int8_t>(raw);
            LoopMode mode = static_cast<LoopMode>(signed_raw);
            if(current_lever_type == AZIPOD){
                if(mode < FUNCTION_AZI_MODE_1 || mode > FUNCTION_MODE_DEFAULT){
                    printf("Received invalid mode %d, ignoring...\n", mode);
                    xSemaphoreGive(tx_sem);
                    continue;
                }
            }else{
                if(mode < FUNCTION_MODE_DEFAULT || mode > FUNCTION_MODE_4){
                    printf("Received invalid mode %d, ignoring...\n", mode);
                    xSemaphoreGive(tx_sem);
                    continue;
                }
            }

            if(current_mode != mode){
                change_mode_requested = true;
                current_mode = mode;
                printf("Requested mode change to %d\n", current_mode);
            }
            if(current_mode == FUNCTION_MODE_5){
                motorHaptic.function_demo_enabled = true;
            }else{
                motorHaptic.function_demo_enabled = false;
            }
            xSemaphoreGive(tx_sem);

            xSemaphoreTake(rx_sem, portMAX_DELAY);
            tx_message = rx_message;
            tx_message.data[0] = 0xFF;
            
            esp_err_t tx_result = twai_transmit(&tx_message, pdMS_TO_TICKS(100));
            if (tx_result != ESP_OK) {
                ESP_LOGW(EXAMPLE_TAG, "Failed to send reply: 0x%x", tx_result);
            }
            
            xSemaphoreGive(rx_sem);
        }else if(rx_message.extd && rx_message.data_length_code == 8 && rx_message.identifier == update_id && motorHaptic.function_demo_enabled){   
            //printf("Processing received message for position update...\n");
            xSemaphoreTake(tx_sem, portMAX_DELAY);
            received_position_value = bytes_to_float(&rx_message.data[0]);
            xSemaphoreGive(tx_sem);

        }
    }
}

// static void demo_function_selftest(void *arg){
//     // This task runs on core 1
//     ESP_LOGI(EXAMPLE_TAG, "demo_function_selftest running on core %d", xPortGetCoreID());
//     float delta = 1.0f * DEG_TO_RAD;

//     while(1){
//         if(motorHaptic.function_demo_enabled){
//             xSemaphoreTake(tx_sem, portMAX_DELAY);
//             received_position_value += delta;
//             if(received_position_value > 60.0f * DEG_TO_RAD){
//                 delta = -1.0f * DEG_TO_RAD;
//             }
//             else if(received_position_value < -60.0f * DEG_TO_RAD){
//                 delta = 1.0f * DEG_TO_RAD;
//             }
//             xSemaphoreGive(tx_sem);
//             vTaskDelay(pdMS_TO_TICKS(10));
            
//         }
//         else{
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
        
//     }
// }

static void BacktoCANBootloader()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *factory = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_FACTORY,
        NULL);

    if (!running) {
        ESP_LOGE(EXAMPLE_TAG, "Running partition is NULL (unexpected)");
        return;
    }

    ESP_LOGI(EXAMPLE_TAG, "Button pressed. Running: label=%s subtype=0x%02x offset=0x%08x",
        running->label, running->subtype, (unsigned)running->address);

    if (running->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
        ESP_LOGI(EXAMPLE_TAG, "Already running factory. Do nothing.");
        return;
    }

    if (!factory) {
        ESP_LOGE(EXAMPLE_TAG, "No factory app partition found. Can't rollback.");
        return;
    }

    ESP_LOGW(EXAMPLE_TAG, "Switching boot partition to factory (label=%s offset=0x%08x) and restarting...",
             factory->label, (unsigned)factory->address);

    esp_err_t err = esp_ota_set_boot_partition(factory);
    if (err != ESP_OK) {
        ESP_LOGE(EXAMPLE_TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Restarting to factory partition...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    fflush(stdout);
    esp_restart();
}

static void check_bootloader_trigger_task(void *arg){

    while(1){
        xSemaphoreTake(tx_sem, portMAX_DELAY);
        if(bootloader_triggered){
            printf("Bootloader triggered! Restarting to bootloader...\n");
            BacktoCANBootloader();
        }
        xSemaphoreGive(tx_sem);
        vTaskDelay(pdMS_TO_TICKS(500));        
    }
}

extern "C" void app_main() {

    rx_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(rx_sem);

    tx_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(tx_sem);

    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");
    vTaskDelay(pdMS_TO_TICKS(1000));

    //xTaskCreatePinnedToCore(demo_function_selftest, "Demo Function Selftest", 4096, NULL, 8, NULL, 1);

    // Tắt watchdog cho task này
    esp_task_wdt_deinit();
    
    // Đặt priority = 24 (thấp hơn Timer task = 25)
    // Timer task sẽ ưu tiên cao nhất, code này thứ 2
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 2);
    
    printf("Starting maximum speed...\n");
    printf("Task priority: %d (Timer task = 25 is highest)\n", 
           uxTaskPriorityGet(NULL));
  
  
    initArduino();
    
    pinMode(ADC_PIN, INPUT);
    int adc_value = 0;
    for (int i = 0; i < 30; i++) {
        adc_value += analogReadMilliVolts(ADC_PIN);
        // printf("ADC Read %d: %d\n", i + 1, adc_value);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    adc_value /= 30;
    // adc_value = 1500; // For testing purpose, force to AZIPOD
    printf("Average ADC Value: %d mV\n", adc_value);
    if(adc_value < 1100){
        current_lever_type = LEFT_LEVER;
        update_id = UPDATED_LEFT_LEVER_ID;
        command_id = COMMAND_LEFT_LEVER_ID;
    }else if(adc_value > 2200){
        current_lever_type = RIGHT_LEVER;
        update_id = UPDATED_RIGHT_LEVER_ID;
        command_id = COMMAND_RIGHT_LEVER_ID;
    }else{
        current_lever_type = AZIPOD;
        update_id = UPDATED_AZIPOD_ID;
        command_id = COMMAND_AZIPOD_ID;
        current_mode = FUNCTION_AZI_MODE_1;
    }
    printf("Detected lever type: %d\n", current_lever_type);

    vTaskDelay(pdMS_TO_TICKS(500));

    xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(check_bootloader_trigger_task, "Check_BL_Trigger", 4096, NULL, 4, NULL, 1);
    
    motorHaptic.init(current_lever_type);

    while (1) {
        // if(bootloader_triggered){
        //     printf("Bootloader triggered! Restarting to bootloader...\n");
        //     BacktoCANBootloader();
        // }
        motorHaptic.setLoopMode(current_mode);
        printf("Set loop mode to %d\n", current_mode);

        motorHaptic.calibrate(current_lever_type);
        if(motorHaptic.getMotorState() == HAPTIC_MOTOR_ERROR){
            printf("Calibration failed! Motor in error state. Retrying...\n");
            while(1){
                xSemaphoreTake(tx_sem, portMAX_DELAY);
                if(change_mode_requested){
                    change_mode_requested = false;
                    xSemaphoreGive(tx_sem);
                    break;
                }
                xSemaphoreGive(tx_sem);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }else{
            printf("Calibration successful! Motor ready.\n");
            motorHaptic.setup();
            while(1) {
                xSemaphoreTake(tx_sem, portMAX_DELAY);
                motorHaptic.TargetPositionDemo = received_position_value;
                xSemaphoreGive(tx_sem);
            
                motorHaptic.loop();

                xSemaphoreTake(rx_sem, portMAX_DELAY); 
                position_value = motorHaptic.getPosition();
                xSemaphoreGive(rx_sem);

                xSemaphoreTake(tx_sem, portMAX_DELAY);
                if(change_mode_requested){
                    change_mode_requested = false;
                    xSemaphoreGive(tx_sem);
                    break;
                }
                xSemaphoreGive(tx_sem);
                // vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

