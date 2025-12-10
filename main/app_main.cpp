
#include "Arduino.h"
#include <SPI.h>
#include <SimpleFOC.h>

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "Motor_Haptic.h"

motor_info_t motor_info = {32, 33, 25, 11, FOC_ZERO_ELECTRIC_ANGLE};
MotorHaptic motorHaptic(motor_info, PIN_SPI_CS);

extern "C" void app_main() {

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
        }
    }
}

