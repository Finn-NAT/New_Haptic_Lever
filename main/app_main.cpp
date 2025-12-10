
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
    motorHaptic.calibrate();

    // int64_t lastTime = esp_timer_get_time();
    // uint64_t total_loops = 0;
    // int counter = 0;

    motorHaptic.setup();
    while (1) {
        motorHaptic.loop(); 


        // total_loops++;
        // if (++counter >= 10000) {
        //     int64_t now = esp_timer_get_time();
        //     int64_t elapsed = now - lastTime;
        //     float loopHz = 10000.0 * 1000000.0 / elapsed;
            
        //     printf("Loop frequency: %.2f Hz (%.2f MHz) | Total: %llu loops\n", 
        //            loopHz, loopHz/1000000.0, total_loops);
            
        //     lastTime = now;
        //     counter = 0;
        // }
        // vTaskDelay(100);
    }
}

