#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#define LED_PLACA   GPIO_NUM_2

uint32_t contador = 0;

void app_main(void) {
    printf("Inicializando Esquenta ESP32... \n");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("Executando %s com %d CPU Cores - WiFi %s %s \n", 
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    
    printf("Revisao de Silicio: %d \n", chip_info.revision);

    printf("%d MB de Flash %s \n", 
        (spi_flash_get_chip_size() / (1024*1024)),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embarcada" : "externa");

    gpio_reset_pin(LED_PLACA);
    gpio_set_direction(LED_PLACA, GPIO_MODE_OUTPUT);

    while(1) {
        printf("----------------------- \n");
        printf("Contador: %d \n", contador);
        printf("Desligando LED...\n");
        gpio_set_level(LED_PLACA, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Ligando LED...\n");
        gpio_set_level(LED_PLACA, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        contador++;
        if (contador == 4294967295) {
            contador = 0;
        }
    }
}