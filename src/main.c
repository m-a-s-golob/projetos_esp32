#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#define LED_PLACA       GPIO_NUM_2
#define BOTAO_1         GPIO_NUM_22
#define LED_CONTROLE    GPIO_NUM_21

uint32_t contador = 0;

void app_main(void)
{
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

    //Método discreto de confirguração de GPIOs
    //gpio_reset_pin(LED_PLACA);
    //gpio_set_direction(LED_PLACA, GPIO_MODE_OUTPUT);

    //Método de configuração de pinos usando o gpio_config_t
    gpio_config_t button_config = {
        .intr_type = GPIO_INTR_DISABLE, //sem interrupção por hora
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOTAO_1),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&button_config);

    //Configuração dos LEDs
    gpio_config_t led_config = {
        .intr_type = GPIO_INTR_DISABLE, //sem interrupção por hora
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PLACA) | (1ULL << LED_CONTROLE),
        .pull_up_en = 0
    };
    gpio_config(&led_config);

	uint32_t botao_1_anterior = 0;

	while(1)
	{
		vTaskDelay(30/portTICK_PERIOD_MS);
		//resposta em borda de descida
		if(botao_1_anterior  == 1 && gpio_get_level(BOTAO_1) == 0)
		{
			//desempenhamos alguma ação ao apertar o botão
			contador++;
		}
		printf("Contador: %d\n", contador);
		botao_1_anterior  = gpio_get_level(BOTAO_1);
	}
}