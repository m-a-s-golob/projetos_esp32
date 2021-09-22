#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"

#define LED_PLACA       GPIO_NUM_2
#define BOTAO_1         GPIO_NUM_22
#define LED_CONTROLE    GPIO_NUM_21
#define ADC_POT         ADC1_CHANNEL_4
#define LED_PWM         GPIO_NUM_19

#define TIMER_DIVIDER	(16)                                // divisor de clock de hardware em fator 16x
#define TIMER_SCALE 	(TIMER_BASE_CLK / TIMER_DIVIDER)    // converte o valor em segundos; TIMER_BASE_CLK = 80MHz por padrão

uint32_t contador = 0;
bool led_controle = 0;

// estrutura de referencia para setup e informacao do timer
typedef struct{
    int timer_group;
    int timer_idx;
    int alarm_interval; //VIP
    bool auto_reload;
} esp_timer_info_t;

// estrutura de referencia para eventos do timer (opcional)
typedef struct{
    esp_timer_info_t info;
    uint64_t timer_counter_value;
} esp_timer_event_t;

typedef struct{
	int timer_group;	
	int timer_idx;
	int alarm_interval;
	bool auto_reload;
} exemplo_timer_info_t;

typedef struct{
	exemplo_timer_info_t info;
	uint64_t timer_counter_value;
} exemplo_timer_event_t;

// callback para tratamento da interrupcao do botao
static void IRAM_ATTR botao_isr_handler(void *arg) // não precisa de protótipo por causa do "static" (bloco não compartilhado)
{
	// verifica se o botao 1 foi a fonte da interrupcao
	// 1 - como fazer debounce na leitura da interrupcao de gpio?
	if(BOTAO_1 == (uint32_t) arg)   // typecast para tipo uint32_t - inteiro usado na definicao  dos pinos
	{
		// if(gpio_get_level(BOTAO_1 == 0)
		if(gpio_get_level((uint32_t) arg) == 0)
		{
			contador++;
		}
	}
}

// callback para tratamento de interrupção de timer
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    esp_timer_info_t *info = (esp_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
    
    // carrega os valores do evento timer = podemos usar isso mais tarde, por hora eh soh referencia
    esp_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    // se nao temos auto_reload configurado
    if (!info->auto_reload){
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    // pisca led na ocorrencia de interrupcao de timer - ACAO no determinado tempo (ou quantidade de eventos) desejado
    gpio_set_level(LED_CONTROLE, led_controle);
    led_controle = !led_controle; // inverte o estado do LED para a proxima vez

    return high_task_awoken == pdTRUE; // retorna se precisamos "ceder" ao final da rotina de interrupcao
    // util em troca de contexto
}

// (CPU_APP - ESP32)
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

    // Método discreto de confirguração de GPIOs
    // gpio_reset_pin(LED_PLACA);
    // gpio_set_direction(LED_PLACA, GPIO_MODE_OUTPUT);

    // Método de configuração de pinos usando o gpio_config_t
    gpio_config_t button_config = {
        .intr_type = GPIO_INTR_DISABLE, //sem interrupção por hora
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BOTAO_1),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&button_config);

    // Configuração dos LEDs
    gpio_config_t led_config = {
        .intr_type = GPIO_INTR_NEGEDGE, // interrupção na borda de descida
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PLACA) | (1ULL << LED_CONTROLE),
        .pull_up_en = 0
    };
    gpio_config(&led_config);

	
	gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); // estamos configurando uma rotian de tratamento de interrupcao de baixa prioridade
	gpio_isr_handler_add(BOTAO_1, botao_isr_handler, (void*) BOTAO_1);
	
    // configurar o timer para execucao
    timer_config_t config = {
        .divider = TIMER_DIVIDER,       // fator de escada - divisor por 16
        .counter_dir = TIMER_COUNT_UP,  // timer como contador crescente
        .counter_en = TIMER_PAUSE,      // timer comeca parado
        .alarm_en = TIMER_ALARM_EN,     // timer com alarme (a.k.a. interrupcao)
        .auto_reload = true,            // auto-reload
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    
    uint32_t intervalo_em_segundos = 5;

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); // comeca a contar em 0
    // 5 segundos para comecar a contar os alarmes ou geracao de interrupcao
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, intervalo_em_segundos * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);          // habilita interrupcao do timer

    // calloc - aloca memoria para um vetor de elementos e inicializa todo mundo em 0
    esp_timer_info_t *timer_info = calloc(1, sizeof(esp_timer_info_t));
    // cria a nossa estrutura de interrupcao do timer para referencia dentro da rotina de callback de interrupcao
    timer_info->timer_group = TIMER_GROUP_0;
    timer_info->timer_idx = TIMER_0;
    timer_info->auto_reload = true;
    timer_info->alarm_interval = intervalo_em_segundos;
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0);

    // ate agora tudo foi perfumaria e ritual de configuracao
    // a partir dessa exato momento, o TIMER comeca a contar independentemente da CPU, e gerando interrupcoes assim em diante
    // conforme configurado
    timer_start(TIMER_GROUP_0, TIMER_0);

	uint8_t estado_led = 0;

	while(1)
	{
		vTaskDelay(1000/portTICK_PERIOD_MS);
		printf("Contador: %d\n", contador);
		gpio_set_level(LED_PLACA, estado_led);
		estado_led = !estado_led;
	}
}