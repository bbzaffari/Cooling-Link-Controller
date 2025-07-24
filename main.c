#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "Utils.h"
#include <driver/i2c_master.h>
#include "htu31.h"  
#include <esp_log.h>
#include "lora_proto.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

// === DEFINES =======================================---------------------------------
// === DEFINES DE PINO =======================================
#define ADD_GPIO        GPIO_NUM_32
#define ADD_BUTTON_MASK (1ULL << ADD_GPIO)

#define SUB_GPIO        GPIO_NUM_33
#define SUB_BUTTON_MASK (1ULL << SUB_GPIO)

#define ENTER_GPIO       GPIO_NUM_23
#define ENTER_MASK       (1ULL << ENTER_GPIO)

#define SDA_GPIO GPIO_NUM_17
#define SCL_GPIO GPIO_NUM_22

// Eventos de ENTER (ConfigTaskHandle)
#define ENTER_SHORT  (1UL << 0)
#define ENTER_LONG   (1UL << 1)

// === Outros defines =======================================
#define FALLING     (1UL << 2)/// Eventos de GPIO (tAdd, tSub)
#define RISING      (1UL << 3)/// Eventos de GPIO (tAdd, tSub)

#define CHOSEN_NONE      0xFF

#define SENSORS_COUNT 3

#define LONG_PRESS_START   1000
#define MUTEX_TIMEOUT 20

#define DEBOUNCE_MS   100 // Debounce em ms    
#define LONG_PRESS_MS 3000 // Duração mínima em ms para considerar LONG_PRESS
#define TimeToWait    30000 //Tempo para esperar 30s

portMUX_TYPE mux_local = portMUX_INITIALIZER_UNLOCKED;
//-------------------------------------------------------------------------------------

/// === LIMITS: =============================================--------------------------
// TEMP: -----------------------
#define REPEAT_MS_TEMP       100
static const float MAX_LIMIT_TEMP  = 80.0f;
static const float MIN_LIMIT_TEMP  = 2.0f;
// HUM: -------------------------
#define REPEAT_MS_HUM       100
static const float MAX_LIMIT_HUM  = 96.0f;
static const float MIN_LIMIT_HUM  = 30.0f;
// CO2: -------------------------
#define REPEAT_MS_CO2       100
static const float MAX_LIMIT_CO2  = 5000.0f;
static const float MIN_LIMIT_CO2  = 400.0f;

static const float ONE  =  1.f;
static const float ZERO  = 0.f;
//-------------------------------------------------------------------------------------

// === TIME CONSTANTS =============================================
const TickType_t TIMEOUT = pdMS_TO_TICKS(LONG_PRESS_START);
const TickType_t inOut_TIME = pdMS_TO_TICKS(LONG_PRESS_MS); 
const TickType_t TimeEnter  = pdMS_TO_TICKS(TimeToWait); 

const TickType_t MUTEX_TIMEOUT_TICKS = pdMS_TO_TICKS(MUTEX_TIMEOUT);

static inline uint32_t getTimeMs(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
//-----------------------------------------------------------------------------

// VARIAVEIS GLOBAIS ==============================================------------
static volatile float TEMP_IN = 0;// TEMPERATURA DO SENSOR CAMARA
static volatile float HUM_IN = 0; // HUMIDADE DO SENSOR CAMARA
static volatile float CO2 = 400;  // DIOXIDO DE CARBONO DO SENSOR CAMARA
static volatile float TEMP_OUT = 0;// TEMPERATURA DO SENSOR PORTA(CONTROLADOR)
static volatile float HUM_OUT = 0; // TEMPERATURA DO SENSOR PORTA(CONTROLADOR)
static volatile float DEW = 0;     // DEWPOINT DO SENSOR PORTA(CONTROLADOR)
//static volatile uint8_t light = ;

/* TRANSIENT: VALOR PARA O ADD E O SUB CONFIGURAREM 
* ANTES DE EFETIVER EM ALGUMA VARIAVEL LIMITE DE SENSOR*/
static float TRANSIENT  = 0.0f;

static TaskHandle_t add_handle_task = NULL;  // SUB
static TaskHandle_t sub_handle_task = NULL;  // ADD
static TaskHandle_t ConfigTaskHandle = NULL; // ENTER

static volatile uint32_t lastInterruptTimeEnter = 0; // FOR ENTER
static volatile TickType_t enterPressTick = 0;       // FOR ENTER

static SemaphoreHandle_t xMutex; // Para exclusao mutua entre ENTER, ADD e SUB

static sConfigActivation *CurrentSensor = NULL; // VARIAVLE PARA DEFINIR ATUAL SENSOR SENDO CONFIGURADO
static sConfigElement    *CurrentElement= NULL; // VARIAVLE PARA DEFINIR ATUAL V. SENSOR SENDO CONFIGURADO

static fsm_state_t state_config = IDLE;
static eGlobalState Gstate = NOTHING; 

static volatile BaseType_t HTU31_FOUND = F; 

// VARIAVEIS "extern" dos drivers ===================
SSD1306_t dev_oled;              // FOR OLED DRIVER
i2c_master_bus_handle_t bus;     // FOR HTU31 DRIVER
i2c_master_dev_handle_t dev_htu; // FOR HTU31 DRIVER


//----------------------------------------------------------------------------------------------

// === Structural Comp. =============================================----------
static sConfigActivation configs[SENSORS_COUNT] = {
    {
        .name = "TEMP",
        .elements = {
            // NAME:| value         |MAX            | MIN| pacing|
            { "MAX", MAX_LIMIT_TEMP, &MAX_LIMIT_TEMP, NULL, 0.5f },
            { "RELAY MAX", 0, &ONE, &ZERO, 1},
            { "MIN", MIN_LIMIT_TEMP, NULL, &MIN_LIMIT_TEMP, 0.5f },
            { "RELAY MIN", 0, &ONE, &ZERO, 1}
        },
        .repeat = REPEAT_MS_TEMP,
        .elements_count = 4
    },
    {
        .name = "HUM",
        .elements = {
            // NAME:| value         |MAX            | MIN| pacing|
            { "MAX", MAX_LIMIT_HUM, &MAX_LIMIT_HUM, NULL, 1.f },//0
            { "RELAY MAX", 0, &ONE, &ZERO, 1},                 //1
            { "MIN", MIN_LIMIT_HUM, NULL, &MIN_LIMIT_HUM, 1.f},//2
            { "RELAY MIN", 0, &ONE, &ZERO, 1}                  //3
        },
        .repeat = REPEAT_MS_HUM,
        .elements_count = 4
    },
    {
        .name = "CO2",
        .elements = {
            // NAME:| value         |MAX            | MIN| pacing|
            { "MAX", MAX_LIMIT_CO2, &MAX_LIMIT_CO2, NULL, 50.f },
            { "RELAY MAX", 0, &ONE, &ZERO, 1},
            { "MIN", MIN_LIMIT_CO2, NULL, &MIN_LIMIT_CO2, 50.f },
            { "RELAY MIN", 0, &ONE, &ZERO, 1}
        },
        .repeat = REPEAT_MS_CO2,
        .elements_count = 4
    }
};
//----------------------------------------------------------------------------------------------


// =============================================================================================
// ****************************************** PROTOTIPOS BEGIN *********************************
// =============================================================================================

// Tasks
void tAdd(void* pvParameters);
void tSub(void* pvParameters);
void tConfig(void* pvParameters);
void task_monitor_porta(void *arg);

// Funções auxiliares
sConfigActivation* setChosen(uint8_t chosen);
static void Add(float* n, float pacing, float MIN, float MAX);
static void Sub(float* n, float pacing, float MIN, float MAX);
void init_config_links(void);
void montar_palavra_estado_simples(char* palavra);
void displayAllSensorsPaged(void);
void fDISPLAY(void);

// LoRa handlers
static void handler_LoRa_Rx_Controler(lora_packet_t *pkt);

// MQTT e Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_init_sta(void);
static void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start(void);

// =============================================================================================
// ****************************************** PROTOTIPOS END ***********************************
// =============================================================================================


// FUNCOES PARA TRATAR INTERRUPCAO (ISR) ======================---------------
static void IRAM_ATTR add_isr_handler(void* arg) { // ADD
    if (state_config == CONFIG){
        int level = gpio_get_level(ADD_GPIO);
        BaseType_t xHPW = F;
        if (level == 0) {
            xTaskNotifyFromISR(add_handle_task, FALLING, eSetBits, &xHPW);
        } else if (level == 1) {
            xTaskNotifyFromISR(add_handle_task, RISING, eSetBits, &xHPW);
        }
        portYIELD_FROM_ISR(xHPW);
    }
}

static void IRAM_ATTR sub_isr_handler(void* arg) { // SUB
    //if(CurrentSensor != NULL){
    if (state_config == CONFIG){
        BaseType_t xHPW = F;
        int level = gpio_get_level(SUB_GPIO);
        if (level == 0) {
            xTaskNotifyFromISR(sub_handle_task, FALLING, eSetBits, &xHPW);
        } else if (level == 1) {
            xTaskNotifyFromISR(sub_handle_task, RISING, eSetBits, &xHPW);
        }
        portYIELD_FROM_ISR(xHPW);
    }
}

void IRAM_ATTR enter_isr_handler(void* arg) { // ENTER
    BaseType_t xHigherPriorityTaskWoken = F;

    int level = gpio_get_level(ENTER_GPIO);

    if (level == 0) {
        xTaskNotifyFromISR(ConfigTaskHandle, FALLING, eSetBits, &xHigherPriorityTaskWoken);
        enterPressTick = xTaskGetTickCountFromISR();
    } 
    else {
        TickType_t nowTick = xTaskGetTickCountFromISR();
        TickType_t diffTicks = nowTick - enterPressTick;
        uint32_t diffMs = diffTicks * portTICK_PERIOD_MS;
        if (diffMs < DEBOUNCE_MS)
            return;
        if (diffMs >= LONG_PRESS_MS)
            xTaskNotifyFromISR(ConfigTaskHandle, ENTER_LONG, eSetBits, &xHigherPriorityTaskWoken);
        else
            xTaskNotifyFromISR(ConfigTaskHandle, ENTER_SHORT, eSetBits, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
// FUNCOES PARA TRATAR INTERRUPCAO (ISR) ======================---------------

// FUNCOES AUXILIARES ======================-----------------------------------
sConfigActivation* setChosen(uint8_t chosen) { // SELECIONA SENSOR ATUAL E VARIAVEL(do sensor)
    size_t sensorID = (chosen >> 4) & 0x0F;    // bits 7-4
    size_t elementID = chosen & 0x0F;          // bits 3-0

    if (sensorID < SENSORS_COUNT) {
        CurrentSensor = &configs[sensorID];
        if (elementID < CurrentSensor->elements_count) { // só tem elements[0] (MAX) e elements[1] (MIN)
            CurrentElement = &configs[sensorID].elements[elementID];
            return CurrentSensor;
        } else {
            CurrentElement = NULL; // inválido
        }
    } else {
        CurrentSensor = NULL;
        CurrentElement = NULL;
    }
    return NULL;
}

static void Add(float* n, float pacing, float MIN, float MAX) { // ADD CIRCULAR
    if (*n < MAX)
        *n += pacing;
    else
        *n = MIN;
}

static void Sub(float* n, float pacing, float MIN, float MAX) { // SUB CIRCULAR
    if (*n > MIN)
        *n -= pacing;
    else
        *n = MAX;
}

void init_config_links(void) {// INICIALIZACAO DE ALGUNS VALORES
    configs[0].elements[0].MIN = &configs[0].elements[2].value;  // TEMP: MAX.MIN -> MIN.value
    configs[0].elements[2].MAX = &configs[0].elements[0].value;  // TEMP: MIN.MAX -> MAX.value

    configs[1].elements[0].MIN = &configs[1].elements[2].value;  // HUM: MAX.MIN -> MIN.value
    configs[1].elements[2].MAX = &configs[1].elements[0].value;  // HUM: MIN.MAX -> MAX.value

    configs[2].elements[0].MIN = &configs[2].elements[2].value;  // HUM: MAX.MIN -> MIN.value
    configs[2].elements[2].MAX = &configs[2].elements[0].value;  // HUM: MIN.MAX -> MAX.value
}

void montar_palavra_estado_simples(char* palavra) {
    // Garante que sempre teremos uma string de 6 caracteres + null terminador
    palavra[6] = '\0';

    // TEMP_MIN: bit 0
    palavra[0] = (TEMP_IN < configs[0].elements[2].value && configs[0].elements[3].value > 0.5f) ? '1' : '0';

    // TEMP_MAX: bit 1
    palavra[1] = (TEMP_IN > configs[0].elements[0].value && configs[0].elements[1].value > 0.5f) ? '1' : '0';

    // HUM_MIN: bit 2
    palavra[2] = (HUM_IN < configs[1].elements[2].value && configs[1].elements[3].value > 0.5f) ? '1' : '0';

    // HUM_MAX: bit 3
    palavra[3] = (HUM_IN > configs[1].elements[0].value && configs[1].elements[1].value > 0.5f) ? '1' : '0';

    // CO2_MIN: bit 4
    palavra[4] = (CO2 < configs[2].elements[2].value && configs[2].elements[3].value > 0.5f) ? '1' : '0';

    // CO2_MAX: bit 5
    palavra[5] = (CO2 > configs[2].elements[0].value && configs[2].elements[1].value > 0.5f) ? '1' : '0';
}

void displayAllSensorsPaged(void) {
    char buffer[17]; // tamanho seguro
    uint8_t  line = 0;    // linha atual (0 reservada para o nome do sensor)
    uint8_t module;
    
    for (size_t i = 0; i < SENSORS_COUNT; i++) {
        ///>>>> DISPLAY: --------------------
        memset(buffer, line, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "%s", configs[i].name);
        write_oled(buffer, 0, T, F, 10);
        ///---------------------------------
        for (size_t j=0; j < configs[i].elements_count; j++) {
            module = (j%2);
            line = module+1;

            ///>>>> DISPLAY: --------------------
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "%s: %03.1f", configs[i].elements[j].name, configs[i].elements[j].value);
            write_oled(buffer, line, F, F, 10);
            ///---------------------------------

            // Se a tela atingir 2 elementos, limpa e recomeça
            if (module) vTaskDelay(pdMS_TO_TICKS(3000)); // Espera para visualizar
            
        } 
    }
    fDISPLAY();
}
// FUNCOES AUXILIARES ======================-----------------------------------

// TASKS ======================------------------------------------------------
void tAdd(void* pvParameters) { // TASK DE ADICAO -----------------------------
    eStateCounter state = COUNTER_IDLE;
    uint32_t notified = 0;
    TickType_t waitFor;
    BaseType_t sReturn;
    char buffer[16];

    for (;;) {
        switch(state){
            case COUNTER_IDLE:
                waitFor = portMAX_DELAY;
                break;
            case COUNTER_PRESS: 
                waitFor = TIMEOUT;
                break;
            case COUNTER_QUICK:
                waitFor = pdMS_TO_TICKS(CurrentSensor->repeat);
                break;
            default:
                waitFor = portMAX_DELAY;
                break;
        }
        ESP_LOGI("tADD", "Waiting \n"); // DEBUG
        BaseType_t result = xTaskNotifyWait(0, ULONG_MAX, &notified, waitFor);

        if ((notified & FALLING) && (state == COUNTER_IDLE)) {
            sReturn = xSemaphoreTake(xMutex, MUTEX_TIMEOUT_TICKS);
            if(sReturn == T) {
                state = COUNTER_PRESS;
                ESP_LOGI("tADD", "COULD GET MUTEX"); // DEBUG
            }
            else {
                ESP_LOGI("tADD", "COULD NOT GET MUTEX"); // DEBUG
                continue;
            }
        }
        else if (result == T && state== COUNTER_PRESS) {
            if ((notified & RISING)) {
                Add(&TRANSIENT, CurrentElement->pacing, *(CurrentElement->MIN), *(CurrentElement->MAX));
                state = COUNTER_IDLE;
                ESP_LOGI("tADD", "SHORT"); // DEBUG
                ESP_LOGI("tADD", "T: %f", TRANSIENT); // DEBUG

                ///>>>> DISPLAY: --------------------
                memset(buffer, 0, sizeof(buffer));
                snprintf(buffer, sizeof(buffer), "%s: %03.1f", CurrentElement->name, TRANSIENT);
                write_oled(buffer, 2, F, F, 0);
                ///---------------------------------

                xSemaphoreGive(xMutex);
            }
        }
        else if (result == F && state == COUNTER_PRESS) {
            state = COUNTER_QUICK;
            ESP_LOGI("tADD", "state = COUNTER_QUICK"); // DEBUG
        }
        else if (result == F && state == COUNTER_QUICK) {
            Add(&TRANSIENT, CurrentElement->pacing, *(CurrentElement->MIN), *(CurrentElement->MAX));
            ESP_LOGI("tADD", "T: %f", TRANSIENT); // DEBUG

            ///>>>> DISPLAY: --------------------
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "%s: %03.1f",CurrentElement->name, TRANSIENT);
            write_oled(buffer, 2, F, F, 0);
            ///---------------------------------
        }
        else if (state == COUNTER_QUICK && result == T && (notified & RISING) ){
            state = COUNTER_IDLE;
            xSemaphoreGive(xMutex);  
        }
    }
}

void tSub(void* pvParameters) { // TASK DE SUBTRACAO  -----------------------------
    // Commit TRANSIENT to actual config after RISING edge
    eStateCounter state = COUNTER_IDLE;
    uint32_t notified = 0;
    TickType_t waitFor;
    BaseType_t sReturn;
    char buffer[16];

    for (;;) {
        switch(state){
            case COUNTER_IDLE:
                waitFor = portMAX_DELAY;
                break;
            case COUNTER_PRESS: 
                waitFor = TIMEOUT;
                break;
            case COUNTER_QUICK:
                waitFor = pdMS_TO_TICKS(CurrentSensor->repeat);
                break;
            default:
                waitFor = portMAX_DELAY;
                break;
        }
        ESP_LOGI("tSUB", "Waiting \n");
        BaseType_t result = xTaskNotifyWait(0, ULONG_MAX, &notified, waitFor);

        if ((notified & FALLING) && (state == COUNTER_IDLE)) {
            sReturn = xSemaphoreTake(xMutex, MUTEX_TIMEOUT_TICKS);
            if(sReturn == T) {
                state = COUNTER_PRESS;
                ESP_LOGI("tSUB", "COULD GET MUTEX"); // DEBUG
            }
            else {
                ESP_LOGI("tSUB", "COULD NOT GET MUTEX"); // DEBUG
                continue;
            }
        }
        else if (result == T && state== COUNTER_PRESS) {
            if ((notified & RISING)) {
                Sub(&TRANSIENT, CurrentElement->pacing, *(CurrentElement->MIN), *(CurrentElement->MAX));
                state = COUNTER_IDLE;
                ESP_LOGI("tSUB", "SHORT"); // DEBUG
                ESP_LOGI("tSUB", "T: %f", TRANSIENT); // DEBUG

                ///>>>> DISPLAY: --------------------
                memset(buffer, 0, sizeof(buffer));
                snprintf(buffer, sizeof(buffer), "%s: %03.1f", CurrentElement->name, TRANSIENT);
                write_oled(buffer, 2, F, F, 0);
                ///---------------------------------

                xSemaphoreGive(xMutex);
            }
        }
        else if (result == F && state == COUNTER_PRESS) {
            state = COUNTER_QUICK;
            ESP_LOGI("tSUB", "state = COUNTER_QUICK"); // DEBUG
        }
        else if (result == F && state == COUNTER_QUICK) {
            Sub(&TRANSIENT, CurrentElement->pacing, *(CurrentElement->MIN), *(CurrentElement->MAX));
            ESP_LOGI("tSUB", "T: %f", TRANSIENT); // DEBUG

            ///>>>> DISPLAY: --------------------
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "%s: %03.1f",CurrentElement->name, TRANSIENT);
            write_oled(buffer, 2, F, F, 0);
            ///---------------------------------

        }
        else if (state == COUNTER_QUICK && result == T && (notified & RISING) ){
            state = COUNTER_IDLE;
            xSemaphoreGive(xMutex);  
        }
    }
}

void tConfig(void* pvParameters) {// Task principal: FSM CONFIG de MAX E MIN dos sensores com “timeout”  -- ENTER
    //0x<variavel><subtipo>
    const static uint8_t configspointer[SENSORS_COUNT*4] = 
    {0X00, 0X01, 0x02, 0x03,0X10, 0X11, 0X12, 0X13, 0x20, 0x21, 0x22, 0x23}; // ORDEM DE CONFIGURACAO

    size_t size = sizeof(configspointer) / sizeof(configspointer[0]);
    uint32_t notified = 0;
    TickType_t waitTicks;
    BaseType_t sReturn;
    BaseType_t result;
    static int var_index = 0;        
    char temp_element[6];
    char temp_sensor[6];
    char buffer[18];
    float temp = 0.0f;

    for (;;) {
        switch (state_config) {
            case CONFIG:// Se estamos em CONFIG, aguardamos por notificações por até 30 s.
                waitTicks = TimeEnter; 
                ESP_LOGI("tENTER", " TimeEnter"); // DEBUG
                break;
            default:// Se estamos em DISPLAY, aguardamos indefinidamente (portMAX_DELAY).
                waitTicks = portMAX_DELAY;
                ESP_LOGI("tENTER"," portMAX_DELAY"); // DEBUG
                break;
        }

        result = xTaskNotifyWait(0, ULONG_MAX, &notified, waitTicks);// Aguarda evento (short ou long) ou timeout
        ESP_LOGI("tENTER", " NOTIFIED ");
        if (state_config == CONFIG) {
            if (result == T){
                if((notified & FALLING)){
                    ESP_LOGI("tENTER"," state_config == CONFIG"); // DEBUG
                    sReturn = xSemaphoreTake(xMutex, MUTEX_TIMEOUT_TICKS);
                    ESP_LOGI("tENTER", "AQUIRE MUTEX"); // DEBUG
                    if(sReturn == F) continue;
                    else xSemaphoreGive(xMutex);
                } 	
            }
            else{
                ESP_LOGI("tENTER", " TIMEOUT"); // DEBUG
                sReturn = xSemaphoreTake(xMutex, portMAX_DELAY);
                if (CurrentElement != NULL) {
                    CurrentElement->value = TRANSIENT;
                }
                xSemaphoreGive(xMutex);
                state_config = FINISHED;
            }
        }

        if ((result == T) && state_config == IDLE && (notified & ENTER_LONG)) {
            Gstate = SETTING_ON;
            // *****  Long press em DISPLAY -> entra em DISPLAY_SET ********
            ESP_LOGI("tENTER"," state_config == IDLE"); // DEBUG
            setChosen(configspointer[var_index]);

            ///>>>> DISPLAY: --------------------
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "CONFIG");
            write_oled(buffer, 1, T, F, 1000);
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "%s", CurrentSensor->name);
            write_oled(buffer, 1, T, F, 0);
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "%s %03.1f",CurrentElement->name, CurrentElement->value);
            write_oled(buffer, 2, F, F, 0);
            ///---------------------------------

            state_config = CONFIG;
            TRANSIENT = CurrentElement->value;
            ESP_LOGI("tENTER", "TRANSIET %s %s start at: %f", CurrentSensor->name, CurrentElement->name, CurrentElement->value); // DEBUG
            ESP_LOGI("tENTER", "TRANSIET %f| CurrentElement->value: %f", TRANSIENT, CurrentElement->value); // DEBUG
            ESP_LOGI("tENTER", "RELEASE MUTEX"); // DEBUG
            ESP_LOGI("tENTER", " RECEIVED A LONG PRESS"); // DEBUG
            xSemaphoreGive(xMutex);
        }
        else if (result == T && (notified & ENTER_SHORT) && state_config == CONFIG) {
            CurrentElement->value = TRANSIENT;

            ///>>>> DISPLAY: --------------------
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "SET");
            write_oled(buffer, 3, F, F, 300);
            ///---------------------------------

            ESP_LOGI("tENTER", "TRANSIET %s %s finishing at: %f", CurrentSensor->name, CurrentElement->name, CurrentElement->value); // DEBUG
            ESP_LOGI("tENTER", "TRANSIET %f| CurrentElement->value: %f", TRANSIENT, CurrentElement->value); // DEBUG 
            if (var_index < size-1) {
                var_index++;    
                setChosen(configspointer[var_index]);  
                TRANSIENT = CurrentElement->value;
                ESP_LOGI("tENTER", "TRANSIET %s %s start at: %f", CurrentSensor->name, CurrentElement->name, CurrentElement->value); // DEBUG
                ESP_LOGI("tENTER", "TRANSIET %f| CurrentElement->value: %f", TRANSIENT, CurrentElement->value); // DEBUG
                ESP_LOGI("tENTER", "CONFIGURAÇÃO: avançando para índice %d", (int)var_index); // DEBUG

                ///>>>> DISPLAY: --------------------          
                memset(buffer, 0, sizeof(buffer));
                snprintf(buffer, sizeof(buffer), "%s", CurrentSensor->name);
                write_oled(buffer, 1, T, F, 0);
                memset(buffer, 0, sizeof(buffer));
                snprintf(buffer, sizeof(buffer), "%s %03.1f", CurrentElement->name, CurrentElement->value);
                write_oled(buffer, 2, F, F, 1000);
                ///---------------------------------

                xSemaphoreGive(xMutex);   
                ESP_LOGI("tENTER", "RELEASE MUTEX\n"); // DEBUG
                        
            }
            else{
                setChosen(CHOSEN_NONE);
                state_config = FINISHED;
                var_index = 0;
            }
            
        }
        else if ((result == F) || (result == T && (notified & ENTER_LONG) && state_config == CONFIG) || (state_config == FINISHED)){ 
            if (CurrentSensor != NULL){
                snprintf(temp_sensor, sizeof(temp_sensor), "%s", CurrentSensor->name);
                snprintf(temp_element, sizeof(temp_sensor), "%s", CurrentElement->name);
                temp = CurrentElement->value;
                setChosen(CHOSEN_NONE);
                var_index = 0;
                

                ///>>>> DISPLAY: --------------------
                memset(buffer, 0, sizeof(buffer));
                snprintf(buffer, sizeof(buffer), "SET");
                write_oled(buffer, 3, F, F, 300);
                ///---------------------------------
            }

            ///>>>> DISPLAY: --------------------
            memset(buffer, 0, sizeof(buffer));
            snprintf(buffer, sizeof(buffer), "EXITING SETTING");
            write_oled(buffer, 1, T, F, 1000);
            ///---------------------------------

            displayAllSensorsPaged();
            
            state_config = IDLE;  
            Gstate = NOTHING;
        }
    }
}
// TASKS ======================------------------------------------------------

// ================================================================================================
// ******************************************* LORA INICIO ****************************************
// ================================================================================================
// === DEFINES DE LORA =======================================
/*ID é um byte adiquirido do xor dos ultimos 2 bytes do MAC*/
#define O_NOVO    0x84 // O ID DO ESP32 LORA NOVO 
#define S_DISPLAY 0xA2 // O ID DO ESP32 LORA COM O DISPLAY QUEBRADO
#define O_VELHO   0x42 // O ID DO ESP32 LORA VELHO


#define DOOR_GPIO        GPIO_NUM_13      // Botão da "porta"
#define DOOR_DEBOUNCE_MS 200              // Tempo de debounce
#define DOOR_IRQ_EDGE    GPIO_INTR_ANYEDGE

#define CLOSE pdTRUE
#define OPEN pdFALSE
static volatile BaseType_t DOOR_STATE;  // true = aberta
static volatile BaseType_t ACK_PENDENTE;

uint8_t ID_DESTINO = S_DISPLAY;

TaskHandle_t task_handle_monitor_porta = NULL;

void IRAM_ATTR door_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(task_handle_monitor_porta, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void task_monitor_porta(void *arg)
{
    char TAG[] = "MONITOR PORTA";
    char payload[64];

    while (1) {
        // Simples e direto: aguarda evento de notificação
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        while (Gstate != NOTHING) vTaskDelay(pdMS_TO_TICKS(5000));

        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(6000)) == pdTRUE) {
            Gstate = TX;

            BaseType_t nivel = gpio_get_level(DOOR_GPIO);
            DOOR_STATE = nivel;

            ESP_LOGI(TAG, "Interrupção: Porta %s", nivel ? "ABERTA" : "FECHADA"); // DEBUG

            snprintf(payload, sizeof(payload), "DEWPOINT:20.0C|DOOR:%s", nivel ? "OPEN" : "CLOSE"); // DEBUG

            ESP_LOGI(TAG, "Payload: %s", payload); // DEBUG

            for (int i = 0; i < 3; i++) {
                ACK_PENDENTE = lora_send_structured(payload, ID_DESTINO);
                vTaskDelay(pdMS_TO_TICKS(2000));
                if (!ACK_PENDENTE) {
                    ESP_LOGI(TAG, "SUCESSO!!!!!!!!!!!!!!!!"); // DEBUG
                    break;
                } else {
                    ESP_LOGI(TAG, "FALHA!!!!!!!!!!!!!!!!"); // DEBUG
                }
            }

            Gstate = NOTHING;
            xSemaphoreGive(xMutex);
        } else {
            ESP_LOGW(TAG, "Falha ao obter mutex"); // DEBUG
        }
    }
}

// HANDLER PARA RECEBER LIDAR COM RX
static void handler_LoRa_Rx_Controler(lora_packet_t *pkt) {

    if ((pkt->type) == LORA_TYPE_DATA){    char TAG[] = "RX LORA";
        ESP_LOGI(TAG, "Recebido: %.*s", pkt->len, pkt->payload); // DEBUG
        float temp_t, hum_t;
        // Cópia local segura
        char buffer[64] = {0};
        memcpy(buffer, pkt->payload, pkt->len < 63 ? pkt->len : 63);

        // Parsing
        char *temp_str = strstr(buffer, "T:");
        char *hum_str  = strstr(buffer, "H:");

        if (!temp_str || !hum_str) {
            //ESP_LOGW(TAG, "Formato inválido (esperado T:...|H:...)");
            send_nack(pkt->id);
            return;
        }
        
        send_ack(pkt->id);// TX ( mas no core 0)

        temp_t = strtof(temp_str + 2, NULL);
        hum_t  = strtof(hum_str + 2, NULL);
        ESP_LOGI(TAG, "Temp RX: %.2f°C | Umidade RX: %.2f%%", temp_t, hum_t); // DEBUG

        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            TEMP_IN = temp_t;
            HUM_IN = hum_t;
            xSemaphoreGive(xMutex);
            ESP_LOGI(TAG, "Atualizado TEMP_IN: %.2f, HUM_IN: %.2f", TEMP_IN, HUM_IN); // DEBUG
        }
    }

    else if((pkt->type) == LORA_TYPE_ACK) {
        ACK_PENDENTE = pdFALSE;
        ESP_LOGI("DEFAULT_HANDLER", "ACK recebido de %d", pkt->id); // DEBUG
    }

    else if ((pkt->type) ==LORA_TYPE_NACK){
        ESP_LOGW("DEFAULT_HANDLER", "NACK recebido de %d", pkt->id); // DEBUG
    }      
}
// ================================================================================================
// ******************************************** LORA FIM ******************************************
// ================================================================================================

// ================================================================================================
// ******************************************* MQTT INICIO ****************************************
// ================================================================================================

esp_mqtt_client_handle_t client;

// Define o GPIO do LED
#define LED_GPIO GPIO_NUM_25
#define WIFI_SSID "ERB"
#define WIFI_PASS "04111999"
// #define MOSQUITTO_IP "mqtt://192.168.15.3:1883"
#define MOSQUITTO_IP "mqtt://192.168.137.1:1883"

#define BOARD "ESP32"
#define TYPE "CONTROLER"
// Handler de evento Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    char TAG[] = "WIFI_TEST";
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi desconectado, tentando reconectar..."); // DEBUG
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado com sucesso, IP: " IPSTR, IP2STR(&event->ip_info.ip)); // DEBUG
    }
}

void wifi_init_sta(void)
{
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("MQTT", "wifi_init_sta finalizado. Tentando conectar a SSID: %s", WIFI_SSID); // DEBUG

    // Aguarda conexão
    esp_err_t err;
    wifi_ap_record_t ap_info;
    while (1) {
        err = esp_wifi_sta_get_ap_info(&ap_info);
        if (err == ESP_OK) {
            ESP_LOGI("wifi_connect", "Conectado ao AP: %s (RSSI: %d)", ap_info.ssid, ap_info.rssi); // DEBUG
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// Callback do MQTT (modelo novo correto!)
static void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
        break;

    case MQTT_EVENT_DATA:
        break;

    default:
        break;
    }
}

// Inicializa o cliente MQTT
void mqtt_app_start(void)
{
    char client_id[39];

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    snprintf(client_id, sizeof(client_id), "%s_%s_%02X%02X%02X%02X%02X%02X", BOARD, TYPE,  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI("MQTT", "Client ID gerado: %s", client_id); // DEBUG

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MOSQUITTO_IP
            }
        },
        .credentials = {
            .client_id = client_id
        }
    };

    client = esp_mqtt_client_init(&mqtt_cfg);

    // Usar o callback correto
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, NULL);

    esp_mqtt_client_start(client);
}
// ================================================================================================
// ******************************************** MQTT FIM ******************************************
// ================================================================================================

void fDISPLAY(void){ // DISPLAY PRINCIPAL
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        Gstate = DISPLAY;
        char buffer[18]={}; 
        /// HTU:-------------------------------------------------------------------------
        float t, rh;
        if(HTU31_FOUND == T) {
            if (htu31_measure_all_combined(dev_htu, &t, &rh) == ESP_OK) {
                TEMP_OUT = t;
                HUM_OUT = rh;
                DEW = calculate_dew_point(t, rh);
            }
            else{
                TEMP_OUT = 0;
                HUM_OUT = 0;
            }
        }
        //-------------------------------------------------------------------------------
        write_oled("  IN /<|>/ OUT  ", 0, T, F, 0);

        memset(buffer, 0, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "%03.1fC %01.0f|%1.0f %03.1fC", TEMP_IN, configs[0].elements[1].value, configs[0].elements[3].value, TEMP_OUT);
        write_oled(buffer, 1, F, F, 0);

        // Exibe HUM
        memset(buffer, 0, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "%03.1f%% %01.0f|%1.0f %03.1f%%", HUM_IN, configs[1].elements[1].value, configs[1].elements[3].value, HUM_OUT);
        write_oled(buffer, 2, F, F, 0);

        // Exibe CO2
        memset(buffer, 0, sizeof(buffer));
        snprintf(buffer, sizeof(buffer), "%04.0f  %01.0f|%01.0f %04.1fC", CO2, configs[2].elements[1].value, configs[2].elements[3].value, DEW);
        write_oled(buffer, 3, F, F, 0);
        
        Gstate = NOTHING;

        char estado[7];  // 6 bits + terminador
        montar_palavra_estado_simples(estado);
        ESP_LOGI("MQTT", "%s", estado); // DEBUG
        esp_mqtt_client_publish(client, "sensor/estado", estado, 0, 1, 0);
        xSemaphoreGive(xMutex);
    }
}
// MAIN ======================-------------------------------------------------
void app_main(void) {
    // === Inicialização drivers ===------------------------------------
    /*OLED*/
    init_display(); 
    /*HTU*/
    int ret= htu31_init(I2C_NUM_0, SDA_GPIO, SCL_GPIO, 100000, &bus, &dev_htu);
    if (ret == 0) HTU31_FOUND = T;
    //------------------------------------------------------------------
    
    // === Inicialização GPIO ===--------------------------------------
    gpio_config_t io_conf_add_sub = {};
    gpio_config_t io_conf_enter = {};
    //________________________________________________________________
    io_conf_add_sub.intr_type    = GPIO_INTR_ANYEDGE;
    io_conf_add_sub.mode         = GPIO_MODE_INPUT;
    io_conf_add_sub.pin_bit_mask = ADD_BUTTON_MASK | SUB_BUTTON_MASK;
    io_conf_add_sub.pull_up_en   = GPIO_PULLUP_ENABLE;
    //________________________________________________________________
    io_conf_enter.intr_type    = GPIO_INTR_ANYEDGE; //ou GPIO_INTR_POSEDGE;?
    io_conf_enter.mode         = GPIO_MODE_INPUT;
    io_conf_enter.pin_bit_mask = ENTER_MASK;
    io_conf_enter.pull_up_en   = GPIO_PULLUP_ENABLE;
    //________________________________________________________________
    gpio_config(&io_conf_add_sub);
    gpio_config(&io_conf_enter);
    //------------------------------------------------------------------

    // === Inicialização OUTROS ===------------------------------------
    init_config_links();
    xMutex = xSemaphoreCreateMutex();
    //------------------------------------------------------------------
    /// LORA:
    lora_setup();  // Inicializa SPI, LoRa, interrupções, tarefa RX
    lora_set_packet_handler(handler_LoRa_Rx_Controler); 
    vTaskDelay(pdMS_TO_TICKS(5000));  // Aguarda sistema estabilizar
    ESP_LOGI("MAIN", "Inicializando controlador..."); // DEBUG
    
    gpio_config_t io_conf_door = {
        .pin_bit_mask = 1ULL << DOOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = DOOR_IRQ_EDGE,
    };
    gpio_config(&io_conf_door);
    

    // xTaskCreatePinnedToCore(task_monitor_porta, "porta", 4096, NULL, 10, NULL, 1);
    xTaskCreate(task_monitor_porta, "porta", 4096, NULL, 10, &task_handle_monitor_porta);
    //-----------------LORA ----------------------------
    //---------------- MQTT ----------------------------
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());


    wifi_init_sta();

    mqtt_app_start();
    //---------------- MQTT ----------------------------
    // === Inicialização TASKS ===------------------------------------
    xTaskCreate(tAdd, "TaskAdd", 4096, NULL, 4, &add_handle_task);
    xTaskCreate(tSub, "TaskSub", 4096, NULL, 4, &sub_handle_task);
    xTaskCreate(tConfig, "TaskConfig", 4096, NULL, 7, &ConfigTaskHandle);
    //------------------------------------------------------------------
    
    // === Inicialização ISR ===--------------------------------------
    // Usa handler padrão compartilhado
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ADD_GPIO, add_isr_handler, NULL);
    gpio_isr_handler_add(SUB_GPIO, sub_isr_handler, NULL);
    gpio_isr_handler_add(ENTER_GPIO, enter_isr_handler, NULL);
    gpio_isr_handler_add(DOOR_GPIO, door_isr_handler, NULL);
    //------------------------------------------------------------------

    for(;;) {
        if (state_config == IDLE) {
            fDISPLAY();
            vTaskDelay(pdMS_TO_TICKS(6000));// Aguarda 1 min antes de atualizar de novo
        }
        else {
            // Só faz delay enquanto tENTER está em CONFIG -> para não atrapalhar
            vTaskDelay(pdMS_TO_TICKS(100000));
        }

    }
}
//============================================================================================


