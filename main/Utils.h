
#pragma once
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include <math.h>

#define SDA_PIN     GPIO_NUM_4
#define SCL_PIN     GPIO_NUM_15
#define RESET_PIN   GPIO_NUM_16
#define OLED_WIDTH 128
#define FONT_WIDTH 8
#define MAX_CHARS_PER_LINE (OLED_WIDTH / FONT_WIDTH)
#define F pdFALSE
#define T pdTRUE

//---------------------------------------------------------------------------------------
// Control structures
typedef struct {//-- ConfigElements
    const char* name;
    float value;
    float* MAX;
    float* MIN;
    float pacing; 
} sConfigElement;

typedef struct {
    const char* name;
    sConfigElement elements[4];
    uint32_t repeat;
    uint8_t elements_count;
} sConfigActivation;

typedef enum{
    COUNTER_PRESS,
    COUNTER_QUICK,
    COUNTER_IDLE,
} eStateCounter;

typedef enum {
    IDLE,
    CONFIG,
    FINISHED,
} fsm_state_t;

typedef enum {// Not important, yet....
    DISPLAY,
    SETTING_ON,
    NOTHING,
    RX, 
    TX,
} eGlobalState;

typedef struct {//-- ConfigVars
    const char* name;
    float value;
    float* MAX;
    float* MIN;
} sAmbientVars;

//---------------------------------------------------------------------------------------
// DISPLAY WOPPER
extern SSD1306_t dev_oled;

void init_display();
void write_oled(const char *mensagem, int pagina, BaseType_t limpar, BaseType_t invertido, int DELAY);
void display_text_line(const char *texto, int page, int start_col);
void oled_show_message(const char *prefix, const char *buf, int len);
size_t initConfigPointers(uint8_t* dest, size_t max_size);

//---------------------------------------------------------------------------------------
// Auxiliary functions

float calculate_dew_point(float temp_celsius, float rh_percent);

//---------------------------------------------------------------------------------------
// Future use
typedef enum{
    UI8,
    UI16,
    UI32,
    I8,
    FLT
} eTypes;

typedef enum{
    AUTOMATIC,
    MANUAL
} eLight;
