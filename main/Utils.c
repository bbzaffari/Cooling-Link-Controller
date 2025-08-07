


#include "Utils.h"

#define TAG "OLED_DISPLAY"

//---------------------------------------------------------------------------------------
// DISPLAY

void init_display() {
    ESP_LOGI(TAG, "Inicializando I2C: SDA=%d, SCL=%d, RESET=%d", SDA_PIN, SCL_PIN, RESET_PIN);
    i2c_master_init(&dev_oled, SDA_PIN, SCL_PIN, RESET_PIN);
    ssd1306_init(&dev_oled, 128, 32);
    ssd1306_clear_screen(&dev_oled, pdFALSE);
    ssd1306_contrast(&dev_oled, 0xCF);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void write_oled(const char *mensagem, int pagina, BaseType_t limpar, BaseType_t invertido, int DELAY) {
    if (pagina < 0 || pagina >= 8) {
        ESP_LOGW(TAG, "Página inválida (%d). dev_olede ser entre 0 e 7.", pagina);
        return;
    }

    int len = strlen(mensagem);
    if (len > MAX_CHARS_PER_LINE) {
        ESP_LOGW(TAG, "Mensagem muito longa (%d chars). Máximo permitido: %d", len, MAX_CHARS_PER_LINE);
        len = MAX_CHARS_PER_LINE;
    }

    if (limpar) {
        ssd1306_clear_screen(&dev_oled, pdFALSE);
    }

    ssd1306_display_text(&dev_oled, pagina, mensagem, len, invertido);
    
    if (DELAY>0) vTaskDelay(pdMS_TO_TICKS(DELAY));
}

void display_text_line(const char *texto, int page, int start_col) {
    if (page < 0 || page >= dev_oled._pages) {
        ESP_LOGW(TAG, "Página inválida (%d). dev_olede estar entre 0 e %d", page, dev_oled._pages - 1);
        return;
    }

    int texto_len = strlen(texto);
    if (texto_len + start_col > MAX_CHARS_PER_LINE) {
        texto_len = MAX_CHARS_PER_LINE - start_col;
    }

    uint8_t composed[OLED_WIDTH] = {0};

    for (int i = 0; i < texto_len; i++) {
        const uint8_t *glyph = font8x8_basic_tr[(uint8_t)texto[i]];
        for (int col = 0; col < FONT_WIDTH; col++) {
            int pos = (start_col + i) * FONT_WIDTH + col;
            if (pos < OLED_WIDTH) {
                composed[pos] = glyph[col];
            }
        }
    }

    i2c_display_image(&dev_oled, page, 0, composed, OLED_WIDTH);
}

void oled_show_message(const char *prefix, const char *buf, int len) {
    static char last_text[17] = {0};
    char texto[17] = {0};
    char clean_str[17] = {0};

    const int max_total_len = 16;
    int prefix_len = strlen(prefix);
    if (prefix_len >= max_total_len) prefix_len = max_total_len - 1;

    int max_payload = max_total_len - prefix_len - 1;
    if (len > max_payload) len = max_payload;

    snprintf(texto, sizeof(texto), "%s%.*s", prefix, len, buf);
    snprintf(clean_str, sizeof(clean_str), "%-*s", (int)strlen(texto), prefix);

    if (strcmp(texto, last_text) != 0) {
        write_oled(clean_str, 2, F, F, 0);
        write_oled(texto, 2, F, F, 0);
        strcpy(last_text, texto);
    }
}

//---------------------------------------------------------------------------------------
// Auxiliary functions

float calculate_dew_point(float temp_celsius, float rh_percent) {
    const float a = 17.62f;
    const float b = 243.12f;

    float alpha = ((a * temp_celsius) / (b + temp_celsius)) + logf(rh_percent / 100.0f);
    float dew_point = (b * alpha) / (a - alpha);

    return dew_point;
}