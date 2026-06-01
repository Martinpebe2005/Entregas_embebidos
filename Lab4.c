#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "bluetooth_spp.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "rfid.h"

#define PIN_LED_R   GPIO_NUM_18
#define PIN_LED_G GPIO_NUM_19
#define PIN_LED_B  GPIO_NUM_21

#define PIN_BUZZER            GPIO_NUM_12
#define CFG_BUZZER_SPEED_MODE       LEDC_LOW_SPEED_MODE
#define CFG_BUZZER_TIMER      LEDC_TIMER_0
#define CFG_BUZZER_CHANNEL    LEDC_CHANNEL_0
#define CFG_BUZZER_RESOLUTION LEDC_TIMER_10_BIT
#define CFG_BUZZER_FREQ_SHORT  2100
#define CFG_BUZZER_FREQ_LONG    950

#define PIN_RESET   17
#define PIN_RFID_MISO 26
#define PIN_RFID_MOSI 25
#define PIN_RFID_SCLK 27
#define PIN_RFID_CS   14

#define PORT_RTC_I2C  I2C_NUM_0
#define PIN_RTC_SDA   GPIO_NUM_23
#define PIN_RTC_SCL   GPIO_NUM_22

#define PORT_DISP_I2C I2C_NUM_1
#define PIN_DISP_SDA  GPIO_NUM_33
#define PIN_DISP_SCL  GPIO_NUM_32

#define ADDR_RTC_DS1307   0x68
#define ADDR_LCD_PCF8574  0x27

#define DELAY_RFID_SCAN_MS      25
#define DELAY_LCD_REFRESH_MS     160
#define TIMEOUT_MSG_EVENT_MS     2100
#define LEN_MSG_ACTIVO_MAX      16

#define NS_NVS_APP      "app_state"
#define KEY_NVS_LAST_MSG   "last_msg"

#define BIT_LCD_RS  0x01
#define BIT_LCD_RW  0x02
#define BIT_LCD_EN  0x04
#define BIT_LCD_BL  0x08

static TickType_t ts_bloqueo_lectura_rfid = 0;

static void lcd_i2c_write_byte(uint8_t byte_data)
{
    i2c_cmd_handle_t h_cmd = i2c_cmd_link_create();
    i2c_master_start(h_cmd);
    i2c_master_write_byte(h_cmd, (ADDR_LCD_PCF8574 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h_cmd, byte_data | BIT_LCD_BL, true);
    i2c_master_stop(h_cmd);
    i2c_master_cmd_begin(PORT_DISP_I2C, h_cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h_cmd);
}

static void lcd_enable_pulse(uint8_t byte_data)
{
    lcd_i2c_write_byte(byte_data);
    esp_rom_delay_us(1);
    lcd_i2c_write_byte(byte_data | BIT_LCD_EN);
    esp_rom_delay_us(1);
    lcd_i2c_write_byte(byte_data & ~BIT_LCD_EN);
    esp_rom_delay_us(50);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t byte_out = (nibble << 4);
    if (mode) byte_out |= BIT_LCD_RS;
    lcd_enable_pulse(byte_out);
}

static void lcd_send_byte(uint8_t value, uint8_t mode)
{
    lcd_send_nibble(value >> 4,   mode);
    lcd_send_nibble(value & 0x0F, mode);
}

static void lcd_command(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02)
        vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_put_char(char ch)
{
    lcd_send_byte((uint8_t)ch, 1);
}

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_send_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x02, 0);
    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_command(0x80 | (col + row_offsets[row]));
}

static void lcd_write_line(uint8_t row, const char *text)
{
    char buf[21];
    snprintf(buf, sizeof(buf), "%-20.20s", text);
    lcd_set_cursor(row, 0);
    for (int idx = 0; idx < 20; idx++) lcd_put_char(buf[idx]);
}

SemaphoreHandle_t g_mutex_i2c = NULL;

static void configurar_i2c_bus(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda,
        .scl_io_num       = scl,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(port, &cfg);
    i2c_driver_install(port, cfg.mode, 0, 0, 0);
}

typedef enum {
    ESTADO_BLOQUEADO,
    ESTADO_CONCEDIDO,
    ESTADO_DENEGADO,
    ESTADO_ACTIVO,
} estado_sistema_t;

typedef struct {
    uint8_t  seg;
    uint8_t  min;
    uint8_t  hora;
    uint8_t  dia_semana;
    uint8_t  dia;
    uint8_t  mes;
    uint16_t anio;
} rtc_fecha_hora_t;

static SemaphoreHandle_t g_mutex_estado = NULL;

static volatile bool   flag_acceso_denegado       = false;
static estado_sistema_t  estado_sistema          = ESTADO_BLOQUEADO;
static bool            flag_ui_dirty              = true;
static TickType_t      ts_estado_deadline        = 0;
static TickType_t      ts_denegado_inicio    = 0;

static uint8_t buf_ultimo_uid[10] = {0};
static uint8_t len_ultimo_uid = 0;

static char buf_msg_activo[17]      = "Sin mensajes";
static bool flag_msg_activo_recibido = false;

static char buf_ui_linea1[17] = "Panel bloqueado";
static char buf_ui_linea2[17] = "Acerque tarjeta";

static rtc_fecha_hora_t rtc_hora_actual = {
    .seg = 0, .min = 50, .hora = 12,
    .dia_semana = 1, .dia = 1, .mes = 1, .anio = 2026,
};

/* UID autorizado principal del laboratorio */
static const uint8_t uid_admin[] = { 0xD9, 0x00, 0xE3, 0x56 };

static void configurar_leds(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_R) |
                        (1ULL << PIN_LED_G) |
                        (1ULL << PIN_LED_B),
        .mode           = GPIO_MODE_OUTPUT,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_LED_R,   0);
    gpio_set_level(PIN_LED_G, 0);
    gpio_set_level(PIN_LED_B,  0);
}

static void configurar_pin_reset(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_RESET),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static void configurar_buzzer_pwm(void)
{
    ledc_timer_config_t cfg_timer = {
        .speed_mode       = CFG_BUZZER_SPEED_MODE,
        .duty_resolution  = CFG_BUZZER_RESOLUTION,
        .timer_num        = CFG_BUZZER_TIMER,
        .freq_hz          = CFG_BUZZER_FREQ_SHORT,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&cfg_timer);

    ledc_channel_config_t cfg_ch = {
        .speed_mode = CFG_BUZZER_SPEED_MODE,
        .channel    = CFG_BUZZER_CHANNEL,
        .timer_sel  = CFG_BUZZER_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_BUZZER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&cfg_ch);
}

static void apagar_buzzer(void)
{
    ledc_set_duty(CFG_BUZZER_SPEED_MODE, CFG_BUZZER_CHANNEL, 0);
    ledc_update_duty(CFG_BUZZER_SPEED_MODE, CFG_BUZZER_CHANNEL);
}

static void emitir_pitido(uint32_t freq_hz, uint32_t dur_ms)
{
    ledc_set_freq(CFG_BUZZER_SPEED_MODE, CFG_BUZZER_TIMER, freq_hz);
    uint32_t duty_val = (1U << CFG_BUZZER_RESOLUTION) / 2U;
    ledc_set_duty(CFG_BUZZER_SPEED_MODE, CFG_BUZZER_CHANNEL, duty_val);
    ledc_update_duty(CFG_BUZZER_SPEED_MODE, CFG_BUZZER_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(dur_ms));
    apagar_buzzer();
}

static uint8_t bcd_to_bin(uint8_t v) { return (uint8_t)(((v >> 4) * 10U) + (v & 0x0F)); }
static uint8_t bin_to_bcd(uint8_t v) { return (uint8_t)(((v / 10U) << 4) | (v % 10U)); }

static bool rtc_i2c_write(uint8_t reg, const uint8_t *data, size_t len)
{
    if (!g_mutex_i2c) return false;
    if (xSemaphoreTake(g_mutex_i2c, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    i2c_cmd_handle_t h_cmd = i2c_cmd_link_create();
    i2c_master_start(h_cmd);
    i2c_master_write_byte(h_cmd, (ADDR_RTC_DS1307 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h_cmd, reg, true);
    if (len > 0) i2c_master_write(h_cmd, (uint8_t *)data, len, true);
    i2c_master_stop(h_cmd);
    esp_err_t resultado = i2c_master_cmd_begin(PORT_RTC_I2C, h_cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h_cmd);

    xSemaphoreGive(g_mutex_i2c);
    return resultado == ESP_OK;
}

static bool rtc_i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    if (!g_mutex_i2c) return false;
    if (xSemaphoreTake(g_mutex_i2c, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    i2c_cmd_handle_t h_cmd = i2c_cmd_link_create();
    i2c_master_start(h_cmd);
    i2c_master_write_byte(h_cmd, (ADDR_RTC_DS1307 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h_cmd, reg, true);
    i2c_master_start(h_cmd);
    i2c_master_write_byte(h_cmd, (ADDR_RTC_DS1307 << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(h_cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(h_cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(h_cmd);
    esp_err_t resultado = i2c_master_cmd_begin(PORT_RTC_I2C, h_cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h_cmd);

    xSemaphoreGive(g_mutex_i2c);
    return resultado == ESP_OK;
}

static bool rtc_leer_fecha_hora(rtc_fecha_hora_t *out)
{
    uint8_t raw[7] = {0};
    if (!rtc_i2c_read(0x00, raw, 7)) return false;
    out->seg      = bcd_to_bin(raw[0] & 0x7F);
    out->min      = bcd_to_bin(raw[1] & 0x7F);
    out->hora        = bcd_to_bin(raw[2] & 0x3F);
    out->dia_semana = bcd_to_bin(raw[3] & 0x07);
    out->dia         = bcd_to_bin(raw[4] & 0x3F);
    out->mes       = bcd_to_bin(raw[5] & 0x1F);
    out->anio        = (uint16_t)(2000U + bcd_to_bin(raw[6]));
    return true;
}

static bool rtc_escribir_fecha_hora(const rtc_fecha_hora_t *dt)
{
    uint8_t raw[7];
    raw[0] = bin_to_bcd(dt->seg      & 0x7F);
    raw[1] = bin_to_bcd(dt->min      & 0x7F);
    raw[2] = bin_to_bcd(dt->hora        & 0x3F);
    raw[3] = bin_to_bcd(dt->dia_semana & 0x07);
    raw[4] = bin_to_bcd(dt->dia         & 0x3F);
    raw[5] = bin_to_bcd(dt->mes       & 0x1F);
    raw[6] = bin_to_bcd((uint8_t)(dt->anio % 100U));
    return rtc_i2c_write(0x00, raw, 7);
}

static bool rtc_esta_activo(void)
{
    uint8_t byte_seg = 0;
    if (!rtc_i2c_read(0x00, &byte_seg, 1)) return false;
    return (byte_seg & 0x80) == 0;
}

static bool rtc_parsear_fecha_compilacion(rtc_fecha_hora_t *out)
{
    static const char *nombres_mes[] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };
    int dia, anio, hora, minuto, segundo;
    char texto_mes[4] = {0};

    if (sscanf(__DATE__, "%3s %d %d", texto_mes, &dia, &anio) != 3) return false;
    if (sscanf(__TIME__, "%d:%d:%d", &hora, &minuto, &segundo) != 3) return false;

    int mes = 1;
    for (int i = 0; i < 12; ++i) {
        if (strcmp(texto_mes, nombres_mes[i]) == 0) { mes = i + 1; break; }
    }
    out->dia         = (uint8_t)dia;
    out->mes       = (uint8_t)mes;
    out->anio        = (uint16_t)anio;
    out->hora        = (uint8_t)hora;
    out->min      = (uint8_t)minuto;
    out->seg      = (uint8_t)segundo;
    out->dia_semana = 1;
    return true;
}

static void rtc_imprimir_debug(const rtc_fecha_hora_t *dt)
{
    printf("[RTC] %04u-%02u-%02u %02u:%02u:%02u\n",
           dt->anio, dt->mes, dt->dia,
           dt->hora, dt->min, dt->seg);
}

static void rtc_inicializar_si_necesario(void)
{
    rtc_fecha_hora_t dt;
    if (rtc_esta_activo() && rtc_leer_fecha_hora(&dt)) {
        rtc_hora_actual = dt;
        rtc_imprimir_debug(&rtc_hora_actual);
        return;
    }
    if (rtc_parsear_fecha_compilacion(&dt)) {
        dt.hora = 12;
        dt.min = 50;
        dt.seg = 0;
    }
    if (rtc_escribir_fecha_hora(&dt)) {
        rtc_hora_actual = dt;
        rtc_imprimir_debug(&rtc_hora_actual);
        return;
    }
    rtc_imprimir_debug(&rtc_hora_actual);
}

static bool normalizar_payload_msg(const uint8_t *data, uint16_t len, char *out, size_t out_size)
{
    if (!data || !out || out_size < 2 || len == 0) return false;

    size_t w = 0;
    for (uint16_t i = 0; i < len && w < (out_size - 1); ++i) {
        uint8_t c = data[i];
        if (c == '\r' || c == '\n') break;
        if (isprint((int)c)) {
            out[w++] = (char)c;
        }
    }
    out[w] = '\0';
    return w > 0;
}

static void guardar_ultimo_mensaje(const char *msg)
{
    if (!msg || msg[0] == '\0') return;

    nvs_handle_t h_nvs;
    if (nvs_open(NS_NVS_APP, NVS_READWRITE, &h_nvs) != ESP_OK) return;

    if (nvs_set_str(h_nvs, KEY_NVS_LAST_MSG, msg) == ESP_OK) {
        (void)nvs_commit(h_nvs);
    }
    nvs_close(h_nvs);
}

static void restaurar_ultimo_mensaje_nvs(void)
{
    nvs_handle_t h_nvs;
    char buf_guardado[LEN_MSG_ACTIVO_MAX + 1] = {0};
    size_t len_guardado = sizeof(buf_guardado);

    if (nvs_open(NS_NVS_APP, NVS_READONLY, &h_nvs) != ESP_OK) return;

    if (nvs_get_str(h_nvs, KEY_NVS_LAST_MSG, buf_guardado, &len_guardado) == ESP_OK && buf_guardado[0] != '\0') {
        snprintf(buf_msg_activo, sizeof(buf_msg_activo), "%s", buf_guardado);
        flag_msg_activo_recibido = true;
        flag_ui_dirty = true;
    }

    nvs_close(h_nvs);
}

static bool intentar_actualizar_hora_rtc_ble(const uint8_t *data, uint16_t len)
{
    char buf_payload[32] = {0};
    int hora, minuto, segundo;

    if (!normalizar_payload_msg(data, len, buf_payload, sizeof(buf_payload))) return false;

    if (sscanf(buf_payload, "HORA=%d:%d:%d", &hora, &minuto, &segundo) != 3 &&
        sscanf(buf_payload, "TIME=%d:%d:%d", &hora, &minuto, &segundo) != 3) {
        return false;
    }

    if (hora < 0 || hora > 23 || minuto < 0 || minuto > 59 || segundo < 0 || segundo > 59) {
        return true;
    }

    rtc_fecha_hora_t nueva_hora;

    xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
    nueva_hora = rtc_hora_actual;
    xSemaphoreGive(g_mutex_estado);

    nueva_hora.hora = (uint8_t)hora;
    nueva_hora.min = (uint8_t)minuto;
    nueva_hora.seg = (uint8_t)segundo;

    if (rtc_escribir_fecha_hora(&nueva_hora)) {
        xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
        rtc_hora_actual = nueva_hora;
        flag_ui_dirty = true;
        xSemaphoreGive(g_mutex_estado);

        if (ble_can_send()) {
            ble_send_string("Hora actualizada\r\n");
        }
    }

    return true;
}

static void formatear_uid_preview(
    const uint8_t *uid, uint8_t length,
    char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    size_t used = 0;
    uint8_t n_preview = (length > 4) ? 4 : length;
    for (uint8_t i = 0; i < n_preview; ++i) {
        int w = snprintf(out + used, out_size - used, "%02X", uid[i]);
        if (w < 0 || (size_t)w >= out_size - used) break;
        used += (size_t)w;
    }
    if (length > 4 && used + 2 < out_size) snprintf(out + used, out_size - used, "..");
}

static bool comparar_uid(const rfid_uid_t *uid, const uint8_t *ref, uint8_t len)
{
    if (uid->length != len) return false;
    return memcmp(uid->uid, ref, len) == 0;
}

static void actualizar_pantalla_estado(void)
{
    char buf_hora[17];
    snprintf(buf_hora, sizeof(buf_hora), "%02u:%02u:%02u",
             rtc_hora_actual.hora,
             rtc_hora_actual.min,
             rtc_hora_actual.seg);

    switch (estado_sistema) {
    case ESTADO_BLOQUEADO:
        snprintf(buf_ui_linea1, sizeof(buf_ui_linea1), "Panel bloqueado");
        snprintf(buf_ui_linea2, sizeof(buf_ui_linea2), "Acerque tarjeta");
        break;
    case ESTADO_CONCEDIDO:
        snprintf(buf_ui_linea1, sizeof(buf_ui_linea1), "Acceso concedido");
        snprintf(buf_ui_linea2, sizeof(buf_ui_linea2), "%s", buf_hora);
        break;
    case ESTADO_DENEGADO:
        snprintf(buf_ui_linea1, sizeof(buf_ui_linea1), "Acceso denegado");
        snprintf(buf_ui_linea2, sizeof(buf_ui_linea2), "UID no reg.");
        break;
    case ESTADO_ACTIVO:
        snprintf(buf_ui_linea1, sizeof(buf_ui_linea1), "%s",
                 flag_msg_activo_recibido ? buf_msg_activo : "Sin mensajes");
        snprintf(buf_ui_linea2, sizeof(buf_ui_linea2), "%s", buf_hora);
        break;
    }
}

static void enviar_snapshot_ble(const char *prefijo)
{
    if (!ble_can_send()) return;

    char buf_uid_preview[12];
    char buf_msg[128];
    estado_sistema_t est;
    bool denegado;
    uint8_t uid_len;

    xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
    est      = estado_sistema;
    denegado     = flag_acceso_denegado;
    uid_len = len_ultimo_uid;
    if (uid_len > 0)
        formatear_uid_preview(buf_ultimo_uid, uid_len, buf_uid_preview, sizeof(buf_uid_preview));
    else
        snprintf(buf_uid_preview, sizeof(buf_uid_preview), "SIN TARJETA");
    xSemaphoreGive(g_mutex_estado);

    snprintf(buf_msg, sizeof(buf_msg),
             "%s | estado=%s | acceso=%s | uid=%s\r\n",
             prefijo,
             (est == ESTADO_ACTIVO) ? "ACTIVO" : "BLOQUEADO",
             denegado ? "DENEGADO" : "OK",
             buf_uid_preview);

    ble_send_string(buf_msg);
}

static void cb_ble_rx(const uint8_t *data, uint16_t len, void *ctx)
{
    (void)ctx;

    if (intentar_actualizar_hora_rtc_ble(data, len)) {
        return;
    }

    char buf_entrante[LEN_MSG_ACTIVO_MAX + 1] = {0};
    bool flag_persistir = false;

    if (!normalizar_payload_msg(data, len, buf_entrante, sizeof(buf_entrante))) {
        return;
    }

    xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
    if (estado_sistema == ESTADO_ACTIVO) {
        snprintf(buf_msg_activo, sizeof(buf_msg_activo), "%s", buf_entrante);
        flag_msg_activo_recibido  = true;
        flag_ui_dirty = true;
        flag_persistir = true;
    }
    xSemaphoreGive(g_mutex_estado);

    if (flag_persistir) {
        guardar_ultimo_mensaje(buf_entrante);
    }
}

static void cb_ble_tx_listo(bool listo, void *ctx)
{
    (void)ctx;
    if (listo) enviar_snapshot_ble("BLE READY");
}

static void procesar_acceso(const rfid_uid_t *uid)
{
    bool autorizado = comparar_uid(uid, uid_admin, sizeof(uid_admin));
    estado_sistema_t estado_prev;
    uint8_t copy_len = uid->length;

    if (copy_len > (uint8_t)sizeof(buf_ultimo_uid)) {
        copy_len = (uint8_t)sizeof(buf_ultimo_uid);
    }

    xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
    estado_prev = estado_sistema;
    len_ultimo_uid = copy_len;
    memcpy(buf_ultimo_uid, uid->uid, copy_len);

    if (estado_prev == ESTADO_ACTIVO) {
        if (autorizado) {
            estado_sistema            = ESTADO_BLOQUEADO;
            flag_acceso_denegado         = false;
            flag_ui_dirty = true;
        }
        xSemaphoreGive(g_mutex_estado);
        if (autorizado) emitir_pitido(CFG_BUZZER_FREQ_SHORT, 500);
        return;
    }

    if (estado_prev != ESTADO_BLOQUEADO) {
        xSemaphoreGive(g_mutex_estado);
        return;
    }

    if (autorizado) {
        estado_sistema    = ESTADO_CONCEDIDO;
        flag_acceso_denegado = false;

        ts_estado_deadline  = xTaskGetTickCount() + pdMS_TO_TICKS(1000);

        ts_bloqueo_lectura_rfid = xTaskGetTickCount() + pdMS_TO_TICKS(2000);

        flag_ui_dirty = true;

        xSemaphoreGive(g_mutex_estado);

        emitir_pitido(CFG_BUZZER_FREQ_SHORT, 500);
        return;
    }

    estado_sistema       = ESTADO_DENEGADO;
    flag_acceso_denegado    = true;
    ts_denegado_inicio = xTaskGetTickCount();
    ts_estado_deadline     = ts_denegado_inicio + pdMS_TO_TICKS(2000);
    flag_ui_dirty           = true;
    xSemaphoreGive(g_mutex_estado);
    emitir_pitido(CFG_BUZZER_FREQ_LONG, 2000);
}

static void reiniciar_estado_acceso(void)
{
    xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
    estado_sistema            = ESTADO_BLOQUEADO;
    flag_acceso_denegado         = false;
    ts_estado_deadline          = 0;
    len_ultimo_uid         = 0;
    memset(buf_ultimo_uid, 0, sizeof(buf_ultimo_uid));
    flag_ui_dirty = true;
    xSemaphoreGive(g_mutex_estado);
}

static void configurar_buzzer(void)
{
    configurar_buzzer_pwm();
    apagar_buzzer();
}

static void actualizar_leds(void)
{
    TickType_t ts_ahora = xTaskGetTickCount();

    if (estado_sistema == ESTADO_DENEGADO) {
        TickType_t elapsed = ts_ahora - ts_denegado_inicio;
        gpio_set_level(PIN_LED_R,
            ((elapsed / pdMS_TO_TICKS(250)) % 2U) == 0U ? 1 : 0);
    } else {
        gpio_set_level(PIN_LED_R,
            estado_sistema == ESTADO_BLOQUEADO ? 1 : 0);
    }

    gpio_set_level(PIN_LED_G, estado_sistema == ESTADO_CONCEDIDO ? 1 : 0);
    gpio_set_level(PIN_LED_B,  estado_sistema == ESTADO_ACTIVO    ? 1 : 0);
}

static void tarea_control_estado(void *arg)
{
    (void)arg;
    while (true) {
        TickType_t ts_ahora = xTaskGetTickCount();
        xSemaphoreTake(g_mutex_estado, portMAX_DELAY);

        if (estado_sistema == ESTADO_CONCEDIDO && ts_ahora >= ts_estado_deadline) {
            estado_sistema            = ESTADO_ACTIVO;
            flag_ui_dirty = true;
        }
        if (estado_sistema == ESTADO_DENEGADO && ts_ahora >= ts_estado_deadline) {
            estado_sistema    = ESTADO_BLOQUEADO;
            flag_acceso_denegado = false;
            flag_ui_dirty        = true;
        }

        xSemaphoreGive(g_mutex_estado);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void tarea_control_rtc(void *arg)
{
    (void)arg;
    while (true) {
        rtc_fecha_hora_t dt;
        if (rtc_leer_fecha_hora(&dt)) {
            xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
            rtc_hora_actual = dt;
            rtc_imprimir_debug(&rtc_hora_actual);
            flag_ui_dirty = true;
            xSemaphoreGive(g_mutex_estado);
        } else {
            rtc_inicializar_si_necesario();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void tarea_control_acceso(void *arg)
{
    (void)arg;
    rfid_uid_t uid;

    static uint8_t buf_uid_local[10] = {0};
    static uint8_t len_uid_local = 0;

    bool flag_tarjeta_presente = false;

    while (true) {

        if (rfid_get_uid(&uid)) {

            bool misma_tarjeta =
                flag_tarjeta_presente &&
                uid.length == len_uid_local &&
                memcmp(uid.uid, buf_uid_local, uid.length) == 0;

            if (!misma_tarjeta) {

            if (xTaskGetTickCount() < ts_bloqueo_lectura_rfid) {
                vTaskDelay(pdMS_TO_TICKS(DELAY_RFID_SCAN_MS));
                continue;
            }

                uint8_t copy_len = uid.length;
                if (copy_len > (uint8_t)sizeof(buf_uid_local)) {
                    copy_len = (uint8_t)sizeof(buf_uid_local);
                }

                memcpy(buf_uid_local, uid.uid, copy_len);
                len_uid_local = copy_len;

                flag_tarjeta_presente = true;

                procesar_acceso(&uid);
            }

        } else {

            flag_tarjeta_presente = false;
            len_uid_local = 0;
            memset(buf_uid_local, 0, sizeof(buf_uid_local));
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

static void tarea_control_leds(void *arg)
{
    (void)arg;
    while (true) {
        actualizar_leds();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void tarea_control_reset(void *arg)
{
    (void)arg;
    TickType_t ts_ultimo_trigger = 0;
    while (true) {
        if (gpio_get_level(PIN_RESET) == 0) {
            TickType_t ts_ahora = xTaskGetTickCount();
            if ((ts_ahora - ts_ultimo_trigger) > pdMS_TO_TICKS(250)) {
                ts_ultimo_trigger = ts_ahora;
                reiniciar_estado_acceso();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void tarea_control_display(void *arg)
{
    (void)arg;
    char buf_linea1[21], buf_linea2[21];

    while (true) {
        bool flag_renderizar = false;

        xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
        if (flag_ui_dirty) {
            actualizar_pantalla_estado();
            flag_ui_dirty     = false;
            flag_renderizar = true;
        }
        if (estado_sistema == ESTADO_ACTIVO || estado_sistema == ESTADO_CONCEDIDO) {
            snprintf(buf_ui_linea2, sizeof(buf_ui_linea2), "%02u:%02u:%02u",
                     rtc_hora_actual.hora,
                     rtc_hora_actual.min,
                     rtc_hora_actual.seg);
            flag_renderizar = true;
        }
        snprintf(buf_linea1, sizeof(buf_linea1), "%s", buf_ui_linea1);
        snprintf(buf_linea2, sizeof(buf_linea2), "%s", buf_ui_linea2);
        xSemaphoreGive(g_mutex_estado);

        if (flag_renderizar) {
            lcd_write_line(0, buf_linea1);
            lcd_write_line(1, buf_linea2);
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_LCD_REFRESH_MS));
    }
}

void app_main(void)
{

    esp_err_t ret_nvs = nvs_flash_init();
    if (ret_nvs == ESP_ERR_NVS_NO_FREE_PAGES || ret_nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret_nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret_nvs);

    g_mutex_i2c   = xSemaphoreCreateMutex();
    g_mutex_estado = xSemaphoreCreateMutex();

    if (!g_mutex_i2c || !g_mutex_estado) return;

    configurar_leds();
    configurar_pin_reset();
    configurar_buzzer();

    configurar_i2c_bus(PORT_RTC_I2C,    PIN_RTC_SDA,     PIN_RTC_SCL);
    configurar_i2c_bus(PORT_DISP_I2C, PIN_DISP_SDA, PIN_DISP_SCL);

    lcd_init();

    restaurar_ultimo_mensaje_nvs();
    rtc_inicializar_si_necesario();

    xSemaphoreTake(g_mutex_estado, portMAX_DELAY);
    actualizar_pantalla_estado();
    xSemaphoreGive(g_mutex_estado);
    lcd_write_line(0, buf_ui_linea1);
    lcd_write_line(1, buf_ui_linea2);

    ble_register_rx_callback(cb_ble_rx, NULL);
    ble_register_tx_ready_callback(cb_ble_tx_listo, NULL);
    bluetooth_init();

    rfid_init(PIN_RFID_MISO, PIN_RFID_MOSI, PIN_RFID_SCLK, PIN_RFID_CS);

    xTaskCreate(tarea_control_estado,   "tarea_estado",   2048, NULL, 4, NULL);
    xTaskCreate(tarea_control_acceso,  "tarea_acceso",  4096, NULL, 5, NULL);
    xTaskCreate(tarea_control_leds,    "tarea_leds",    2048, NULL, 3, NULL);
    xTaskCreate(tarea_control_reset,   "tarea_reset",   2048, NULL, 4, NULL);
    xTaskCreate(tarea_control_display, "tarea_lcd", 4096, NULL, 4, NULL);
    xTaskCreate(tarea_control_rtc,     "tarea_rtc",     4096, NULL, 4, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}