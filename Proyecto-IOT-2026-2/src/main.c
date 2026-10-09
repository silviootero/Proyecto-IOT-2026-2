#include <stdio.h>
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "SmartGreen_Perception";

// -------------------------------------------------------------------
// DEFINICIÓN DE PINES Y PERIFÉRICOS
// -------------------------------------------------------------------
#define ADC_UV_CHANNEL      ADC_CHANNEL_5    // GPIO33 (Sensor UV GUVA-S12SD, ADC1_CH5)
#define ADC_SOIL_CHANNEL    ADC_CHANNEL_7    // GPIO35 (Sensor humedad de tierra Funduino, ADC1_CH7)
#define GPIO_SOIL_PWR       GPIO_NUM_26      // GPIO26 (Alimentación del sensor de tierra, anti-corrosión)
#define GPIO_DHT22          GPIO_NUM_27      // GPIO27 (Sensor DHT22)
#define GPIO_OUT_LIGHT      GPIO_NUM_32      // GPIO32 (Transistor 2N2222: Tira LED / Grow Light)
#define GPIO_OUT_PUMP       GPIO_NUM_25      // GPIO25 (Transistor 2N2222: Micro-bomba / Humidificador)
#define GPIO_SERVO_VENT     GPIO_NUM_23      // GPIO23 (Servomotor SG90 - Ventana)

// Configuración PWM LEDC para Servomotor SG90 (50Hz)
#define LEDC_SERVO_TIMER    LEDC_TIMER_0
#define LEDC_SERVO_MODE     LEDC_LOW_SPEED_MODE
#define LEDC_SERVO_CHANNEL  LEDC_CHANNEL_0
#define LEDC_SERVO_FREQ     50               // 50 Hz para servomotores
#define LEDC_SERVO_RES      LEDC_TIMER_13_BIT// Resolución de 13 bits (0 a 8191)

// Mínimo y máximo Duty Cycle para SG90 (0° a 90° - 180°)
// ~0.5ms (205 counts) a ~2.5ms (1024 counts) sobre un periodo de 20ms
#define SERVO_MIN_DUTY      205              // Posición Ventana Cerrada (0°)
#define SERVO_MAX_DUTY      614              // Posición Ventana Abierta (~90°)

// UMBRALES DE CONTROL LOCAL
#define UV_THRESHOLD_INDEX  2.0f             // Si UV < 2.0, encender luz de apoyo
#define HUMIDITY_THRESHOLD  60.0f            // Si Humedad < 60%, activar humidificador / abrir ventana

// Calibración del sensor de humedad de tierra (ajustar con mediciones reales)
#define SOIL_DRY_MV         2600.0f          // mV medidos en aire / tierra seca
#define SOIL_WET_MV         1200.0f          // mV medidos en tierra húmeda
#define SOIL_DRY_PCT        30.0f            // Si humedad de tierra < 30%, activar riego

// MODO PRUEBA: 1 = control manual de actuadores por teclado (monitor serie)
//             0 = operación normal con control automático
#define DEMO_MODE           0

// -------------------------------------------------------------------
// ESTRUCTURA DE DATOS PARA SENSORES
// -------------------------------------------------------------------
typedef struct {
    float uv_index;
    float uv_voltage_mv;
    float soil_moisture_mv;
    float soil_moisture_pct;
    float humidity;
    float temperature;
} sensor_data_t;

// Cola de FreeRTOS para comunicación entre tareas
static QueueHandle_t xSensorQueue = NULL;

// Handle del ADC1 en modo oneshot
static adc_oneshot_unit_handle_t adc1_handle = NULL;

// Spinlock para la lectura bit-bang del DHT22 (timing crítico)
static portMUX_TYPE dht_mux = portMUX_INITIALIZER_UNLOCKED;

// -------------------------------------------------------------------
// FUNCIONES AUXILIARES DE HARDWARE
// -------------------------------------------------------------------

// Inicialización del periférico LEDC para el Servomotor
static void init_servo_pwm(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_SERVO_MODE,
        .duty_resolution = LEDC_SERVO_RES,
        .timer_num       = LEDC_SERVO_TIMER,
        .freq_hz         = LEDC_SERVO_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .gpio_num   = GPIO_SERVO_VENT,
        .speed_mode = LEDC_SERVO_MODE,
        .channel    = LEDC_SERVO_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_SERVO_TIMER,
        .duty       = SERVO_MIN_DUTY,
        .hpoint     = 0
    };
    ledc_channel_config(&ch_conf);
}

// Establecer posición del servomotor (0 a 100 %)
static void set_servo_angle_percent(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint32_t duty = SERVO_MIN_DUTY + ((SERVO_MAX_DUTY - SERVO_MIN_DUTY) * percent) / 100;
    ledc_set_duty(LEDC_SERVO_MODE, LEDC_SERVO_CHANNEL, duty);
    ledc_update_duty(LEDC_SERVO_MODE, LEDC_SERVO_CHANNEL);
}

// Inicialización de GPIOs de salida (transistores 2N2222: LEDs y bomba)
static void init_actuator_outputs(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUT_LIGHT) | (1ULL << GPIO_OUT_PUMP),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Iniciar actuadores en estado desactivado (0)
    gpio_set_level(GPIO_OUT_LIGHT, 0);
    gpio_set_level(GPIO_OUT_PUMP, 0);
}

// Inicialización del pin de datos del DHT22 (entrada con pull-up interno)
static void init_dht22(void) {
    gpio_set_direction(GPIO_DHT22, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_DHT22, GPIO_PULLUP_ONLY);
}

// Inicialización del ADC1 para sensores analógicos (GUVA-S12SD y humedad de tierra)
static void init_analog_sensors(void) {
    // GPIO26 alimenta el sensor de tierra solo durante la lectura (evita corrosión)
    gpio_set_direction(GPIO_SOIL_PWR, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_SOIL_PWR, 0);

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&init_cfg, &adc1_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,    // Rango de lectura 0V - 3.3V (aprox.)
        .bitwidth = ADC_BITWIDTH_12     // Resolución de 12 bits (0-4095)
    };
    adc_oneshot_config_channel(adc1_handle, ADC_UV_CHANNEL, &chan_cfg);
    adc_oneshot_config_channel(adc1_handle, ADC_SOIL_CHANNEL, &chan_cfg);
}

// Lectura promediada de un canal ADC1 (10 muestras), retorna mV
static float adc_read_avg_mv(adc_channel_t channel) {
    uint32_t acc = 0;
    for (int i = 0; i < 10; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc1_handle, channel, &raw) == ESP_OK) {
            acc += raw;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    acc /= 10;
    // Conversión 12 bits (0-4095) a 3300mV
    return ((float)acc / 4095.0f) * 3300.0f;
}

// Espera a que el pin abandone el nivel dado; retorna el tiempo en us, o -1 si timeout
static int dht_wait_level(gpio_num_t pin, int level, int timeout_us) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) == level) {
        if (esp_timer_get_time() - start > timeout_us) return -1;
    }
    return (int)(esp_timer_get_time() - start);
}

// Lectura real del DHT22 (protocolo 1-wire propietario, timing crítico)
static esp_err_t read_dht22(float *humidity, float *temperature) {
    uint8_t data[5] = {0};

    // Señal de inicio: MCU mantiene la línea en bajo por >= 1 ms
    gpio_set_direction(GPIO_DHT22, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_DHT22, 0);
    esp_rom_delay_us(1100);
    gpio_set_level(GPIO_DHT22, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(GPIO_DHT22, GPIO_MODE_INPUT);

    // Sección crítica: deshabilita interrupciones durante el muestreo (~4-5 ms)
    portENTER_CRITICAL(&dht_mux);

    // Respuesta del sensor: ~80us LOW + ~80us HIGH
    if (dht_wait_level(GPIO_DHT22, 1, 100) < 0 ||
        dht_wait_level(GPIO_DHT22, 0, 100) < 0 ||
        dht_wait_level(GPIO_DHT22, 1, 100) < 0) {
        portEXIT_CRITICAL(&dht_mux);
        return ESP_ERR_TIMEOUT;
    }

    // 40 bits: cada bit = 50us LOW + pulso HIGH (~28us = 0, ~70us = 1)
    for (int i = 0; i < 40; i++) {
        if (dht_wait_level(GPIO_DHT22, 0, 80) < 0) {
            portEXIT_CRITICAL(&dht_mux);
            return ESP_ERR_TIMEOUT;
        }
        int high_us = dht_wait_level(GPIO_DHT22, 1, 100);
        if (high_us < 0) {
            portEXIT_CRITICAL(&dht_mux);
            return ESP_ERR_TIMEOUT;
        }
        data[i / 8] = (data[i / 8] << 1) | (high_us > 40);
    }

    portEXIT_CRITICAL(&dht_mux);

    // Verificación de checksum
    if (data[4] != (uint8_t)(data[0] + data[1] + data[2] + data[3])) {
        return ESP_ERR_INVALID_CRC;
    }

    *humidity = (((uint16_t)data[0] << 8) | data[1]) / 10.0f;
    int16_t raw_temp = (((uint16_t)data[2] & 0x7F) << 8) | data[3];
    *temperature = raw_temp / 10.0f;
    if (data[2] & 0x80) *temperature = -*temperature;

    return ESP_OK;
}

// -------------------------------------------------------------------
// TAREAS FREERTOS
// -------------------------------------------------------------------

// Tarea 1: Lectura periódica de sensores (GUVA-S12SD y DHT22)
void vTaskReadSensors(void *pvParameters) {
    sensor_data_t current_data;
    // Valores iniciales por si el primer intento de lectura del DHT22 falla
    static float last_humidity = 60.0f;
    static float last_temperature = 25.0f;

    while (1) {
        // 1. Lectura con promediado del Sensor UV GUVA-S12SD
        float voltage_mv = adc_read_avg_mv(ADC_UV_CHANNEL);

        // Conversión a Índice UV aproximado para GUVA-S12SD (Índice UV ≈ Voltaje_mV / 113mV)
        float uv_index = voltage_mv / 113.0f;

        current_data.uv_voltage_mv = voltage_mv;
        current_data.uv_index = uv_index;

        // 1b. Lectura del sensor de humedad de tierra (Funduino)
        // Se enciende su alimentación (GPIO26) solo durante la medición para
        // evitar la corrosión electrolítica de las puntas del sensor.
        gpio_set_level(GPIO_SOIL_PWR, 1);
        vTaskDelay(pdMS_TO_TICKS(100));              // Tiempo de estabilización
        float soil_mv = adc_read_avg_mv(ADC_SOIL_CHANNEL);
        gpio_set_level(GPIO_SOIL_PWR, 0);

        float soil_pct = ((SOIL_DRY_MV - soil_mv) / (SOIL_DRY_MV - SOIL_WET_MV)) * 100.0f;
        if (soil_pct < 0.0f) soil_pct = 0.0f;
        if (soil_pct > 100.0f) soil_pct = 100.0f;

        current_data.soil_moisture_mv = soil_mv;
        current_data.soil_moisture_pct = soil_pct;

        // 2. Lectura real del sensor DHT22 (reutiliza último valor válido si falla)
        if (read_dht22(&current_data.humidity, &current_data.temperature) == ESP_OK) {
            last_humidity = current_data.humidity;
            last_temperature = current_data.temperature;
        } else {
            current_data.humidity = last_humidity;
            current_data.temperature = last_temperature;
            ESP_LOGW(TAG, "Lectura DHT22 fallida (timeout/checksum). Usando valores anteriores.");
        }

        // 3. Enviar datos estructurados a la Cola de FreeRTOS
        xQueueSend(xSensorQueue, &current_data, portMAX_DELAY);

        ESP_LOGI(TAG, "Sensor UV -> Vout: %.1f mV | Índice UV: %.2f", voltage_mv, uv_index);
        ESP_LOGI(TAG, "Sensor Tierra -> Vout: %.1f mV | Humedad suelo: %.1f %%", soil_mv, soil_pct);
        ESP_LOGI(TAG, "Sensor DHT22 -> Humedad: %.1f %% | Temp: %.1f °C", current_data.humidity, current_data.temperature);

        // Periodo de Muestreo: 2 segundos
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Tarea 2: Evaluación de umbrales y control local de actuadores
void vTaskControlActuators(void *pvParameters) {
    sensor_data_t received_data;

    while (1) {
        // Bloqueo a la espera de nuevos datos en la cola
        if (xQueueReceive(xSensorQueue, &received_data, portMAX_DELAY) == pdTRUE) {
            
            // --- Control 1: Iluminación Artificial según Radiación UV ---
            if (received_data.uv_index < UV_THRESHOLD_INDEX) {
                gpio_set_level(GPIO_OUT_LIGHT, 1); // Encender Tira LED
                ESP_LOGW(TAG, "[ACTUADOR] UV bajo (%.2f). Tira LED ACTIVADA.", received_data.uv_index);
            } else {
                gpio_set_level(GPIO_OUT_LIGHT, 0); // Apagar Tira LED
                ESP_LOGI(TAG, "[ACTUADOR] Radiación UV adecuada. Tira LED APAGADA.");
            }

            // --- Control 2: Humidificación, Riego y Ventilación ---
            bool air_dry  = received_data.humidity < HUMIDITY_THRESHOLD;
            bool soil_dry = received_data.soil_moisture_pct < SOIL_DRY_PCT;

            // Bomba: riega si el aire está seco O la tierra está seca
            gpio_set_level(GPIO_OUT_PUMP, (air_dry || soil_dry) ? 1 : 0);
            // Ventana: solo responde a humedad del aire
            set_servo_angle_percent(air_dry ? 100 : 0);

            if (air_dry || soil_dry) {
                ESP_LOGW(TAG, "[ACTUADOR] Aire seco=%d (%.1f%%) | Tierra seca=%d (%.1f%%). Bomba ACTIVADA%s.",
                         air_dry, received_data.humidity, soil_dry, received_data.soil_moisture_pct,
                         air_dry ? " y Ventana ABIERTA" : "");
            } else {
                ESP_LOGI(TAG, "[ACTUADOR] Humedad óptima. Bomba APAGADA y Ventana CERRADA.");
            }
        }
    }
}

// -------------------------------------------------------------------
// MODO PRUEBA: Control manual de actuadores por teclado (monitor serie)
// -------------------------------------------------------------------
// Teclas (escribe la letra en el monitor serie y presiona Enter o nada):
//   L -> Alternar Tira LED      B -> Alternar Bomba
//   V -> Ventana: cicla 0% -> 50% -> 100% -> 0%
//   T -> Secuencia completa de prueba    H -> Ayuda
static void demo_print_help(void) {
    ESP_LOGI(TAG, "===== MODO PRUEBA =====");
    ESP_LOGI(TAG, "L: LED on/off | B: Bomba on/off | V: Ventana cicla | T: Secuencia completa | H: Ayuda");
}

static void demo_run_sequence(void) {
    ESP_LOGI(TAG, "[TEST] Iniciando secuencia de prueba de actuadores...");

    ESP_LOGI(TAG, "[TEST] 1/3 - Tira LED ON (2s)");
    gpio_set_level(GPIO_OUT_LIGHT, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUT_LIGHT, 0);
    ESP_LOGI(TAG, "[TEST] Tira LED OFF");

    ESP_LOGI(TAG, "[TEST] 2/3 - Bomba ON (2s)");
    gpio_set_level(GPIO_OUT_PUMP, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUT_PUMP, 0);
    ESP_LOGI(TAG, "[TEST] Bomba OFF");

    ESP_LOGI(TAG, "[TEST] 3/3 - Ventana 0%% -> 100%% -> 0%%");
    set_servo_angle_percent(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_servo_angle_percent(100);
    vTaskDelay(pdMS_TO_TICKS(1500));
    set_servo_angle_percent(0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "[TEST] Secuencia completada. Todos los actuadores verificados.");
}

void vTaskDemo(void *pvParameters) {
    bool light_on = false, pump_on = false;
    uint8_t vent_state = 0; // 0 = cerrada, 1 = 50%, 2 = abierta

    vTaskDelay(pdMS_TO_TICKS(500)); // Esperar a que el log de arranque termine
    demo_print_help();

    while (1) {
        int c = getchar();  // Bloqueante: la tarea duerme hasta recibir una tecla
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        if (c >= 'A' && c <= 'Z') c += 'a' - 'A';

        switch (c) {
            case 'l':
                light_on = !light_on;
                gpio_set_level(GPIO_OUT_LIGHT, light_on);
                ESP_LOGW(TAG, "[MANUAL] Tira LED -> %s", light_on ? "ON" : "OFF");
                break;
            case 'b':
                pump_on = !pump_on;
                gpio_set_level(GPIO_OUT_PUMP, pump_on);
                ESP_LOGW(TAG, "[MANUAL] Bomba -> %s", pump_on ? "ON" : "OFF");
                break;
            case 'v':
                vent_state = (vent_state + 1) % 3;
                set_servo_angle_percent(vent_state * 50);
                ESP_LOGW(TAG, "[MANUAL] Ventana -> %d%%", vent_state * 50);
                break;
            case 't':
                demo_run_sequence();
                light_on = false;
                pump_on = false;
                vent_state = 0;
                break;
            case 'h':
                demo_print_help();
                break;
            default:
                break; // Ignorar otras teclas
        }
    }
}

// -------------------------------------------------------------------
// PUNTO DE ENTRADA PRINCIPAL (app_main)
// -------------------------------------------------------------------
void app_main(void) {
    ESP_LOGI(TAG, "Iniciando Capa de Percepcion SmartGreen (ESP-IDF + FreeRTOS)...");

    // Inicializar Periféricos de Hardware
    init_actuator_outputs();
    init_servo_pwm();
    init_analog_sensors();
    init_dht22();

    // Crear la Cola de FreeRTOS (Capacidad para 5 estructuras de datos)
    xSensorQueue = xQueueCreate(5, sizeof(sensor_data_t));

    if (xSensorQueue != NULL) {
        // Creación de Tareas de FreeRTOS
        xTaskCreatePinnedToCore(vTaskReadSensors, "Task_ReadSensors", 3072, NULL, 2, NULL, 0);
#if DEMO_MODE
        // En modo prueba, los actuadores se controlan por teclado (sin control automático)
        xTaskCreatePinnedToCore(vTaskDemo, "Task_Demo", 3072, NULL, 2, NULL, 1);
        ESP_LOGI(TAG, "MODO PRUEBA activo: actuadores en control manual por teclado.");
#else
        xTaskCreatePinnedToCore(vTaskControlActuators, "Task_ControlActuators", 3072, NULL, 2, NULL, 1);
        ESP_LOGI(TAG, "Tareas FreeRTOS creadas y corriendo en núcleos independientes.");
#endif
    } else {
        ESP_LOGE(TAG, "Error crítico: No se pudo crear la cola de FreeRTOS.");
    }
}