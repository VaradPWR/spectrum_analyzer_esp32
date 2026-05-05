/*
 * Spectrum Analyzer — FFT Master with SPI Output
 * ESP32 #1: ADC → FFT → SPI → ESP32 #2
 *
 * SPI Wiring (VSPI / SPI3_HOST):
 *   ESP32 #1 (Master)    ESP32 #2 (Slave)
 *   GPIO23 MOSI ───────► GPIO23
 *   GPIO19 MISO ◄─────── GPIO19
 *   GPIO18 CLK  ───────► GPIO18
 *   GPIO15 CS   ───────► GPIO15
 *   GND         ───────  GND  ← essential
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/spi_master.h"
#include "dsps_fft2r.h"
#include "dsps_wind.h"

// ==================== CONFIGURATION ====================
#define EXAMPLE_ADC_UNIT        ADC_UNIT_1
#define EXAMPLE_ADC_CONV_MODE   ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN       ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH   SOC_ADC_DIGI_MAX_BITWIDTH

#define FFT_SIZE                512
#define FFT_BINS                (FFT_SIZE / 2)
#define SAMPLE_RATE_HZ          80000
#define FREQ_BIN_WIDTH_HZ       (SAMPLE_RATE_HZ / FFT_SIZE)

#define ADC_READ_LEN            256
#define SAMPLES_PER_BUFFER      FFT_SIZE

#define PIN_NUM_MOSI            23
#define PIN_NUM_MISO            19
#define PIN_NUM_CLK             18
#define PIN_NUM_CS              15
#define SPI_CLOCK_HZ            2000000   

#define SEND_EVERY_N_FFT        4
#define SPI_PACKET_MAGIC        0xE5FF1234U
#define SPI_QUEUE_DEPTH         2

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[1] = {ADC_CHANNEL_6};  // GPIO34
#else
static adc_channel_t channel[1] = {ADC_CHANNEL_2};
#endif

static const char *TAG = "FFT_MASTER";

// ==================== SPI PACKET ====================
typedef struct {
    uint32_t magic;
    uint32_t fft_num;
    uint32_t adc_min;
    uint32_t adc_max;
    float    adc_mean;
    float    peak_freq;
    float    peak_mag;
    uint32_t sample_rate;
    uint16_t num_bins;
    uint16_t _pad;
    float    magnitudes[256];
} __attribute__((packed)) spi_fft_packet_t;

// ==================== DATA STRUCTURES ====================
typedef struct {
    float    magnitude[FFT_BINS];
    uint32_t sample_number;
    float    peak_freq;
    float    peak_magnitude;
    uint32_t adc_min;
    uint32_t adc_max;
    float    adc_mean;
} fft_result_t;

typedef struct {
    uint32_t samples_collected;
    uint32_t ffts_computed;
    uint32_t spi_sent;
    uint32_t spi_dropped;
    uint32_t buffer_overflows;
} stats_t;

// ==================== GLOBALS ====================
static TaskHandle_t            adc_task_handle   = NULL;
static TaskHandle_t            fft_task_handle   = NULL;
static adc_continuous_handle_t adc_handle        = NULL;
static spi_device_handle_t     spi_dev           = NULL;
static stats_t                 stats             = {0};

static float    *sample_buffer_1      = NULL;
static float    *sample_buffer_2      = NULL;
static uint32_t *raw_adc_buffer_1     = NULL;
static uint32_t *raw_adc_buffer_2     = NULL;
static uint8_t   current_write_buffer = 0;
static SemaphoreHandle_t buffer_mutex   = NULL;
static SemaphoreHandle_t data_ready_sem = NULL;

static float        *fft_window = NULL;
static float        *fft_data   = NULL;
static fft_result_t  fft_result;
static QueueHandle_t spi_queue  = NULL;

// ==================== MEMORY ====================
static bool allocate_buffers(void)
{
    sample_buffer_1  = (float *)    heap_caps_malloc(SAMPLES_PER_BUFFER * sizeof(float),    MALLOC_CAP_8BIT);
    sample_buffer_2  = (float *)    heap_caps_malloc(SAMPLES_PER_BUFFER * sizeof(float),    MALLOC_CAP_8BIT);
    raw_adc_buffer_1 = (uint32_t *) heap_caps_malloc(SAMPLES_PER_BUFFER * sizeof(uint32_t), MALLOC_CAP_8BIT);
    raw_adc_buffer_2 = (uint32_t *) heap_caps_malloc(SAMPLES_PER_BUFFER * sizeof(uint32_t), MALLOC_CAP_8BIT);
    fft_window       = (float *)    heap_caps_malloc(FFT_SIZE            * sizeof(float),    MALLOC_CAP_8BIT);
    fft_data         = (float *)    heap_caps_malloc(FFT_SIZE * 2        * sizeof(float),    MALLOC_CAP_8BIT);

    if (!sample_buffer_1 || !sample_buffer_2 || !raw_adc_buffer_1 ||
        !raw_adc_buffer_2 || !fft_window || !fft_data) {
        ESP_LOGE(TAG, "Buffer allocation failed! Free heap: %"PRIu32,
                 esp_get_free_heap_size());
        return false;
    }

    memset(sample_buffer_1, 0, SAMPLES_PER_BUFFER * sizeof(float));
    memset(sample_buffer_2, 0, SAMPLES_PER_BUFFER * sizeof(float));
    memset(fft_data,        0, FFT_SIZE * 2        * sizeof(float));
    ESP_LOGI(TAG, "Buffers allocated OK — free heap: %"PRIu32,
             esp_get_free_heap_size());
    return true;
}

// ==================== SPI MASTER ====================
static esp_err_t spi_master_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num     = PIN_NUM_MOSI,
        .miso_io_num     = PIN_NUM_MISO,
        .sclk_io_num     = PIN_NUM_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = sizeof(spi_fft_packet_t),
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode           = 0,
        .spics_io_num   = PIN_NUM_CS,
        .queue_size     = 1,
    };

    ESP_LOGI(TAG, "Initializing SPI bus (SPI3_HOST)...");
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI master ready — %d MHz on MOSI:%d CLK:%d CS:%d",
             SPI_CLOCK_HZ / 1000000, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
    return ESP_OK;
}

// ==================== ADC ====================
static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle,
                                       const adc_continuous_evt_data_t *edata,
                                       void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(adc_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

static esp_err_t adc_init(void)
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size    = ADC_READ_LEN,
    };

    esp_err_t ret = adc_continuous_new_handle(&adc_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC handle failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode      = EXAMPLE_ADC_CONV_MODE,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num       = 1;
    adc_pattern[0].atten     = EXAMPLE_ADC_ATTEN;
    adc_pattern[0].channel   = channel[0] & 0x7;
    adc_pattern[0].unit      = EXAMPLE_ADC_UNIT;
    adc_pattern[0].bit_width = EXAMPLE_ADC_BIT_WIDTH;
    dig_cfg.adc_pattern       = adc_pattern;

    ret = adc_continuous_config(adc_handle, &dig_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_conv_done_cb };
    ret = adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADC ready — %d Hz on GPIO34", SAMPLE_RATE_HZ);
    return ESP_OK;
}

// ==================== FFT ====================
static void fft_init(void)
{
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: %s", esp_err_to_name(ret));
        return;
    }
    dsps_wind_hann_f32(fft_window, FFT_SIZE);
    ESP_LOGI(TAG, "FFT ready — %d pts %.1f Hz/bin", FFT_SIZE, (float)FREQ_BIN_WIDTH_HZ);
}

static void compute_fft(float *samples, uint32_t *raw_samples, fft_result_t *result)
{
    uint64_t adc_sum = 0;
    uint32_t adc_min = 4095, adc_max = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        adc_sum += raw_samples[i];
        if (raw_samples[i] < adc_min) adc_min = raw_samples[i];
        if (raw_samples[i] > adc_max) adc_max = raw_samples[i];
    }
    float dc_mean    = (float)adc_sum / FFT_SIZE;
    result->adc_min  = adc_min;
    result->adc_max  = adc_max;
    result->adc_mean = dc_mean;

    for (int i = 0; i < FFT_SIZE; i++) {
        float dc_removed     = (raw_samples[i] - dc_mean) / 2048.0f;
        fft_data[2 * i]     = dc_removed * fft_window[i];
        fft_data[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(fft_data, FFT_SIZE);
    dsps_bit_rev2r_fc32(fft_data, FFT_SIZE);

    float    max_mag = -200.0f;
    uint32_t max_bin = 0;
    for (int i = 0; i < FFT_BINS; i++) {
        float re  = fft_data[2 * i];
        float im  = fft_data[2 * i + 1];
        float mag = 20.0f * log10f(sqrtf(re * re + im * im) / FFT_SIZE + 1e-10f);
        result->magnitude[i] = mag;
        if (i > 0 && mag > max_mag) { max_mag = mag; max_bin = i; }
    }
    result->peak_freq      = max_bin * FREQ_BIN_WIDTH_HZ;
    result->peak_magnitude = max_mag;
    stats.ffts_computed++;
}

// ==================== TASKS ====================
static void adc_sampling_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t ret = adc_continuous_start(adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC start failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "ADC sampling started");

    uint8_t  read_buffer[ADC_READ_LEN];
    uint32_t sample_count = 0;
    float    *cur_buf     = sample_buffer_1;
    uint32_t *cur_raw     = raw_adc_buffer_1;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (1) {
            uint32_t ret_num = 0;
            ret = adc_continuous_read(adc_handle, read_buffer, ADC_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
                for (int i = 0; i + (int)SOC_ADC_DIGI_RESULT_BYTES <= (int)ret_num;
                     i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&read_buffer[i];
                    if (p->type1.channel != (channel[0] & 0x7)) continue;

                    uint32_t raw      = p->type1.data;
                    cur_raw[sample_count] = raw;
                    cur_buf[sample_count] = ((float)raw - 2048.0f) / 2048.0f;
                    sample_count++;
                    stats.samples_collected++;

                    if (sample_count >= SAMPLES_PER_BUFFER) {
                        if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                            if (current_write_buffer == 0) {
                                cur_buf = sample_buffer_2;
                                cur_raw = raw_adc_buffer_2;
                            } else {
                                cur_buf = sample_buffer_1;
                                cur_raw = raw_adc_buffer_1;
                            }
                            current_write_buffer = 1 - current_write_buffer;
                            xSemaphoreGive(buffer_mutex);
                            xSemaphoreGive(data_ready_sem);
                            sample_count = 0;
                        } else {
                            stats.buffer_overflows++;
                            sample_count = 0;
                        }
                    }
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }
}

static void fft_processing_task(void *arg)
{
    static spi_fft_packet_t pkt;
    ESP_LOGI(TAG, "FFT task started");

    while (1) {
        if (xSemaphoreTake(data_ready_sem, portMAX_DELAY) != pdTRUE) continue;
        if (xSemaphoreTake(buffer_mutex,   pdMS_TO_TICKS(10)) != pdTRUE) continue;

        float    *proc_buf = (current_write_buffer == 0) ? sample_buffer_2 : sample_buffer_1;
        uint32_t *proc_raw = (current_write_buffer == 0) ? raw_adc_buffer_2 : raw_adc_buffer_1;
        xSemaphoreGive(buffer_mutex);

        compute_fft(proc_buf, proc_raw, &fft_result);
        fft_result.sample_number = stats.ffts_computed;

        if (stats.ffts_computed % SEND_EVERY_N_FFT == 0) {
            pkt.magic       = SPI_PACKET_MAGIC;
            pkt.fft_num     = fft_result.sample_number;
            pkt.adc_min     = fft_result.adc_min;
            pkt.adc_max     = fft_result.adc_max;
            pkt.adc_mean    = fft_result.adc_mean;
            pkt.peak_freq   = fft_result.peak_freq;
            pkt.peak_mag    = fft_result.peak_magnitude;
            pkt.sample_rate = SAMPLE_RATE_HZ;
            pkt.num_bins    = FFT_BINS;
            pkt._pad        = 0;
            memcpy(pkt.magnitudes, fft_result.magnitude, FFT_BINS * sizeof(float));

            ESP_LOGI(TAG, "Queuing pkt #%"PRIu32" | %.0f Hz @ %.1f dB",
                     pkt.fft_num, pkt.peak_freq, pkt.peak_mag);

            if (xQueueSend(spi_queue, &pkt, 0) != pdTRUE) {
                stats.spi_dropped++;
                ESP_LOGW(TAG, "Queue full — dropped #%"PRIu32, pkt.fft_num);
            }
        }
    }
}

static void spi_send_task(void *arg)
{
    static spi_fft_packet_t pkt;
    ESP_LOGI(TAG, "SPI send task started");

    while (1) {
        if (xQueueReceive(spi_queue, &pkt, portMAX_DELAY) != pdTRUE) continue;

        ESP_LOGI(TAG, "Transmitting pkt #%"PRIu32" over SPI...", pkt.fft_num);

        spi_transaction_t trans = {
            .length    = sizeof(spi_fft_packet_t) * 8,
            .tx_buffer = &pkt,
            .rx_buffer = NULL,
        };

        esp_err_t ret = spi_device_polling_transmit(spi_dev, &trans);
        if (ret == ESP_OK) {
            stats.spi_sent++;
            ESP_LOGI(TAG, "SPI sent #%"PRIu32" OK | total sent: %"PRIu32,
                     pkt.fft_num, stats.spi_sent);
        } else {
            ESP_LOGE(TAG, "SPI transmit FAILED: %s", esp_err_to_name(ret));
        }
    }
}

// ==================== MAIN ====================
void app_main(void)
{
    ESP_LOGI(TAG, "=== FFT Master boot — free heap: %"PRIu32, esp_get_free_heap_size());
    ESP_LOGI(TAG, "SR:%d Hz | FFT:%d pts | %.1f Hz/bin | every %d FFTs | CS:GPIO%d",
             SAMPLE_RATE_HZ, FFT_SIZE, (float)FREQ_BIN_WIDTH_HZ,
             SEND_EVERY_N_FFT, PIN_NUM_CS);

    buffer_mutex   = xSemaphoreCreateMutex();
    data_ready_sem = xSemaphoreCreateBinary();
    if (!buffer_mutex || !data_ready_sem) {
        ESP_LOGE(TAG, "Semaphore creation failed"); return;
    }

    spi_queue = xQueueCreate(SPI_QUEUE_DEPTH, sizeof(spi_fft_packet_t));
    if (!spi_queue) {
        ESP_LOGE(TAG, "SPI queue creation failed"); return;
    }
    ESP_LOGI(TAG, "Queue created — item size %d bytes x depth %d",
             sizeof(spi_fft_packet_t), SPI_QUEUE_DEPTH);

    if (!allocate_buffers())         return;
    if (spi_master_init() != ESP_OK) return;
    if (adc_init()        != ESP_OK) return;
    fft_init();

    xTaskCreatePinnedToCore(adc_sampling_task,   "adc",      8192, NULL, 5, &adc_task_handle, 0);
    xTaskCreatePinnedToCore(fft_processing_task, "fft",      6144, NULL, 4, &fft_task_handle, 1);
    xTaskCreatePinnedToCore(spi_send_task,       "spi_send", 4096, NULL, 3, NULL,             1);

    ESP_LOGI(TAG, "All tasks started");
}
