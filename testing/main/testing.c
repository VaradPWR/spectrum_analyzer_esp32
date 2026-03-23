/*
 * Spectrum Analyzer — Web Slave
 * ESP32 #2: Receives FFT packets over SPI, hosts WiFi AP, serves spectrum webpage
 *
 * WiFi: SSID "ESP32-Spectrum", password "spectrum1"
 * Open browser → http://192.168.4.1
 *
 * NO INTERNET REQUIRED — chart drawn with pure Canvas 2D API, zero CDN dependencies
 *
 * CMakeLists.txt:
 *   idf_component_register(
 *       SRCS "web_slave.c"
 *       INCLUDE_DIRS "."
 *       REQUIRES esp_wifi esp_http_server nvs_flash driver
 *   )
 */

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

// ==================== CONFIGURATION ====================
#define WIFI_SSID        "ESP32-Spectrum"
#define WIFI_PASS        "spectrum1"
#define WIFI_MAX_CONN    4

#define PIN_NUM_MOSI     23
#define PIN_NUM_MISO     19
#define PIN_NUM_CLK      18
#define PIN_NUM_CS       15

#define SPI_PACKET_MAGIC 0xE5FF1234U

static const char *TAG = "WEB_SLAVE";

// ==================== EMBEDDED HTML ====================
// Pure Canvas 2D — no external libraries, works with no internet
static const char INDEX_HTML[] =
"<!DOCTYPE html>"
"<html lang='en'>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>ESP32 Spectrum Analyzer</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{background:#0d1117;color:#c9d1d9;font-family:'Courier New',monospace;"
     "font-size:13px;padding:16px;min-height:100vh}"
"header{display:flex;align-items:center;justify-content:space-between;"
        "margin-bottom:14px;border-bottom:1px solid #21262d;padding-bottom:10px}"
"header h1{font-size:1em;color:#58a6ff;letter-spacing:.1em;text-transform:uppercase}"
"#conn{font-size:.8em;padding:3px 10px;border-radius:20px;"
      "background:#21262d;color:#8b949e;transition:all .3s}"
"#conn.live{background:#0d3321;color:#3fb950}"
"#conn.err{background:#3d0c10;color:#f85149}"
"#meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));"
       "gap:8px;margin-bottom:14px}"
".card{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:10px 12px}"
".lbl{font-size:.7em;color:#8b949e;letter-spacing:.08em;text-transform:uppercase;margin-bottom:4px}"
".val{font-size:1.15em;font-weight:bold;color:#3fb950}"
".a .val{color:#58a6ff}"
".w .val{color:#e3b341}"
"#cw{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:14px}"
"#cw h2{font-size:.75em;color:#58a6ff;text-transform:uppercase;"
        "letter-spacing:.1em;margin-bottom:10px}"
"canvas{display:block;background:#0d1117;border-radius:4px}"
"footer{margin-top:10px;font-size:.75em;color:#484f58;text-align:right}"
"</style>"
"</head>"
"<body>"
"<header>"
"<h1>&#9889; ESP32 Spectrum Analyzer</h1>"
"<span id='conn'>Waiting...</span>"
"</header>"
"<div id='meta'>"
"<div class='card a'><div class='lbl'>Frame #</div><div class='val' id='mf'>&#8212;</div></div>"
"<div class='card a'><div class='lbl'>Peak Freq</div><div class='val' id='mpf'>&#8212;</div></div>"
"<div class='card a'><div class='lbl'>Peak Mag</div><div class='val' id='mpm'>&#8212;</div></div>"
"<div class='card w'><div class='lbl'>ADC Min/Max</div><div class='val' id='madc'>&#8212;</div></div>"
"<div class='card w'><div class='lbl'>ADC Mean</div><div class='val' id='mme'>&#8212;</div></div>"
"<div class='card'><div class='lbl'>Sample Rate</div><div class='val' id='msr'>&#8212;</div></div>"
"</div>"
"<div id='cw'>"
"<h2>Real-time Spectrum</h2>"
"<canvas id='c'></canvas>"
"</div>"
"<footer id='ft'>No data yet</footer>"
"<script>"
"(function(){"

// Canvas setup
"var canvas=document.getElementById('c');"
"var g=canvas.getContext('2d');"
"var PAD={t:20,r:20,b:40,l:55};"

"function resize(){"
  "canvas.width=canvas.parentElement.clientWidth-28;"
  "canvas.height=Math.max(260,Math.floor(canvas.width*0.35));"
"}"
"resize();"
"window.addEventListener('resize',resize);"

// Chart drawing function — pure canvas, no libs
"var lastMags=null,lastFreqs=null,peakIdx=0;"

"function draw(){"
  "if(!lastMags)return;"
  "var W=canvas.width,H=canvas.height;"
  "var pw=W-PAD.l-PAD.r,ph=H-PAD.t-PAD.b;"
  "var n=lastMags.length;"

  // Background
  "g.fillStyle='#0d1117';g.fillRect(0,0,W,H);"

  // Y range — dynamic with padding
  "var ymin=Infinity,ymax=-Infinity;"
  "for(var i=0;i<n;i++){if(lastMags[i]<ymin)ymin=lastMags[i];if(lastMags[i]>ymax)ymax=lastMags[i];}"
  "ymin=Math.floor((ymin-5)/10)*10;"
  "ymax=Math.ceil((ymax+5)/10)*10;"
  "var yr=ymax-ymin||1;"

  "function gy(v){return PAD.t+ph*(1-(v-ymin)/yr);}"
  "function gx(i){return PAD.l+pw*i/(n-1);}"

  // Grid lines + Y labels
  "g.strokeStyle='#21262d';g.lineWidth=1;g.fillStyle='#8b949e';g.font='10px Courier New';g.textAlign='right';"
  "var ystep=Math.ceil(yr/6/10)*10;"
  "for(var v=Math.ceil(ymin/ystep)*ystep;v<=ymax;v+=ystep){"
    "var yy=gy(v);"
    "g.beginPath();g.moveTo(PAD.l,yy);g.lineTo(W-PAD.r,yy);g.stroke();"
    "g.fillText(v.toFixed(0),PAD.l-4,yy+3);"
  "}"

  // X grid + labels
  "var xstep=Math.ceil(n/8);"
  "g.textAlign='center';"
  "for(var i=0;i<n;i+=xstep){"
    "var xx=gx(i);"
    "g.strokeStyle='#21262d';g.beginPath();g.moveTo(xx,PAD.t);g.lineTo(xx,H-PAD.b);g.stroke();"
    "var fhz=lastFreqs?lastFreqs[i]:0;"
    "g.fillStyle='#8b949e';g.fillText((fhz>=1000?(fhz/1000).toFixed(1)+'k':fhz.toFixed(0)),xx,H-PAD.b+14);"
  "}"

  // Axis labels
  "g.fillStyle='#8b949e';g.font='11px Courier New';"
  "g.textAlign='center';g.fillText('Frequency (Hz)',PAD.l+pw/2,H-2);"
  "g.save();g.translate(12,PAD.t+ph/2);g.rotate(-Math.PI/2);"
  "g.fillText('Magnitude (dBFS)',0,0);g.restore();"

  // Fill under spectrum
  "g.beginPath();g.moveTo(gx(0),gy(lastMags[0]));"
  "for(var i=1;i<n;i++)g.lineTo(gx(i),gy(lastMags[i]));"
  "g.lineTo(gx(n-1),H-PAD.b);g.lineTo(gx(0),H-PAD.b);g.closePath();"
  "g.fillStyle='rgba(88,166,255,0.08)';g.fill();"

  // Spectrum line
  "g.beginPath();g.moveTo(gx(0),gy(lastMags[0]));"
  "for(var i=1;i<n;i++)g.lineTo(gx(i),gy(lastMags[i]));"
  "g.strokeStyle='#58a6ff';g.lineWidth=1.5;g.stroke();"

  // Peak marker
  "var px=gx(peakIdx),py=gy(lastMags[peakIdx]);"
  "g.beginPath();g.arc(px,py,5,0,2*Math.PI);"
  "g.fillStyle='#f85149';g.fill();"

  // Peak label
  "var lbl=(lastFreqs?lastFreqs[peakIdx]:0);"
  "lbl=(lbl>=1000?(lbl/1000).toFixed(2)+'kHz':lbl.toFixed(0)+'Hz');"
  "g.fillStyle='#f85149';g.font='bold 11px Courier New';g.textAlign='center';"
  "g.fillText(lbl,px,py-10);"
"}"

// Poll /data
"var lastFrame=-1;"
"var connEl=document.getElementById('conn');"
"function setConn(t,c){connEl.textContent=t;connEl.className=c||'';}"

"async function poll(){"
  "try{"
    "var r=await fetch('/data',{cache:'no-store'});"
    "if(!r.ok)throw new Error('HTTP '+r.status);"
    "var d=await r.json();"
    "if(!d.ready||d.n===lastFrame)return;"
    "lastFrame=d.n;"
    "var mags=d.m,sr=d.sr,nb=mags.length,bw=sr/(nb*2);"

    // Build freq array
    "lastFreqs=Array.from({length:nb},function(_,i){return i*bw;});"
    "lastMags=mags;"

    // Find peak — use ESP32 reported peak_freq
    "peakIdx=Math.round(d.pf/bw);"
    "if(peakIdx<0)peakIdx=0;if(peakIdx>=nb)peakIdx=nb-1;"

    "draw();"

    // Update metadata cards
    "document.getElementById('mf').textContent='#'+d.n;"
    "document.getElementById('mpf').textContent=d.pf.toFixed(0)+' Hz';"
    "document.getElementById('mpm').textContent=d.pm.toFixed(1)+' dBFS';"
    "document.getElementById('madc').textContent=d.mn+' / '+d.mx;"
    "document.getElementById('mme').textContent=d.me.toFixed(0);"
    "document.getElementById('msr').textContent=(sr/1000).toFixed(0)+' kHz';"
    "setConn('LIVE','live');"
    "document.getElementById('ft').textContent="
      "'Updated '+new Date().toLocaleTimeString()+'  |  '+"
      "nb+' bins  |  '+bw.toFixed(1)+' Hz/bin';"
  "}catch(e){"
    "setConn('ERROR','err');"
    "document.getElementById('ft').textContent='Error: '+e.message;"
  "}"
"}"

"window.addEventListener('resize',draw);"
"setInterval(poll,150);"
"poll();"
"})();"
"</script>"
"</body>"
"</html>";

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

#define PACKET_SIZE_BYTES sizeof(spi_fft_packet_t)

// ==================== SHARED STATE ====================
static spi_fft_packet_t  latest_packet;
static bool              packet_ready = false;
static SemaphoreHandle_t data_mutex   = NULL;

#define JSON_BUF_SIZE 4096
static char json_buf[JSON_BUF_SIZE];

// ==================== SPI SLAVE ====================
static WORD_ALIGNED_ATTR uint8_t spi_rx_buf[PACKET_SIZE_BYTES];

static void spi_slave_task(void *arg)
{
    ESP_LOGI(TAG, "SPI slave task started on core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "Pins — MOSI:%d MISO:%d CLK:%d CS:%d",
             PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS);

    spi_bus_config_t buscfg = {
        .mosi_io_num   = PIN_NUM_MOSI,
        .miso_io_num   = PIN_NUM_MISO,
        .sclk_io_num   = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode         = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size   = 1,
        .flags        = 0,
    };

    ESP_LOGI(TAG, "Calling spi_slave_initialize (SPI3_HOST)...");
    esp_err_t ret = spi_slave_initialize(SPI3_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    ESP_LOGI(TAG, "spi_slave_initialize returned: %s (0x%x)",
             esp_err_to_name(ret), ret);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI slave init FAILED — task exiting");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "SPI slave ready — waiting for packets (%d bytes each)",
             PACKET_SIZE_BYTES);

    spi_slave_transaction_t trans;
    uint32_t pkt_count = 0;

    while (1) {
        memset(&trans, 0, sizeof(trans));
        trans.length    = PACKET_SIZE_BYTES * 8;
        trans.rx_buffer = spi_rx_buf;
        trans.tx_buffer = NULL;

        ret = spi_slave_transmit(SPI3_HOST, &trans, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "spi_slave_transmit error: %s", esp_err_to_name(ret));
            continue;
        }

        uint32_t rx_bytes = trans.trans_len / 8;
        if (rx_bytes != PACKET_SIZE_BYTES) {
            ESP_LOGW(TAG, "Size mismatch: got %"PRIu32" expected %d",
                     rx_bytes, PACKET_SIZE_BYTES);
            continue;
        }

        spi_fft_packet_t *pkt = (spi_fft_packet_t *)spi_rx_buf;
        if (pkt->magic != SPI_PACKET_MAGIC) {
            ESP_LOGW(TAG, "Bad magic: 0x%08"PRIx32, pkt->magic);
            continue;
        }

        pkt_count++;
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&latest_packet, spi_rx_buf, PACKET_SIZE_BYTES);
            packet_ready = true;
            xSemaphoreGive(data_mutex);
        }

        ESP_LOGI(TAG, "Pkt #%"PRIu32" OK | %.0f Hz @ %.1f dB | total: %"PRIu32,
                 pkt->fft_num, pkt->peak_freq, pkt->peak_mag, pkt_count);
    }
}

// ==================== JSON BUILDER ====================
static int build_json(char *buf, size_t buf_size)
{
    if (!packet_ready)
        return snprintf(buf, buf_size, "{\"ready\":false}");

    int n = snprintf(buf, buf_size,
        "{\"ready\":true"
        ",\"n\":%"PRIu32
        ",\"pf\":%.0f"
        ",\"pm\":%.1f"
        ",\"mn\":%"PRIu32
        ",\"mx\":%"PRIu32
        ",\"me\":%.0f"
        ",\"sr\":%"PRIu32
        ",\"m\":[",
        latest_packet.fft_num,
        latest_packet.peak_freq,
        latest_packet.peak_mag,
        latest_packet.adc_min,
        latest_packet.adc_max,
        latest_packet.adc_mean,
        latest_packet.sample_rate);

    if (n < 0 || n >= (int)buf_size) return -1;

    uint16_t bins = latest_packet.num_bins;
    if (bins > 256) bins = 256;

    for (int i = 0; i < bins; i++) {
        int rem = (int)buf_size - n;
        if (rem < 16) return -1;
        n += snprintf(buf + n, rem, "%.1f%s",
                      latest_packet.magnitudes[i],
                      (i < bins - 1) ? "," : "");
    }

    int rem = (int)buf_size - n;
    if (rem < 3) return -1;
    n += snprintf(buf + n, rem, "]}");
    return n;
}

// ==================== HTTP HANDLERS ====================
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t data_handler(httpd_req_t *req)
{
    int json_len = -1;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        json_len = build_json(json_buf, JSON_BUF_SIZE);
        xSemaphoreGive(data_mutex);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (json_len <= 0)
        return httpd_resp_send(req, "{\"ready\":false}", 15);
    return httpd_resp_send(req, json_buf, json_len);
}

static httpd_handle_t start_http_server(void)
{
    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return NULL;
    }

    httpd_uri_t root = { .uri="/",     .method=HTTP_GET, .handler=root_handler };
    httpd_uri_t data = { .uri="/data", .method=HTTP_GET, .handler=data_handler };
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &data);

    ESP_LOGI(TAG, "HTTP server ready — http://192.168.4.1");
    return server;
}

// ==================== WIFI AP ====================
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "Client connected — MAC %02x:%02x:%02x:%02x:%02x:%02x AID %d",
                 e->mac[0],e->mac[1],e->mac[2],e->mac[3],e->mac[4],e->mac[5],e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "Client disconnected — MAC %02x:%02x:%02x:%02x:%02x:%02x",
                 e->mac[0],e->mac[1],e->mac[2],e->mac[3],e->mac[4],e->mac[5]);
    }
}

static esp_err_t wifi_ap_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid           = WIFI_SSID,
            .ssid_len       = strlen(WIFI_SSID),
            .password       = WIFI_PASS,
            .max_connection = WIFI_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP: \"%s\" | IP: 192.168.4.1", WIFI_SSID);
    return ESP_OK;
}

// ==================== MAIN ====================
void app_main(void)
{
    ESP_LOGI(TAG, "=== Spectrum Web Slave ===");
    ESP_LOGI(TAG, "Boot — free heap: %"PRIu32, esp_get_free_heap_size());
    ESP_LOGI(TAG, "Packet size: %d bytes", PACKET_SIZE_BYTES);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    data_mutex = xSemaphoreCreateMutex();
    if (!data_mutex) {
        ESP_LOGE(TAG, "Mutex creation failed");
        return;
    }

    xTaskCreatePinnedToCore(spi_slave_task, "spi_slave", 8192, NULL, 6, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(wifi_ap_init());
    start_http_server();

    ESP_LOGI(TAG, "Ready — connect to \"%s\" then open http://192.168.4.1", WIFI_SSID);
}
