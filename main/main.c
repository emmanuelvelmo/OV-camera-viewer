#include <string.h> // Funciones de manipulación de memoria
#include <stdio.h>
#include "freertos/FreeRTOS.h" // Sistema operativo FreeRTOS
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_camera.h" // Control de cámara OV2640

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← OV2640: 3.3V, GND, GND (PWDN), G4 (SCL), G19 (SDA), G22 (VSYNC), G27 (HREF), G23 (PCLK), G5 (D0), G18 (D1), G0 (D2), G32 (D3), G36 (D4), G39 (D5), G34 (D6), G35 (D7)
//                   ← ILI9341: 5V, GND, G14 (SCLK), G13 (MOSI), G2 (DC), G15 (CS), G26 (RST), G21 (BLK)

// WIFI
static const char *wifi_ssid     = "Wi-Fi";
static const char *wifi_password = "748A0DE91A64";

/* ==============================
   IP FIJA
   ============================== */

static const char *ip_fija    = "192.168.0.100";
static const char *ip_gateway = "192.168.0.1";
static const char *ip_mascara = "255.255.255.0";

/* ==============================
   ESTADO
   ============================== */

static int wifi_conectado = 0;

/* ==============================
   HTML (ESCALADO PROPORCIONAL)
   ============================== */

static const char *pagina_html =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n\r\n"
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<style>"
    "html,body{margin:0;padding:0;width:100%;height:100%;background:black;overflow:hidden;}"
    "img{width:100%;height:100%;object-fit:contain;}"
    "</style>"
    "</head>"
    "<body>"
    "<img src='/stream'>"
    "</body>"
    "</html>";

/* ==============================
   CONFIGURACIÓN CÁMARA
   ============================== */

static camera_config_t camara_config = {
    .pin_pwdn  = -1,
    .pin_reset = -1,
    .pin_xclk  = -1,

    .pin_sscb_sda = 19,
    .pin_sscb_scl = 4,

    .pin_d7 = 35,
    .pin_d6 = 34,
    .pin_d5 = 39,
    .pin_d4 = 36,
    .pin_d3 = 32,
    .pin_d2 = 0,
    .pin_d1 = 18,
    .pin_d0 = 5,

    .pin_vsync = 22,
    .pin_href  = 27,
    .pin_pclk  = 23,

    .xclk_freq_hz = 12000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size   = FRAMESIZE_VGA, // 640x480

    .jpeg_quality = 12,
    .fb_count     = 1,
    .fb_location  = CAMERA_FB_IN_DRAM,
    .grab_mode    = CAMERA_GRAB_LATEST
};

/* ==============================
   WIFI EVENTS
   ============================== */

static void manejador_eventos_wifi(
    void *arg,
    esp_event_base_t base,
    int32_t id,
    void *datos
)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_conectado = 0;
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        wifi_conectado = 1;
    }
}

/* ==============================
   IP FIJA
   ============================== */

static void configurar_ip_fija(esp_netif_t *netif)
{
    esp_netif_dhcpc_stop(netif);

    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr      = inet_addr(ip_fija);
    ip_info.gw.addr      = inet_addr(ip_gateway);
    ip_info.netmask.addr = inet_addr(ip_mascara);

    esp_netif_set_ip_info(netif, &ip_info);
}

/* ==============================
   WIFI INIT
   ============================== */

static void inicializar_wifi(void)
{
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    configurar_ip_fija(netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &manejador_eventos_wifi, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &manejador_eventos_wifi, NULL);

    wifi_config_t wifi_cfg = { 0 };
    memcpy(wifi_cfg.sta.ssid, wifi_ssid, strlen(wifi_ssid));
    memcpy(wifi_cfg.sta.password, wifi_password, strlen(wifi_password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
}

static void esperar_wifi(void)
{
    while (!wifi_conectado)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

/* ==============================
   SERVIDOR MJPEG + HTML
   ============================== */

static void servidor_video(void *param)
{
    int servidor = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in dir_servidor = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(80)
    };

    bind(servidor, (struct sockaddr *)&dir_servidor, sizeof(dir_servidor));
    listen(servidor, 1);

    while (1)
    {
        struct sockaddr_in cliente_addr;
        socklen_t len = sizeof(cliente_addr);

        int cliente = accept(servidor, (struct sockaddr *)&cliente_addr, &len);
        if (cliente < 0) continue;

        char buffer[256] = { 0 };
        recv(cliente, buffer, sizeof(buffer), 0);

        /* STREAM */
        if (strstr(buffer, "GET /stream"))
        {
            send(cliente,
                 "HTTP/1.1 200 OK\r\n"
                 "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n",
                 79,
                 0);

            while (1)
            {
                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb) break;

                char header[64];

                send(cliente, "--frame\r\n", 9, 0);
                send(cliente, "Content-Type: image/jpeg\r\n", 28, 0);

                sprintf(header, "Content-Length: %d\r\n\r\n", fb->len);
                send(cliente, header, strlen(header), 0);

                send(cliente, fb->buf, fb->len, 0);
                send(cliente, "\r\n", 2, 0);

                esp_camera_fb_return(fb);

                vTaskDelay(33 / portTICK_PERIOD_MS);
            }
        }
        /* HTML */
        else
        {
            send(cliente, pagina_html, strlen(pagina_html), 0);
        }

        shutdown(cliente, 0);
        close(cliente);
    }
}

/* ==============================
   MAIN
   ============================== */

void app_main(void)
{
    nvs_flash_init();

    esp_camera_init(&camara_config);

    inicializar_wifi();
    esperar_wifi();

    xTaskCreate(
        servidor_video,
        "servidor_video",
        8192,
        NULL,
        5,
        NULL
    );

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
