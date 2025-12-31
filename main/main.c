#include <stdio.h> // Funciones de entrada/salida estándar
#include <string.h> // Funciones de manipulación de cadenas y memoria
#include "freertos/FreeRTOS.h" // Núcleo de sistema operativo FreeRTOS
#include "freertos/task.h" // Tareas y delays
#include "esp_event.h" // Sistema de eventos del ESP
#include "esp_wifi.h" // Configuración y control WiFi
#include "esp_netif.h" // Interfaz de red TCP/IP
#include "nvs_flash.h" // Sistema de almacenamiento no volátil
#include "lwip/sockets.h" // Funciones de socket (TCP/UDP)
#include "lwip/inet.h" // Conversiones de dirección IP
#include "esp_camera.h" // Control de cámara OV2640

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← OV2640: 3.3V, GND, GND (PWDN), G4 (SCL), G19 (SDA), G22 (VSYNC), G27 (HREF), G23 (PCLK), G5 (D0), G18 (D1), G0 (D2), G32 (D3), G36 (D4), G39 (D5), G34 (D6), G35 (D7)

// VARIABLES DE CONFIGURACIÓN WiFi
static const char *wifi_ssid = ""; // Nombre de red WiFi
static const char *wifi_password = ""; // Contraseña de red WiFi

// VARIABLES DE CONFIGURACIÓN IP ESTÁTICA
static const char *ip_fija = "192.168.0.100"; // IP estática del dispositivo
static const char *ip_gateway = "192.168.0.1"; // Puerta de enlace de red
static const char *ip_mascara = "255.255.255.0"; // Máscara de subred

// VARIABLES DE ESTADO
static int wifi_conectado = 0; // Bandera de conexión WiFi exitosa (0 = desconectado, 1 = conectado)

// PÁGINA HTML PARA SERVIDOR WEB (INTERFAZ CON IMAGEN ESCALADA)
static const char *pagina_html = // Contenido HTML completo para cliente HTTP
    "HTTP/1.1 200 OK\r\n" // Encabezado de respuesta HTTP
    "Content-Type: text/html\r\n\r\n" // Tipo de contenido
    "<!DOCTYPE html>" // Definición de documento HTML5
    "<html>" // Inicio del documento HTML
    "<head>" // Sección de metadatos
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>" // Configuración para dispositivos móviles
    "<style>" // Estilos CSS incrustados
    "html,body{margin:0;padding:0;width:100%;height:100%;background:black;overflow:hidden;}" // Estilos generales
    "img{width:100%;height:100%;object-fit:contain;}" // Estilo para imagen de video
    "</style>" // Fin de estilos
    "</head>" // Fin de sección de metadatos
    "<body>" // Cuerpo del documento
    "<img src='/stream'>" // Imagen que apunta al stream MJPEG
    "</body>" // Fin del cuerpo
    "</html>"; // Fin del documento

// CONFIGURACIÓN DE PINES Y PARÁMETROS DE CÁMARA OV2640
static camera_config_t camara_config = // Estructura de configuración de cámara
(
    .pin_pwdn  = -1, // Pin de apagado (no usado)
    .pin_reset = -1, // Pin de reinicio (no usado)
    .pin_xclk  = -1, // Pin de reloj externo (no usado)

    .pin_sscb_sda = 19, // Pin de datos I2C (SDA)
    .pin_sscb_scl = 4, // Pin de reloj I2C (SCL)

    .pin_d7 = 35, // Pin de datos bit 7
    .pin_d6 = 34, // Pin de datos bit 6
    .pin_d5 = 39, // Pin de datos bit 5
    .pin_d4 = 36, // Pin de datos bit 4
    .pin_d3 = 32, // Pin de datos bit 3
    .pin_d2 = 0, // Pin de datos bit 2
    .pin_d1 = 18, // Pin de datos bit 1
    .pin_d0 = 5, // Pin de datos bit 0

    .pin_vsync = 22, // Pin de sincronización vertical
    .pin_href  = 27, // Pin de referencia horizontal
    .pin_pclk  = 23, // Pin de reloj de píxeles

    .xclk_freq_hz = 12000000, // Frecuencia de reloj de cámara (12 MHz)
    .ledc_timer   = LEDC_TIMER_0, // Temporizador LEDC para señal XCLK
    .ledc_channel = LEDC_CHANNEL_0, // Canal LEDC para señal XCLK

    .pixel_format = PIXFORMAT_JPEG, // Formato de salida (JPEG comprimido)
    .frame_size   = FRAMESIZE_VGA, // Resolución de captura (640x480 píxeles)

    .jpeg_quality = 12, // Calidad de compresión JPEG (12 = más baja, más rápida)
    .fb_count     = 1, // Número de framebuffers (1 para modo simple)
    .fb_location  = CAMERA_FB_IN_DRAM, // Ubicación de framebuffers en DRAM
    .grab_mode    = CAMERA_GRAB_LATEST // Modo de captura (último frame disponible)
);

// FUNCIONES
// Gestiona eventos de conexión WiFi (conexión exitosa, desconexión, obtención de IP)
static void manejador_eventos_wifi(
    void *arg, // Argumento pasado al manejador (no usado)
    esp_event_base_t base, // Base del evento (WIFI_EVENT o IP_EVENT)
    int32_t id, // Identificador específico del evento
    void *datos // Datos adicionales del evento (no usado)
)
{
    // Evento: interfaz WiFi iniciada (modo estación)
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // Inicia conexión a punto de acceso
    }
    // Evento: desconexión de punto de acceso
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_conectado = 0; // Actualiza estado a desconectado
        
        esp_wifi_connect(); // Reintenta conexión
    }
    // Evento: dirección IP obtenida exitosamente
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        wifi_conectado = 1; // Actualiza estado a conectado
    }
}

// Configura dirección IP estática en interfaz de red
static void configurar_ip_fija(esp_netif_t *netif) // Puntero a interfaz de red
{
    esp_netif_dhcpc_stop(netif); // Detiene cliente DHCP para asignación manual

    // Estructura para almacenar información de IP
    esp_netif_ip_info_t ip_info;
    
    ip_info.ip.addr      = inet_addr(ip_fija); // Convierte dirección IP a formato binario
    ip_info.gw.addr      = inet_addr(ip_gateway); // Convierte puerta de enlace a formato binario
    ip_info.netmask.addr = inet_addr(ip_mascara); // Convierte máscara de subred a formato binario

    esp_netif_set_ip_info(netif, &ip_info); // Aplica configuración de IP a interfaz
}

// Inicializa subsistema WiFi en modo estación (cliente) con configuración personalizada
static void inicializar_wifi(void)
{
    esp_netif_init(); // Inicializa stack de red TCP/IP
    esp_event_loop_create_default(); // Crea loop de eventos por defecto

    // Crea interfaz de red para modo estación WiFi
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    
    configurar_ip_fija(netif); // Aplica configuración de IP estática

    // Configuración de inicialización WiFi por defecto
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    esp_wifi_init(&cfg); // Inicializa controlador WiFi

    // Registra manejadores para eventos WiFi
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &manejador_eventos_wifi, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &manejador_eventos_wifi, NULL);

    // Configuración de credenciales de red WiFi
    wifi_config_t wifi_cfg = { 0 }; // Inicializa estructura a ceros
    
    memcpy(wifi_cfg.sta.ssid, wifi_ssid, strlen(wifi_ssid)); // Copia SSID
    memcpy(wifi_cfg.sta.password, wifi_password, strlen(wifi_password)); // Copia contraseña

    esp_wifi_set_mode(WIFI_MODE_STA); // Configura modo estación (cliente)
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg); // Aplica configuración a interfaz estación
    esp_wifi_start(); // Inicia controlador WiFi
}

// Espera bloqueante hasta que se establezca conexión WiFi exitosa
static void esperar_wifi(void)
{
    // Ciclo que verifica estado de conexión periódicamente
    while (!wifi_conectado)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS); // Espera 500ms antes de verificar nuevamente
    }
}

// Servidor HTTP que maneja solicitudes de página HTML y stream MJPEG de video
static void servidor_video(void *param) // Parámetro de tarea (no usado)
{
    // Crea socket TCP para servidor web
    int servidor = socket(AF_INET, SOCK_STREAM, 0); // Familia IPv4, tipo stream (TCP)

    // Configura dirección del servidor (escucha en todos los interfaces, puerto 80 HTTP)
    struct sockaddr_in dir_servidor = (
        .sin_family = AF_INET, // Familia de direcciones IPv4
        .sin_addr.s_addr = INADDR_ANY, // Acepta conexiones de cualquier dirección
        .sin_port = htons(80) // Puerto 80 en orden de red (big-endian)
    );

    bind(servidor, (struct sockaddr *)&dir_servidor, sizeof(dir_servidor)); // Asocia socket a dirección
    listen(servidor, 1); // Pone socket en modo escucha (cola máxima: 1 cliente)

    // Bucle principal de servidor (atiende clientes secuencialmente)
    while (1)
    {
        // Estructura para dirección del cliente
        struct sockaddr_in cliente_addr;
        socklen_t len = sizeof(cliente_addr);

        // Acepta conexión entrante (bloqueante hasta que llegue cliente)
        int cliente = accept(servidor, (struct sockaddr *)&cliente_addr, &len);
        
        if (cliente < 0) continue; // Si error en aceptación, continua a siguiente ciclo

        // Buffer para almacenar solicitud HTTP del cliente
        char buffer[256] = { 0 }; // Inicializa buffer con ceros
        
        recv(cliente, buffer, sizeof(buffer), 0); // Recibe datos de solicitud

        // Verifica si solicitud es para stream MJPEG
        if (strstr(buffer, "GET /stream"))
        {
            // Envía encabezados HTTP para stream MJPEG
            send(cliente,
                 "HTTP/1.1 200 OK\r\n"
                 "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n",
                 79,
                 0);

            // Bucle de transmisión continua de frames JPEG
            while (1)
            {
                // Captura frame de cámara
                camera_fb_t *fb = esp_camera_fb_get();
                
                if (!fb) break; // Si no hay frame, sale del bucle

                // Buffer para encabezado HTTP personalizado
                char header[64];

                send(cliente, "--frame\r\n", 9, 0); // Marca inicio de frame MJPEG
                send(cliente, "Content-Type: image/jpeg\r\n", 28, 0); // Tipo de contenido

                // Formatea encabezado con longitud de frame
                sprintf(header, "Content-Length: %d\r\n\r\n", fb->len);
                send(cliente, header, strlen(header), 0); // Envía encabezado

                send(cliente, fb->buf, fb->len, 0); // Envía datos de imagen JPEG
                send(cliente, "\r\n", 2, 0); // Terminador de frame

                esp_camera_fb_return(fb); // Libera framebuffer para reutilización

                vTaskDelay(33 / portTICK_PERIOD_MS); // Aproximadamente 30 FPS (1000ms/30≈33ms)
            }
        }
        // Solicitud para página HTML principal
        else
        {
            send(cliente, pagina_html, strlen(pagina_html), 0); // Envía página HTML completa
        }

        shutdown(cliente, 0); // Cierra conexión de forma ordenada
        close(cliente); // Libera descriptor de socket
    }
}

// PUNTO DE PARTIDA (FUNCIÓN PRINCIPAL DEL PROGRAMA)
void app_main(void)
{
    nvs_flash_init(); // Inicializa almacenamiento no volátil (flash)

    esp_camera_init(&camara_config); // Inicializa cámara con configuración especificada

    inicializar_wifi(); // Configura y conecta a red WiFi
    esperar_wifi(); // Espera hasta que conexión WiFi esté establecida

    // Crea tarea para servidor de video (ejecuta función servidor_video)
    xTaskCreate(
        servidor_video, // Función a ejecutar en la tarea
        "servidor_video", // Nombre descriptivo de la tarea
        8192, // Tamaño de pila en bytes
        NULL, // Parámetros pasados a la tarea (ninguno)
        5, // Prioridad de tarea (media)
        NULL // Manejador de tarea (no usado)
    );

    // Bucle principal (mantiene programa en ejecución)
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Espera 1 segundo
    }
}
