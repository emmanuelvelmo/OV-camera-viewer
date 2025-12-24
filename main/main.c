#include <stdio.h> // Funciones de entrada/salida
#include "freertos/FreeRTOS.h" // Sistema operativo FreeRTOS
#include "freertos/task.h" // Tareas y delays
#include "esp_system.h" // Sistema ESP32
#include "esp_camera.h" // Control de cámara OV2640
#include "esp_lcd_panel_io.h" // Interface para pantallas LCD
#include "esp_lcd_panel_ops.h" // Operaciones de panel LCD
#include "driver/spi_master.h" // Controlador SPI maestro
#include "driver/gpio.h" // Control de GPIOs

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← OV2640: 5V, GND, G15(SCL), G14(SDA), G22(VSYNC), G27(HREF), G13(PCLK), G4(XCLK), G5(D0), G18(D1), G19(D2), G21(D3), G36(D4), G39(D5), G34(D6), G35(D7), PWDN(-1), RESET(-1)
//                   ← TFT SPI 240*320 V1.0: 5V, GND, G23(SCLK), G18(MOSI), G2(DC), G5(CS), G4(RST), G32(BLK)

// PINES DE LA PLACA
#define PIN_TFT_DC 2 // Pin de comando/datos para TFT
#define PIN_TFT_CS 5 // Pin de selección de chip para TFT
#define PIN_TFT_RST 4 // Pin de reset para TFT
#define PIN_TFT_BLK 32 // Pin de retroiluminación para TFT

// RESOLUCIÓN DE CÁMARA
#define ANCHO_CAMARA 240 // Ancho de imagen de cámara (QVGA)
#define ALTO_CAMARA 320 // Alto de imagen de cámara (QVGA)

// CONFIGURACIÓN DE CÁMARA OV2640
static const camera_config_t configuracion_camara = 
{
    .pin_pwdn = -1, // Sin pin de apagado
    .pin_reset = -1, // Sin pin de reset
    .pin_xclk = 4, // G4 para reloj
    .pin_sscb_sda = 14, // G14 para SDA
    .pin_sscb_scl = 15, // G15 para SCL
    
    .pin_d7 = 35, // G35 para dato 7
    .pin_d6 = 34, // G34 para dato 6
    .pin_d5 = 39, // G39 para dato 5
    .pin_d4 = 36, // G36 para dato 4
    .pin_d3 = 21, // G21 para dato 3
    .pin_d2 = 19, // G19 para dato 2
    .pin_d1 = 18, // G18 para dato 1
    .pin_d0 = 5, // G5 para dato 0
    
    .pin_vsync = 22, // G22 para sincronización vertical
    .pin_href = 27, // G27 para referencia horizontal
    .pin_pclk = 13, // G13 para reloj de píxel
    
    .xclk_freq_hz = 20000000, // Frecuencia de reloj de 20MHz
    
    .ledc_timer = LEDC_TIMER_0, // Timer para LEDC
    .ledc_channel = LEDC_CHANNEL_0, // Canal para LEDC
    
    .pixel_format = PIXFORMAT_RGB565, // Formato de píxel RGB565 (16 bits)
    .frame_size = FRAMESIZE_QVGA, // Tamaño de cuadro QVGA (240x320)
    
    .jpeg_quality = 12, // Calidad JPEG (no usado en RGB565)
    .fb_count = 2, // Número de buffers de frame
    .fb_location = CAMERA_FB_IN_PSRAM, // Ubicación de buffer en PSRAM
    .grab_mode = CAMERA_GRAB_LATEST // Modo de captura más reciente
};

// VARIABLES GLOBALES
spi_device_handle_t dispositivo_spi_tft; // Manejador de dispositivo SPI para TFT
camera_fb_t* frame_camara; // Puntero a frame de cámara

// FUNCIONES
// Envía comando a pantalla TFT
void enviar_comando_tft(uint8_t comando)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 0); // Modo comando
    
    spi_transaction_t transaccion_spi;
    
    memset(&transaccion_spi, 0, sizeof(transaccion_spi));
    
    transaccion_spi.length = 8; // Longitud en bits
    transaccion_spi.tx_buffer = &comando; // Buffer de transmisión
    transaccion_spi.flags = SPI_TRANS_USE_TXDATA; // Usar datos de transmisión
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi); // Transmisión
}

// Envía dato a pantalla TFT
void enviar_dato_tft(uint8_t dato)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 1); // Modo dato
    
    spi_transaction_t transaccion_spi;
    
    memset(&transaccion_spi, 0, sizeof(transaccion_spi));
    
    transaccion_spi.length = 8; // Longitud en bits
    transaccion_spi.tx_buffer = &dato; // Buffer de transmisión
    transaccion_spi.flags = SPI_TRANS_USE_TXDATA; // Usar datos de transmisión
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi); // Transmisión
}

// Envía múltiples datos a pantalla TFT
void enviar_datos_tft(uint8_t* datos, uint32_t longitud)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 1); // Modo dato
    
    spi_transaction_t transaccion_spi;
    
    memset(&transaccion_spi, 0, sizeof(transaccion_spi));
    
    transaccion_spi.length = longitud * 8; // Longitud en bits
    transaccion_spi.tx_buffer = datos; // Buffer de transmisión
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi); // Transmisión
}

// Inicializa pantalla TFT SPI
void inicializar_pantalla_tft()
{
    // Configurar pines GPIO
    gpio_config_t configuracion_pines;
    
    configuracion_pines.pin_bit_mask = (1ULL << PIN_TFT_DC) | (1ULL << PIN_TFT_RST) | (1ULL << PIN_TFT_BLK);
    configuracion_pines.mode = GPIO_MODE_OUTPUT; // Modo salida
    configuracion_pines.pull_up_en = GPIO_PULLUP_DISABLE; // Sin pull-up
    configuracion_pines.pull_down_en = GPIO_PULLDOWN_DISABLE; // Sin pull-down
    configuracion_pines.intr_type = GPIO_INTR_DISABLE; // Sin interrupciones
    
    gpio_config(&configuracion_pines); // Aplicar configuración
    
    // Resetear pantalla TFT
    gpio_set_level((gpio_num_t)PIN_TFT_RST, 0); // Reset bajo
    vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100ms
    gpio_set_level((gpio_num_t)PIN_TFT_RST, 1); // Reset alto
    vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100ms
    
    // Configurar bus SPI
    spi_bus_config_t configuracion_bus_spi;
    
    memset(&configuracion_bus_spi, 0, sizeof(configuracion_bus_spi));
    
    configuracion_bus_spi.mosi_io_num = 18; // G18 para MOSI
    configuracion_bus_spi.miso_io_num = -1; // Sin MISO
    configuracion_bus_spi.sclk_io_num = 23; // G23 para SCLK
    configuracion_bus_spi.quadwp_io_num = -1; // Sin WP
    configuracion_bus_spi.quadhd_io_num = -1; // Sin HD
    configuracion_bus_spi.max_transfer_sz = ANCHO_CAMARA * ALTO_CAMARA * 2; // Tamaño máximo de transferencia
    
    spi_bus_initialize(SPI2_HOST, &configuracion_bus_spi, SPI_DMA_CH_AUTO); // Inicializar bus SPI
    
    // Configurar dispositivo SPI
    spi_device_interface_config_t configuracion_dispositivo_spi;
    
    memset(&configuracion_dispositivo_spi, 0, sizeof(configuracion_dispositivo_spi));
    
    configuracion_dispositivo_spi.clock_speed_hz = 40000000; // Frecuencia de reloj 40MHz
    configuracion_dispositivo_spi.mode = 0; // Modo SPI 0
    configuracion_dispositivo_spi.spics_io_num = PIN_TFT_CS; // Pin CS
    configuracion_dispositivo_spi.queue_size = 7; // Tamaño de cola
    configuracion_dispositivo_spi.pre_cb = NULL; // Sin callback previo
    configuracion_dispositivo_spi.post_cb = NULL; // Sin callback posterior
    
    spi_bus_add_device(SPI2_HOST, &configuracion_dispositivo_spi, &dispositivo_spi_tft); // Agregar dispositivo
    
    // Configurar pantalla TFT (comandos básicos para ILI9341)
    enviar_comando_tft(0x01); // Software reset
    vTaskDelay(150 / portTICK_PERIOD_MS); // Espera 150ms
    
    enviar_comando_tft(0x11); // Sleep out
    vTaskDelay(120 / portTICK_PERIOD_MS); // Espera 120ms
    
    enviar_comando_tft(0x3A); // Interface pixel format
    enviar_dato_tft(0x55); // 16 bits per pixel
    
    enviar_comando_tft(0x36); // Memory access control
    enviar_dato_tft(0x48); // Configuración de orientación
    
    enviar_comando_tft(0x21); // Display inversion on
    
    enviar_comando_tft(0x13); // Normal display mode
    
    enviar_comando_tft(0x29); // Display on
    
    // Encender retroiluminación
    gpio_set_level((gpio_num_t)PIN_TFT_BLK, 1); // Retroiluminación encendida
}

// Dibuja frame de cámara en pantalla TFT
void dibujar_frame_pantalla(uint8_t* datos_frame, uint32_t ancho_frame, uint32_t alto_frame)
{
    // Configurar ventana de dibujo
    enviar_comando_tft(0x2A); // Comando para columna
    enviar_dato_tft(0); // Columna inicio alta
    enviar_dato_tft(0); // Columna inicio baja
    enviar_dato_tft(0); // Columna fin alta
    enviar_dato_tft(ancho_frame - 1); // Columna fin baja
    
    enviar_comando_tft(0x2B); // Comando para fila
    enviar_dato_tft(0); // Fila inicio alta
    enviar_dato_tft(0); // Fila inicio baja
    enviar_dato_tft(0); // Fila fin alta
    enviar_dato_tft(alto_frame - 1); // Fila fin baja
    
    enviar_comando_tft(0x2C); // Comando para escribir memoria
    
    // Enviar datos de píxeles (RGB565)
    enviar_datos_tft(datos_frame, ancho_frame * alto_frame * 2);
}

// Bucle principal de captura y visualización
void bucle_visualizacion_camara()
{
    while (1)
    {
        // Capturar frame de cámara
        frame_camara = esp_camera_fb_get(); // Obtener frame
        
        if (frame_camara)
        {
            // Verificar formato correcto
            if (frame_camara->format == PIXFORMAT_RGB565)
            {
                // Dibujar frame en pantalla
                dibujar_frame_pantalla(frame_camara->buf, ANCHO_CAMARA, ALTO_CAMARA);
            }
            
            // Devolver frame a buffer
            esp_camera_fb_return(frame_camara); // Liberar frame
        }
        
        // Pequeña pausa para estabilidad
        vTaskDelay(1 / portTICK_PERIOD_MS); // Espera 1ms
    }
}

// PUNTO DE PARTIDA
void app_main()
{
    // INICIALIZACIÓN DE MÓDULOS
    
    // Inicializar cámara OV2640
    esp_err_t error_camara = esp_camera_init(&configuracion_camara);
    
    if (error_camara != ESP_OK)
    {
        printf("Error inicializando cámara: 0x%x\n", error_camara);
        
        return; // Terminar si hay error
    }
    
    printf("Cámara OV2640 inicializada correctamente\n");
    
    // Inicializar pantalla TFT
    inicializar_pantalla_tft();
    
    printf("Pantalla TFT inicializada correctamente\n");
    
    // EJECUCIÓN PRINCIPAL
    
    // Ejecutar bucle de visualización
    bucle_visualizacion_camara();
}
