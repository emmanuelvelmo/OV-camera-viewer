#include "freertos/FreeRTOS.h" // Núcleo de FreeRTOS
#include "freertos/task.h" // Tareas y delays
#include "esp_camera.h" // Control de cámara OV2640
#include "driver/spi_master.h" // Controlador SPI maestro
#include "driver/gpio.h" // Control de GPIOs

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← OV2640: 3.3V, GND, G15 (SCL), G14 (SDA), G22 (VSYNC), G27 (HREF), G13 (PCLK), G4 (XCLK), G5 (D0), G18 (D1), G19 (D2), G21 (D3), G36 (D4), G39 (D5), G34 (D6), G35 (D7)
//                   ← TFT SPI: 5V, GND, G23 (SCLK), G12 (MOSI), G2 (DC), G25 (CS), G26 (RST), G32 (BLK)

// PINES DE LA PLACA
#define PIN_TFT_DC 2 // G2 comando/datos TFT
#define PIN_TFT_CS 25 // G25 selección chip TFT
#define PIN_TFT_RST 26 // G26 reset TFT
#define PIN_TFT_BLK 32 // G32 retroiluminación TFT

// RESOLUCIÓN DE CÁMARA
#define ANCHO_CAMARA 240 // Ancho imagen QVGA
#define ALTO_CAMARA 320 // Alto imagen QVGA

// CONFIGURACIÓN DE CÁMARA OV2640
static const camera_config_t configuracion_camara = 
{
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = 4,
    .pin_sscb_sda = 14,
    .pin_sscb_scl = 15,
    
    .pin_d7 = 35,
    .pin_d6 = 34,
    .pin_d5 = 39,
    .pin_d4 = 36,
    .pin_d3 = 21,
    .pin_d2 = 19,
    .pin_d1 = 18,
    .pin_d0 = 5,
    
    .pin_vsync = 22,
    .pin_href = 27,
    .pin_pclk = 13,
    
    .xclk_freq_hz = 20000000,
    
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size = FRAMESIZE_QVGA,
    
    .jpeg_quality = 12,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST
};

// VARIABLES GLOBALES
spi_device_handle_t dispositivo_spi_tft; // Manejador SPI TFT

// FUNCIONES MÍNIMAS
// Envía comando a TFT
void enviar_comando_tft(uint8_t comando)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 0);
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.length = 8;
    transaccion_spi.tx_buffer = &comando;
    transaccion_spi.flags = SPI_TRANS_USE_TXDATA;
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi);
}

// Envía dato a TFT
void enviar_dato_tft(uint8_t dato)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 1);
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.length = 8;
    transaccion_spi.tx_buffer = &dato;
    transaccion_spi.flags = SPI_TRANS_USE_TXDATA;
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi);
}

// Envía datos múltiples a TFT
void enviar_datos_tft(uint8_t* datos, uint32_t longitud)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 1);
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.length = longitud * 8;
    transaccion_spi.tx_buffer = datos;
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi);
}

// Inicializa pantalla TFT
void inicializar_pantalla_tft()
{
    // Configurar pines GPIO
    gpio_config_t configuracion_pines;
    configuracion_pines.pin_bit_mask = (1ULL << PIN_TFT_DC) | (1ULL << PIN_TFT_RST) | (1ULL << PIN_TFT_BLK);
    configuracion_pines.mode = GPIO_MODE_OUTPUT;
    configuracion_pines.pull_up_en = GPIO_PULLUP_DISABLE;
    configuracion_pines.pull_down_en = GPIO_PULLDOWN_DISABLE;
    configuracion_pines.intr_type = GPIO_INTR_DISABLE;
    
    gpio_config(&configuracion_pines);
    
    // Resetear pantalla
    gpio_set_level((gpio_num_t)PIN_TFT_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level((gpio_num_t)PIN_TFT_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Configurar SPI
    spi_bus_config_t configuracion_bus_spi;
    configuracion_bus_spi.mosi_io_num = 12; // G12 para MOSI
    configuracion_bus_spi.miso_io_num = -1;
    configuracion_bus_spi.sclk_io_num = 23;
    configuracion_bus_spi.quadwp_io_num = -1;
    configuracion_bus_spi.quadhd_io_num = -1;
    configuracion_bus_spi.max_transfer_sz = ANCHO_CAMARA * ALTO_CAMARA * 2;
    
    spi_bus_initialize(SPI2_HOST, &configuracion_bus_spi, SPI_DMA_CH_AUTO);
    
    spi_device_interface_config_t configuracion_dispositivo_spi;
    configuracion_dispositivo_spi.clock_speed_hz = 40000000;
    configuracion_dispositivo_spi.mode = 0;
    configuracion_dispositivo_spi.spics_io_num = PIN_TFT_CS;
    configuracion_dispositivo_spi.queue_size = 7;
    configuracion_dispositivo_spi.pre_cb = NULL;
    configuracion_dispositivo_spi.post_cb = NULL;
    
    spi_bus_add_device(SPI2_HOST, &configuracion_dispositivo_spi, &dispositivo_spi_tft);
    
    // Configurar pantalla TFT
    enviar_comando_tft(0x01); // Software reset
    vTaskDelay(150 / portTICK_PERIOD_MS);
    
    enviar_comando_tft(0x11); // Sleep out
    vTaskDelay(120 / portTICK_PERIOD_MS);
    
    enviar_comando_tft(0x3A); // Interface pixel format
    enviar_dato_tft(0x55); // 16 bits per pixel
    
    enviar_comando_tft(0x36); // Memory access control
    enviar_dato_tft(0x48);
    
    enviar_comando_tft(0x21); // Display inversion on
    
    enviar_comando_tft(0x13); // Normal display mode
    
    enviar_comando_tft(0x29); // Display on
    
    // Encender retroiluminación
    gpio_set_level((gpio_num_t)PIN_TFT_BLK, 1);
}

// Dibuja frame de cámara en pantalla
void dibujar_frame_pantalla(uint8_t* datos_frame)
{
    // Configurar ventana de dibujo
    enviar_comando_tft(0x2A); // Comando para columna
    enviar_dato_tft(0);
    enviar_dato_tft(0);
    enviar_dato_tft(0);
    enviar_dato_tft(ANCHO_CAMARA - 1);
    
    enviar_comando_tft(0x2B); // Comando para fila
    enviar_dato_tft(0);
    enviar_dato_tft(0);
    enviar_dato_tft(0);
    enviar_dato_tft(ALTO_CAMARA - 1);
    
    enviar_comando_tft(0x2C); // Comando para escribir memoria
    
    // Enviar datos de píxeles
    enviar_datos_tft(datos_frame, ANCHO_CAMARA * ALTO_CAMARA * 2);
}

// Bucle principal de visualización
void bucle_visualizacion_camara()
{
    while (1)
    {
        // Capturar frame
        camera_fb_t* frame_camara = esp_camera_fb_get();
        
        if (frame_camara)
        {
            // Dibujar frame en pantalla
            dibujar_frame_pantalla(frame_camara->buf);
            
            // Liberar frame
            esp_camera_fb_return(frame_camara);
        }
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// PUNTO DE PARTIDA
void app_main()
{
    // Inicializar cámara
    esp_camera_init(&configuracion_camara);
    
    // Inicializar pantalla
    inicializar_pantalla_tft();
    
    // Ejecutar bucle de visualización
    bucle_visualizacion_camara();
}
