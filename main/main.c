#include "freertos/FreeRTOS.h" // Núcleo de FreeRTOS
#include "freertos/task.h" // Tareas y delays
#include "esp_camera.h" // Control de cámara OV2640 (descargado automáticamente)
#include "driver/spi_master.h" // Controlador SPI maestro
#include "driver/gpio.h" // Control de GPIOs

// PLACA Y MÓDULOS
// ESPWROOM32 XX5R69 ← OV2640: 3.3V, GND, G15 (SCL), G14 (SDA), G22 (VSYNC), G27 (HREF), G13 (PCLK), G4 (XCLK), G5 (D0), G18 (D1), G19 (D2), G21 (D3), G36 (D4), G39 (D5), G34 (D6), G35 (D7)
//                   ← TFT SPI 240*320: 5V, GND, G23 (SCLK), G12 (MOSI), G2 (DC), G25 (CS), G26 (RST), G32 (BLK)

// PINES DE LA PLACA
#define PIN_TFT_DC 2 // G2 comando/datos TFT
#define PIN_TFT_CS 25 // G25 selección chip TFT
#define PIN_TFT_RST 26 // G26 reset TFT
#define PIN_TFT_BLK 32 // G32 retroiluminación TFT

// RESOLUCIÓN DE CÁMARA
#define ANCHO_CAMARA 240 // Ancho imagen QVGA (240 píxeles)
#define ALTO_CAMARA 320 // Alto imagen QVGA (320 píxeles)

// CONFIGURACIÓN DE CÁMARA OV2640
static const camera_config_t configuracion_camara = 
{
    .pin_pwdn = -1, // Sin pin de apagado
    .pin_reset = -1, // Sin pin de reset (reset por software)
    .pin_xclk = 4, // G4 para reloj de cámara
    .pin_sscb_sda = 14, // G14 para SDA (I2C datos)
    .pin_sscb_scl = 15, // G15 para SCL (I2C reloj)
    
    .pin_d7 = 35, // G35 para dato 7 (bus paralelo)
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
    
    .xclk_freq_hz = 20000000, // Frecuencia de reloj 20MHz
    
    .ledc_timer = LEDC_TIMER_0, // Timer 0 para LEDC
    .ledc_channel = LEDC_CHANNEL_0, // Canal 0 para LEDC
    
    .pixel_format = PIXFORMAT_RGB565, // Formato RGB565 (16 bits por píxel)
    .frame_size = FRAMESIZE_QVGA, // Resolución QVGA 240x320
    
    .jpeg_quality = 12, // Calidad JPEG (no usado en RGB565)
    .fb_count = 2, // Dos buffers de frame para mejor rendimiento
    .fb_location = CAMERA_FB_IN_PSRAM, // Buffers en PSRAM (requerido)
    .grab_mode = CAMERA_GRAB_LATEST // Capturar el frame más reciente
};

// VARIABLES GLOBALES
spi_device_handle_t dispositivo_spi_tft; // Manejador de dispositivo SPI para TFT

// FUNCIONES
// Envía comando a pantalla TFT (modo comando: DC bajo)
void enviar_comando_tft(uint8_t comando)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 0); // DC bajo = modo comando
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.length = 8; // Comando de 8 bits
    transaccion_spi.tx_buffer = &comando; // Buffer de transmisión
    transaccion_spi.flags = SPI_TRANS_USE_TXDATA; // Usar datos internos
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi); // Transmisión SPI
}

// Envía dato a pantalla TFT (modo dato: DC alto)
void enviar_dato_tft(uint8_t dato)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 1); // DC alto = modo dato
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.length = 8; // Dato de 8 bits
    transaccion_spi.tx_buffer = &dato; // Buffer de transmisión
    transaccion_spi.flags = SPI_TRANS_USE_TXDATA; // Usar datos internos
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi); // Transmisión SPI
}

// Envía múltiples datos a pantalla TFT (para imágenes completas)
void enviar_datos_tft(uint8_t* datos, uint32_t longitud)
{
    gpio_set_level((gpio_num_t)PIN_TFT_DC, 1); // DC alto = modo dato
    
    spi_transaction_t transaccion_spi;
    transaccion_spi.length = longitud * 8; // Longitud total en bits
    transaccion_spi.tx_buffer = datos; // Buffer de transmisión
    
    spi_device_polling_transmit(dispositivo_spi_tft, &transaccion_spi); // Transmisión SPI
}

// Inicializa pantalla TFT SPI (configura pines, SPI y comandos de inicialización)
void inicializar_pantalla_tft()
{
    // Configurar pines GPIO para control TFT
    gpio_config_t configuracion_pines;
    configuracion_pines.pin_bit_mask = (1ULL << PIN_TFT_DC) | (1ULL << PIN_TFT_RST) | (1ULL << PIN_TFT_BLK);
    configuracion_pines.mode = GPIO_MODE_OUTPUT; // Modo salida
    configuracion_pines.pull_up_en = GPIO_PULLUP_DISABLE; // Sin pull-up interno
    configuracion_pines.pull_down_en = GPIO_PULLDOWN_DISABLE; // Sin pull-down interno
    configuracion_pines.intr_type = GPIO_INTR_DISABLE; // Sin interrupciones
    
    gpio_config(&configuracion_pines); // Aplicar configuración
    
    // Resetear pantalla TFT (secuencia de reset)
    gpio_set_level((gpio_num_t)PIN_TFT_RST, 0); // Reset bajo
    vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100ms
    gpio_set_level((gpio_num_t)PIN_TFT_RST, 1); // Reset alto
    vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100ms
    
    // Configurar bus SPI para comunicación con TFT
    spi_bus_config_t configuracion_bus_spi;
    configuracion_bus_spi.mosi_io_num = 12; // G12 para MOSI (datos salida)
    configuracion_bus_spi.miso_io_num = -1; // Sin MISO (solo escritura)
    configuracion_bus_spi.sclk_io_num = 23; // G23 para SCLK (reloj)
    configuracion_bus_spi.quadwp_io_num = -1; // Sin modo quad (SPI estándar)
    configuracion_bus_spi.quadhd_io_num = -1; // Sin modo quad (SPI estándar)
    configuracion_bus_spi.max_transfer_sz = ANCHO_CAMARA * ALTO_CAMARA * 2; // Tamaño máximo: 240*320*2 bytes
    
    spi_bus_initialize(SPI2_HOST, &configuracion_bus_spi, SPI_DMA_CH_AUTO); // Inicializar bus SPI2
    
    // Configurar dispositivo SPI específico para TFT
    spi_device_interface_config_t configuracion_dispositivo_spi;
    configuracion_dispositivo_spi.clock_speed_hz = 40000000; // Frecuencia 40MHz
    configuracion_dispositivo_spi.mode = 0; // Modo SPI 0 (CPOL=0, CPHA=0)
    configuracion_dispositivo_spi.spics_io_num = PIN_TFT_CS; // Pin de chip select
    configuracion_dispositivo_spi.queue_size = 7; // Tamaño de cola de transacciones
    configuracion_dispositivo_spi.pre_cb = NULL; // Sin callback previo
    configuracion_dispositivo_spi.post_cb = NULL; // Sin callback posterior
    
    spi_bus_add_device(SPI2_HOST, &configuracion_dispositivo_spi, &dispositivo_spi_tft); // Agregar dispositivo
    
    // Secuencia de inicialización de pantalla TFT ILI9341
    enviar_comando_tft(0x01); // Comando software reset
    vTaskDelay(150 / portTICK_PERIOD_MS); // Espera 150ms para reset
    
    enviar_comando_tft(0x11); // Comando sleep out (salir de modo sleep)
    vTaskDelay(120 / portTICK_PERIOD_MS); // Espera 120ms
    
    enviar_comando_tft(0x3A); // Comando interface pixel format
    enviar_dato_tft(0x55); // 16 bits por píxel (RGB565)
    
    enviar_comando_tft(0x36); // Comando memory access control
    enviar_dato_tft(0x48); // Configuración de orientación
    
    enviar_comando_tft(0x21); // Comando display inversion on
    
    enviar_comando_tft(0x13); // Comando normal display mode
    
    enviar_comando_tft(0x29); // Comando display on
    
    // Encender retroiluminación
    gpio_set_level((gpio_num_t)PIN_TFT_BLK, 1); // Retroiluminación encendida
}

// Dibuja frame de cámara en pantalla TFT
void dibujar_frame_pantalla(uint8_t* datos_frame)
{
    // Configurar ventana de dibujo (área completa)
    enviar_comando_tft(0x2A); // Comando set column address
    enviar_dato_tft(0); // Columna inicio alta
    enviar_dato_tft(0); // Columna inicio baja
    enviar_dato_tft(0); // Columna fin alta
    enviar_dato_tft(ANCHO_CAMARA - 1); // Columna fin baja (239)
    
    enviar_comando_tft(0x2B); // Comando set page address
    enviar_dato_tft(0); // Fila inicio alta
    enviar_dato_tft(0); // Fila inicio baja
    enviar_dato_tft(0); // Fila fin alta
    enviar_dato_tft(ALTO_CAMARA - 1); // Fila fin baja (319)
    
    enviar_comando_tft(0x2C); // Comando write memory
    
    // Enviar datos de píxeles (240*320*2 = 153,600 bytes)
    enviar_datos_tft(datos_frame, ANCHO_CAMARA * ALTO_CAMARA * 2);
}

// Bucle principal de captura y visualización de cámara
void bucle_visualizacion_camara()
{
    while (1) // Bucle infinito
    {
        // Capturar frame de cámara (función proporcionada por esp32-camera)
        camera_fb_t* frame_camara = esp_camera_fb_get();
        
        if (frame_camara) // Verificar que se capturó correctamente
        {
            // Dibujar frame en pantalla TFT
            dibujar_frame_pantalla(frame_camara->buf);
            
            // Devolver frame al buffer para reutilización
            esp_camera_fb_return(frame_camara);
        }
        
        // Pequeña pausa para estabilidad (1ms)
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// PUNTO DE PARTIDA (función principal de ESP-IDF)
void app_main()
{
    // INICIALIZACIÓN DE CÁMARA
    // Nota: esp_camera.h descargado automáticamente por Component Manager
    esp_err_t resultado_camara = esp_camera_init(&configuracion_camara);
    
    // Verificar inicialización de cámara
    if (resultado_camara != ESP_OK)
    {
        // En producción, aquí podrías agregar manejo de error
        return; // Terminar si la cámara falla
    }
    
    // INICIALIZACIÓN DE PANTALLA
    inicializar_pantalla_tft();
    
    // EJECUCIÓN PRINCIPAL
    bucle_visualizacion_camara(); // Bucle infinito de visualización
}
