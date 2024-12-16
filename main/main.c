#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#define I2C_MASTER_SCL_IO 22          // GPIO para SCL
#define I2C_MASTER_SDA_IO 21          // GPIO para SDA
#define I2C_MASTER_NUM I2C_NUM_0      // Puerto I2C
#define I2C_MASTER_FREQ_HZ 100000     // Frecuencia I2C
#define AS7263_ADDR 0x49              // Dirección I2C del AS7263

// Registros del AS7263
#define I2C_AS72XX_SLAVE_STATUS_REG 0x00
#define I2C_AS72XX_SLAVE_WRITE_REG  0x01
#define I2C_AS72XX_SLAVE_READ_REG   0x02

//Registros de los canales del sensor
#define R_HIGH_REG 0x08
#define R_LOW_REG  0x09
#define S_HIGH_REG 0x0A
#define S_LOW_REG  0x0B
#define T_HIGH_REG 0x0C
#define T_LOW_REG  0x0D
#define U_HIGH_REG 0x0E
#define U_LOW_REG  0x0F
#define V_HIGH_REG 0x10
#define V_LOW_REG  0x11
#define W_HIGH_REG 0x12
#define W_LOW_REG  0x13
// Registro para la temperatura
#define VIRTUAL_REG_DEVICE_TEMP 0x06 

#define tag "SSD1306"


//################## WIFI y THINGSBOARD ########################

//URL thingsboard HTTP
#define THINGSBOARD_URL "http://demo.thingsboard.io/api/v1/"
#define TOKEN "NZ3agLpeFyLjdUkiqJQI"
static const char *TAG ="wifi_Aquanir";

// Función para enviar los datos del LDR a ThingsBoard mediante Http
void send_data_to_thingsboard_http(int r, int s, int t,int u,int v, int w, int temp){
    char json_data[200];

    snprintf(json_data, sizeof(json_data), "{\"r\": %d, \"s\": %d, \"t\": %d, \"u\": %d, \"v\": %d, \"w\": %d,\"temperature\": %d}",r, s, t, u, v, w,temp);

    esp_http_client_config_t config = {
        .url = THINGSBOARD_URL TOKEN "/telemetry",
        .method = HTTP_METHOD_POST,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_http_client_set_header(client, "Content-Type", "aplication/json");
    esp_http_client_set_post_field(client, json_data, strlen(json_data));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI("HTTP", "Data sent successfully: %s", json_data);
    } else {
        ESP_LOGE("HTTP", "Error sending data: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

// Función para manejar eventos de Wi-Fi
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Conexión Wi-Fi perdida, reconectando...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado! IP Address: %s", ip4addr_ntoa(&event->ip_info.ip));
    }
}

// Inicialización de Wi-Fi
void wifi_init_sta(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SBC",              // Nombre del Wi-Fi (SSID)
            .password = "SBCwifi$",      // Contraseña del Wi-Fi
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Conectando al SSID: %s...", wifi_config.sta.ssid);
}

//################## I2C y SENSOR AS7263 ########################
esp_err_t i2c_master_read_slave_reg(uint8_t reg_addr, uint8_t *data) {
    esp_err_t ret;
    // Comienza una operación de escritura para indicar el registro a leer
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS7263_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret; // Error durante la escritura
    }

    // Comienza una operación de lectura para obtener el valor del registro
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS7263_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t i2c_master_write_slave_reg(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS7263_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Escritura en un registro virtual
void write_virtual_register(uint8_t reg, uint8_t value) {
    uint8_t status;
    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (status & 0x02); // Espera hasta que TX_VALID sea 0

    // Escribe la dirección del registro
    uint8_t reg_with_write = reg | 0x80;
    i2c_master_write_slave_reg(I2C_AS72XX_SLAVE_WRITE_REG, reg_with_write);

    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (status & 0x02); // Espera hasta que TX_VALID sea 0

    // Escribe el valor
    i2c_master_write_slave_reg(I2C_AS72XX_SLAVE_WRITE_REG, value);
}

// Lectura de un registro virtual
uint8_t read_virtual_register(uint8_t reg) {
    uint8_t status, data;

    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (status & 0x02); // Espera hasta que TX_VALID sea 0

    // Escribe la dirección del registro
    i2c_master_write_slave_reg(I2C_AS72XX_SLAVE_WRITE_REG, reg);

    do {
        i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_STATUS_REG, &status);
    } while (!(status & 0x01)); // Espera hasta que RX_VALID sea 1

    // Lee el dato
    i2c_master_read_slave_reg(I2C_AS72XX_SLAVE_READ_REG, &data);

    return data;
}

//Escaneo para comprobar que todos los dispositivos han sido encontrados
void i2c_scanner() {
    printf("Escaneando el bus I2C...\n");
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Dispositivo encontrado en dirección: 0x%02X\n", addr);
        }
    }
    printf("Escaneo finalizado.\n");
}


// Función principal
void app_main() {
	
    	//Inicializar el wifi
	wifi_init_sta();
    	vTaskDelay(pdMS_TO_TICKS(2000));
	
	//Inicializar dispositivos I2C
 	SSD1306_t dev;
	i2c_master_init(&dev, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, CONFIG_RESET_GPIO);
	#if CONFIG_FLIP
	dev._flip = true;
	ESP_LOGW(tag, "Flip upside down");
	#endif
	#if CONFIG_SSD1306_128x64
	ESP_LOGI(tag, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
	#endif // CONFIG_SSD1306_128x64

	//Comprobar que todos los dispositivos se han inicializado
	i2c_scanner();
	
    while (1) {

	//Leer temperatura
        uint8_t temperature = read_virtual_register(VIRTUAL_REG_DEVICE_TEMP);
        printf("Temperatura del sensor: %d °C\n", temperature);
        
        // Leer valor del canal R
        uint8_t r_low = read_virtual_register(R_LOW_REG);
        uint8_t r_high = read_virtual_register(R_HIGH_REG);
        uint16_t r_value = (r_high << 8) | r_low;
	    
        // Leer valor del canal S
        uint8_t s_low = read_virtual_register(S_LOW_REG);
        uint8_t s_high = read_virtual_register(S_HIGH_REG);
        uint16_t s_value = (s_high << 8) | s_low
	    
         // Leer valor del canal T
        uint8_t t_low = read_virtual_register(T_LOW_REG);
        uint8_t t_high = read_virtual_register(T_HIGH_REG);
        uint16_t t_value = (t_high << 8) | t_low;
	    
        // Leer valor del canal U
        uint8_t u_low = read_virtual_register(U_LOW_REG);
        uint8_t u_high = read_virtual_register(U_HIGH_REG);
        uint16_t u_value = (u_high << 8) | u_low;
	    
        // Leer valor del canal V
        uint8_t v_low = read_virtual_register(V_LOW_REG);
        uint8_t v_high = read_virtual_register(V_HIGH_REG);
        uint16_t v_value = (v_high << 8) | v_low;
	    
        // Leer valor del canal W
        uint8_t w_low = read_virtual_register(W_LOW_REG);
        uint8_t w_high = read_virtual_register(W_HIGH_REG);
        uint16_t w_value = (w_high << 8) | w_low;
	    
        printf("R: %u, S: %u, T: %u, U: %u, V: %u, W: %u\n", r_value, s_value, t_value, u_value, v_value, w_value);
        
	char buffer[128];

	// Limpiar la pantalla antes de actualizar
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xFF);
		
	// Crear y mostrar cada línea de texto en el OLED
	snprintf(buffer, sizeof(buffer), "  Temp -> %u", temperature);
	ssd1306_display_text(&dev, 0, buffer, strlen(buffer), false);
		
	snprintf(buffer, sizeof(buffer), "  R -> %u", r_value);
	ssd1306_display_text(&dev, 1, buffer, strlen(buffer), false);
		
	snprintf(buffer, sizeof(buffer), "  S -> %u", s_value);
	ssd1306_display_text(&dev, 2, buffer, strlen(buffer), false);
		
	snprintf(buffer, sizeof(buffer), "  T -> %u", t_value);
	ssd1306_display_text(&dev, 3, buffer, strlen(buffer), false);
		
	snprintf(buffer, sizeof(buffer), "  U -> %u", u_value);
	ssd1306_display_text(&dev, 4, buffer, strlen(buffer), false);
		
	snprintf(buffer, sizeof(buffer), "  V -> %u", v_value);
	ssd1306_display_text(&dev, 5, buffer, strlen(buffer), false);
		
	snprintf(buffer, sizeof(buffer), "  W -> %u", w_value);
	ssd1306_display_text(&dev, 6, buffer, strlen(buffer), false);
		
	send_data_to_thingsboard_http(r_value, s_value, t_value, u_value, v_value, w_value,temperature);
	    
        vTaskDelay(pdMS_TO_TICKS(1000)); // Retraso de 1 segundo
	    
    }
}
