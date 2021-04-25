/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_netif.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "pca9685_regs.h"

#include "protocol_examples_common.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "tcpip_adapter.h"





#define TAG "main"



esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_READ, 11);
    i2c_master_read(cmd, data, len, 2); // nack
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_write(cmd, data, len, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	return ret;
}

void pca9685_write_byte(uint8_t addr, uint8_t reg)
{
	if(i2c_write(PCA9685_I2C_ADDRESS, addr, &reg, 1) != ESP_OK)
		ESP_LOGE(TAG, "pca9685 write byte failed");
}

void pca9685_read_byte(uint8_t addr, uint8_t *reg)
{
	if(i2c_read(PCA9685_I2C_ADDRESS, addr, reg, 1) != ESP_OK)
		ESP_LOGE(TAG, "pca9685 write byte failed");
}

esp_err_t pca9685_set_pwm(uint8_t num, uint16_t on, uint16_t off)
{
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PCA9685_I2C_ADDRESS << 1 | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, PCA9685_LED0_ON_L + 4 * num, 1);
    i2c_master_write_byte(cmd, on, 1);
    i2c_master_write_byte(cmd, on >> 8, 1);
    i2c_master_write_byte(cmd, off, 1);
    i2c_master_write_byte(cmd, off >> 8, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	return ret;
}

void pca9685_servo_init()
{

	pca9685_write_byte(PCA9685_MODE1, MODE1_RESTART);
	float freq = 50;
	// Range output modulation frequency is dependant on oscillator
	if (freq < 1)
	freq = 1;
	if (freq > 3500)
	freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)
#define PCA9685_OSCILLATOR_FREQ 27000000
	float prescaleval = ((PCA9685_OSCILLATOR_FREQ / (freq * 4096.0)) + 0.5) - 1;
	if (prescaleval < PCA9685_PRESCALE_MIN)
		prescaleval = PCA9685_PRESCALE_MIN;
	if (prescaleval > PCA9685_PRESCALE_MAX)
		prescaleval = PCA9685_PRESCALE_MAX;
	uint8_t prescale = (uint8_t)prescaleval;
	
	uint8_t oldmode;
	pca9685_read_byte(PCA9685_MODE1, &oldmode);
	uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
	pca9685_write_byte(PCA9685_MODE1, newmode);                             // go to sleep
	pca9685_write_byte(PCA9685_PRESCALE, prescale); // set the prescaler
	pca9685_write_byte(PCA9685_MODE1, oldmode);
	vTaskDelay(10 / portTICK_RATE_MS);
  // This sets the MODE1 register to turn on auto increment.
	pca9685_write_byte(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
	vTaskDelay(100 / portTICK_RATE_MS);
}

void tcp_server_task(void *arg)
{
    char rx_buffer[128];
    char addr_str[32];
	
	struct sockaddr_in destAddr;
	destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	destAddr.sin_family = AF_INET;
	destAddr.sin_port = htons(1234);
	inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

	int listen_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_IP);
	if (listen_sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		return;
	}
	ESP_LOGI(TAG, "Socket created");

	int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
	if (err != 0) {
		ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
		return;
	}
	ESP_LOGI(TAG, "Socket binded");
	
	err = listen(listen_sock, 3);
	if (err != 0) {
		ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
		return;
	}
	ESP_LOGI(TAG, "Socket listening");

	while(1)
	{
		struct sockaddr_in sourceAddr;
		uint addrLen = sizeof(sourceAddr);
		int sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
		if (sock < 0) {
			ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
			continue;
		}
		inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
		ESP_LOGI(TAG, "Socket accepted from %s", addr_str);

		int rlen = recv(sock, rx_buffer, sizeof(rx_buffer), 0);
		ESP_LOGI(TAG, "Socket received %d bytes", rlen);


		uint16_t onduty;

		onduty = 0;
		onduty |= rx_buffer[0];
		onduty <<= 8;
		onduty |= rx_buffer[1];
		if(120 <= onduty && onduty <= 400)
		{
			pca9685_set_pwm(0, 0, onduty);
			ESP_LOGI(TAG, "write %d at servo %d", onduty, 0);
		}
		else
		{
			ESP_LOGE(TAG, "servo 0 parameter error onduty is %d", onduty);
		}

		onduty = 0;
		onduty |= rx_buffer[2];
		onduty <<= 8;
		onduty |= rx_buffer[3];
		
		if(120 <= onduty && onduty <= 480)
		{
			pca9685_set_pwm(1, 0, onduty);
			ESP_LOGI(TAG, "write %d at servo %d", onduty, 1);
		}
		else
		{
			ESP_LOGE(TAG, "servo 1 parameter error onduty is %d", onduty);
		}

		closesocket(sock);
	
	}	
}



void app_main()
{
	ESP_LOGI(TAG, "app_main started");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = 5;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;
	i2c_driver_install(I2C_NUM_0, conf.mode);
	i2c_param_config(I2C_NUM_0, &conf);
	vTaskDelay(10 / portTICK_RATE_MS);
	
	pca9685_servo_init();
	
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
	example_set_connection_info("ipTIMEA2003NS", "hyeon__1123");
    ESP_ERROR_CHECK(example_connect());
	
    xTaskCreate(tcp_server_task, "tcp_server", 2048, NULL, 5, NULL);

	return;
	
	while(1)
	{

	
		/*
		for(int i=120;i<=400;i+=10)
		{
			pca9685_set_pwm(0, 0, i);
			ESP_LOGI(TAG, "%d", i);
			vTaskDelay(10 / portTICK_RATE_MS);
		}
		
		for(int i=400;i>=120;i-=10)
		{
			pca9685_set_pwm(0, 0, i);
			ESP_LOGI(TAG, "%d", i);
			vTaskDelay(10 / portTICK_RATE_MS);
		}

		*/

		vTaskDelay(100 / portTICK_RATE_MS);
	
	}
}
