#include <stdio.h>
#include <string.h>
#include <intr_types.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "portmacro.h"

static const char*TAG = "nvs_demo";

void app_main(void){
	esp_err_t err = nvs_flash_init();
	if(err== ESP_ERR_NVS_NEW_VERSION_FOUND){
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);
	
	nvs_handle_t nvs_handle;
	err=nvs_open("system",NVS_READWRITE,&nvs_handle);
	if(err!=ESP_OK){
		ESP_LOGE(TAG,"Failde to open NVS");
		return;
	}
	
	char ssid[33]={0};
	size_t ssid_len = sizeof(ssid);
	err = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len);
	if(err==ESP_ERR_NVS_NOT_FOUND){
		ESP_LOGE(TAG, "Setting default Wi-Fi Credentails");
		nvs_set_str(nvs_handle,"wifi_ssid","MyHomeWiFi");
		nvs_set_str(nvs_handle,"wifi_pass","SecurePass123");
		nvs_commit(nvs_handle);
	}
	nvs_get_str(nvs_handle, "wifi_ssid", ssid, &ssid_len);
	ESP_LOGI(TAG,"Wi-Fi SSID: %s",ssid);
	
	nvs_close(nvs_handle);
	
	while(1){
		vTaskDelay(5000/portTICK_PERIOD_MS);
	}
}
