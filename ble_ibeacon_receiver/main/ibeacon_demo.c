#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "math.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_ibeacon_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <stddef.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wpa2.h"
#include "esp_smartconfig.h"
#include "protocol_examples_common.h"

#include "lwip/sockets.h"
#include "lwip/api.h"
#include "lwip/udp.h"
#include <lwip/netdb.h>
#include "driver/timer.h"
#include "lwip/apps/sntp.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"
#include "KalmanFilter.h"

KalmanFunc_t filter1;
KalmanFunc_t filter2;
KalmanFunc_t filter3;
esp_mqtt_client_handle_t client_demo;
TaskHandle_t xHandle_ibeacon = NULL;
TaskHandle_t xHandle_mqtt = NULL;
//Filter

char* url_mqtt = "mqtt://192.168.1.3:1883";
char* name_client = "Phone";

//char* mac_test = "00:00:00:00:00:00"; //HUB
char* mac_test = "11:11:11:11:11:11"; //Phone

char* format_data_test(int rssi, char* mac2){
	char* payload = NULL;
	char* mac1 = "11:11:11:11:11:11";
	asprintf(&payload,"{\"e\":[{\"m\":\"%s\",\"r\":\"%d\"}],\"st\":\"%s\",\"t\":\"50\"}", mac1, rssi, mac2);
	return payload;
}

char* format_data(int rssi, char* mac1, char* mac2){
	char* payload = NULL;
	asprintf(&payload,"{\"e\":[{\"m\":\"%s\",\"r\":\"%d\"}],\"st\":\"%s\",\"t\":\"50\"}", mac1, rssi, mac2);
	return payload;
}

///////////////////////////////////// iBeacon /////////////////////////////////////////////////////

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

int count1 = 0;
int count_arr1[20];
int count2 = 0;
int count_arr2[20];
int count3 = 0;
int count_arr3[20];
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
        break;
    }
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE("Scan iBeacon", "Scan start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE("Scan iBeacon", "Adv start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            if (esp_ble_is_ibeacon_packet(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len)){
                esp_ble_ibeacon_t *ibeacon_data = (esp_ble_ibeacon_t*)(scan_result->scan_rst.ble_adv);

                if(ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.major) == 10000 && ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.minor) == 10000){
//                	ESP_LOGI("Scan iBeacon", "----------iBeacon Found----------");
//                    esp_log_buffer_hex("IBEACON_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN );
//                    esp_log_buffer_hex("IBEACON_DEMO: Proximity UUID:", ibeacon_data->ibeacon_vendor.proximity_uuid, ESP_UUID_LEN_128);
//                    uint16_t major = ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.major);
//                    uint16_t minor = ENDIAN_CHANGE_U16(ibeacon_data->ibeacon_vendor.minor);
//                    ESP_LOGI("Scan iBeacon", "Major: 0x%04x (%d)", major, major);
//                    ESP_LOGI("Scan iBeacon", "Minor: 0x%04x (%d)", minor, minor);
//                    ESP_LOGI("Scan iBeacon", "Measured power (RSSI at a 1m distance):%d dbm", ibeacon_data->ibeacon_vendor.measured_power);
//                    ESP_LOGI("Scan iBeacon", "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);

//					char* mac_ibeacon = hex2str(scan_result->scan_rst.bda, 6);
//					int Tx = -59;
//					float h = 1.5;
//					if(!strcmp(mac_ibeacon, "ac67b27d79da")){
//						int rssi = scan_result->scan_rst.rssi;
//						float dx = pow(10, (float)(Tx - rssi)/(float)(10*2));
//						float dx1 = sqrt(pow(dx, 2) - pow(h,2));
//
//						int rssi_filter = (int)filter1.updateEstimate_func(&filter1.KalmanVariable, scan_result->scan_rst.rssi);
//						float dx2 = pow(10, (float)(Tx - rssi_filter)/(float)(10*2));
//						float dx3 = sqrt(pow(dx2, 2) - pow(h,2));
//						printf("\n Data1: %d ==> %.03f ==> %.03f, %d ==> %.03f==> %.03f", rssi, dx, dx1, rssi_filter, dx2, dx3);
//
//						char* data = format_data_test(dx, mac_ibeacon);
//						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
////						printf("\n Publish Data 1: %s", data);
//					}else if(!strcmp(mac_ibeacon, "ac67b27d79a2")){ //ac67b27d7a16
//						int rssi_filter = scan_result->scan_rst.rssi;
//						float dx = pow(10, (Tx - rssi_filter)/(10*2));
//						float dx1 = sqrt(pow(dx, 2) - pow(h,2));
//						printf("\n Data1: %.03f, %.03f", dx, dx1);
//						char* data = format_data_test(dx, mac_ibeacon);
//						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
////						printf("\n Publish Data 2: %s", data);
//					}

                	//Estimate Position
                	char* mac_ibeacon = hex2str(scan_result->scan_rst.bda, 6);
                	if(!strcmp(mac_ibeacon, "ac67b27d79da")){
//                		count_arr1[count1] = scan_result->scan_rst.rssi;
//                		count1++;
//                		if(count1 >= 10){
//                			int rssi = 0;
//                			for(int i = 0; i<10; i++){
//                				rssi+= count_arr1[i];
//                			}
//                			count1 = 0;
//                			char* data = format_data_test((int)(rssi/10), mac_ibeacon);
//							esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
//                		}

//						int rssi_filter = (int)filter1.updateEstimate_func(&filter1.KalmanVariable, scan_result->scan_rst.rssi);
//						char* data = format_data_test(rssi_filter, mac_ibeacon);
//						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);

						char* data = format_data_test(scan_result->scan_rst.rssi, mac_ibeacon);
						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
                	}else if(!strcmp(mac_ibeacon, "ac67b27d79a2")){ //ac67b27d7a16
//                		count_arr2[count2] = scan_result->scan_rst.rssi;
//                		count2++;
//                		if(count2 >= 10){
//                			int rssi = 0;
//                			for(int i = 0; i<10; i++){
//                				rssi+= count_arr2[i];
//                			}
//                			count2 = 0;
//                			char* data = format_data_test((int)(rssi/10), mac_ibeacon);
//							esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
//                		}

//						int rssi_filter = (int)filter2.updateEstimate_func(&filter2.KalmanVariable, scan_result->scan_rst.rssi);
//						char* data = format_data_test(rssi_filter, mac_ibeacon);
//						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);

						char* data = format_data_test(scan_result->scan_rst.rssi, mac_ibeacon);
						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
                	}else if(!strcmp(mac_ibeacon, "ac67b27d7992")){
//                		count_arr3[count3] = scan_result->scan_rst.rssi;
//                		count3++;
//                		if(count3 >= 10){
//                			int rssi = 0;
//                			for(int i = 0; i<10; i++){
//                				rssi+= count_arr3[i];
//                			}
//                			count3 = 0;
//                			char* data = format_data_test((int)(rssi/10), mac_ibeacon);
//							esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
//                		}

//						int rssi_filter = (int)filter3.updateEstimate_func(&filter3.KalmanVariable, scan_result->scan_rst.rssi);
//						char* data = format_data_test(rssi_filter, mac_ibeacon);
//						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);

						char* data = format_data_test(scan_result->scan_rst.rssi, mac_ibeacon);
						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
                	}

                	//Compare
//                	  char* mac1 = "00:00:00:00:00:00";
//                	  char* data1 = format_data_test(scan_result->scan_rst.rssi, mac1);
//                	  esp_mqtt_client_publish(client_demo, "/beacons/office", data1, strlen(data1), 1, 0);
//
//                	  char* mac2 = "11:11:11:11:11:11";
//                	  int rssi_filter = (int)filter1.updateEstimate_func(&filter1.KalmanVariable, scan_result->scan_rst.rssi);
//                	  char* data2 = format_data_test(rssi_filter, mac2);
//                	  esp_mqtt_client_publish(client_demo, "/beacons/office", data2, strlen(data2), 1, 0);

//                	esp_log_buffer_hex("IBEACON_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN );
//					char* mac_test = "00:00:00:00:00:00"; //HUB
////					char* mac_test = "11:11:11:11:11:11"; //Phone
//					char* mac_ibeacon = hex2str(scan_result->scan_rst.bda, 6);
//					if(!strcmp(mac_ibeacon, "ac67b27d79da")){
//						int rssi_filter = scan_result->scan_rst.rssi;
//						char* data = format_data_test(rssi_filter, mac_test);
//						esp_mqtt_client_publish(client_demo, "/beacons/office", data, strlen(data), 1, 0);
//					}

                }
            }
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE("Scan iBeacon", "Scan stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI("Scan iBeacon", "Stop scan successfully");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE("Scan iBeacon", "Adv stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI("Scan iBeacon", "Stop adv successfully");
        }
        break;

    default:
        break;
    }
}

void ble_ibeacon_appRegister(void)
{
    esp_err_t status;
    ESP_LOGI("Scan iBeacon", "register callback");

    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE("Scan iBeacon", "gap register error: %s", esp_err_to_name(status));
        return;
    }

}

void ble_ibeacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_ibeacon_appRegister();
}

void ibeacon_scan_task(void *pvParameters){
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    ble_ibeacon_init();
    esp_ble_gap_set_scan_params(&ble_scan_params);
	while(1){
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

////////////////////////////// MQTT ///////////////////////////////////////////////////

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE("MQTT", "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD("MQTT", "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
//    esp_mqtt_client_handle_t client = event->client;
    client_demo = event->client;
//    char* topic = "/beacons/office";
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
    	if(xHandle_ibeacon == NULL){
    		xTaskCreate(ibeacon_scan_task, "ibeacon_scan_task", 4096, NULL, 3, &xHandle_ibeacon);
    	}
        ESP_LOGI("MQTT", "MQTT_EVENT_CONNECTED");
//        esp_mqtt_client_publish(client, topic, "abc", 0, 1, 0);
//      ESP_LOGI("MQTT", "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
//        ESP_LOGI("MQTT", "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
//        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
//        ESP_LOGI("MQTT", "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
//        ESP_LOGI("MQTT", "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
//        ESP_LOGI("MQTT", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("MQTT", "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI("MQTT", "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI("MQTT", "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = url_mqtt,
		.username = "station",
		.password = "bledemo",
		.client_id = name_client,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void mqtt_task(void *pvParameters){
    ESP_LOGI("MQTT", "[APP] Startup..");
    ESP_LOGI("MQTT", "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI("MQTT", "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());
    mqtt_app_start();

	while(1){
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void app_main(void) {
	ESP_ERROR_CHECK(nvs_flash_init());
	static uint8_t mac_address[6];
	if (esp_efuse_mac_get_default(mac_address) == ESP_OK){
	ESP_LOGI(__FUNCTION__, "Mac Address: %x-%x-%x-%x-%x-%x", mac_address[0],
			mac_address[1], mac_address[2], mac_address[3], mac_address[4],
			mac_address[5]);
	}
	//Filter 1
	filter1.SimpleKalmanFilter_func = SimpleKalmanFilter;
	filter1.SimpleKalmanFilter_func(&filter1.KalmanVariable, 6, 4, 0.05); //9 1 0.065
	filter1.updateEstimate_func = updateEstimate;

	//Filter 2
	filter2.SimpleKalmanFilter_func = SimpleKalmanFilter;
	filter2.SimpleKalmanFilter_func(&filter2.KalmanVariable, 6, 4, 0.05);
	filter2.updateEstimate_func = updateEstimate;

	//Filter 2
	filter3.SimpleKalmanFilter_func = SimpleKalmanFilter;
	filter3.SimpleKalmanFilter_func(&filter3.KalmanVariable, 6, 4, 0.05);
	filter3.updateEstimate_func = updateEstimate;

	xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 3, &xHandle_mqtt);
	while(1){
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
