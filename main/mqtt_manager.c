#include "mqtt_manager.h"
#include "config.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "mqtt_manager";

static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool s_is_connected = false;
static mqtt_led_callback_t led_callback = NULL;

// MQTT 이벤트 핸들러
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                                int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "MQTT 이벤트: base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT 브로커 연결됨");
        s_is_connected = true;
        
        // LED 제어 토픽 구독
        msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC_LED, 0);
        ESP_LOGI(TAG, "토픽 구독: %s (msg_id=%d)", MQTT_TOPIC_LED, msg_id);
        
        // 연결 상태 발행
        msg_id = esp_mqtt_client_publish(client, MQTT_TOPIC_STATUS, "connected", 0, 1, 0);
        ESP_LOGI(TAG, "상태 발행: %s (msg_id=%d)", MQTT_TOPIC_STATUS, msg_id);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT 브로커 연결 끊어짐");
        s_is_connected = false;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "토픽 구독 완료 (msg_id=%d)", event->msg_id);
        break;
        
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "토픽 구독 해제 (msg_id=%d)", event->msg_id);
        break;
        
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "메시지 발행 완료 (msg_id=%d)", event->msg_id);
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT 데이터 수신");
        ESP_LOGI(TAG, "토픽: %.*s", event->topic_len, event->topic);
        ESP_LOGI(TAG, "데이터: %.*s", event->data_len, event->data);
        
        // LED 제어 메시지 처리
        if (strncmp(event->topic, MQTT_TOPIC_LED, event->topic_len) == 0) {
            bool led_on = false;
            if (strncmp(event->data, "on", event->data_len) == 0) {
                led_on = true;
            } else if (strncmp(event->data, "off", event->data_len) == 0) {
                led_on = false;
            }
            
            // 콜백 호출
            if (led_callback) {
                led_callback(led_on);
            }
        }
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT 오류 발생");
        break;
        
    default:
        ESP_LOGD(TAG, "기타 MQTT 이벤트: %d", event->event_id);
        break;
    }
}

esp_err_t mqtt_manager_init(void)
{
    ESP_LOGI(TAG, "MQTT 클라이언트 초기화");
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .broker.address.port = MQTT_BROKER_PORT,
        .credentials.client_id = MQTT_CLIENT_ID,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT 클라이언트 초기화 실패");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t ret = esp_mqtt_client_start(mqtt_client);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MQTT 클라이언트 시작됨 (브로커: %s)", MQTT_BROKER_URL);
    } else {
        ESP_LOGE(TAG, "MQTT 클라이언트 시작 실패: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

bool mqtt_manager_is_connected(void)
{
    return s_is_connected && (mqtt_client != NULL);
}

int mqtt_manager_publish(const char* topic, const char* data, int len, int qos, int retain)
{
    if (mqtt_client == NULL || !s_is_connected) {
        ESP_LOGW(TAG, "MQTT 클라이언트가 연결되지 않음");
        return -1;
    }
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, data, len, qos, retain);
    ESP_LOGI(TAG, "메시지 발행: %s -> %.*s (msg_id=%d)", topic, len, data, msg_id);
    return msg_id;
}

int mqtt_manager_subscribe(const char* topic, int qos)
{
    if (mqtt_client == NULL || !s_is_connected) {
        ESP_LOGW(TAG, "MQTT 클라이언트가 연결되지 않음");
        return -1;
    }
    
    int msg_id = esp_mqtt_client_subscribe(mqtt_client, topic, qos);
    ESP_LOGI(TAG, "토픽 구독: %s (msg_id=%d)", topic, msg_id);
    return msg_id;
}

void mqtt_manager_register_led_callback(mqtt_led_callback_t callback)
{
    led_callback = callback;
}

