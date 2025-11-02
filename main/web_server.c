#include "web_server.h"
#include "config.h"
#include "led_control.h"
#include "rgb_led_control.h"
#include "buzzer_control.h"
#include "sensor_average.h"
#include "mqtt_manager.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

static const char *TAG = "web_server";
static httpd_handle_t server = NULL;

// 메인 페이지 핸들러
static esp_err_t root_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "루트 페이지 요청");
    
    const char* html_page = 
        "<!DOCTYPE html><html lang='ko'><head>"
        "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
        "<title>ESP32 센서 제어</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;max-width:900px;margin:20px auto;padding:20px;background:#f0f0f0;}"
        ".container{background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);margin-bottom:20px;}"
        "h1{color:#333;text-align:center;margin-bottom:30px;}"
        "h2{color:#555;border-bottom:2px solid #007bff;padding-bottom:10px;}"
        ".sensor-box{padding:20px;margin:15px 0;border-radius:8px;background:#f8f9fa;}"
        ".sensor-value{font-size:24px;font-weight:bold;color:#007bff;}"
        ".avg-value{font-size:18px;color:#28a745;margin-top:10px;}"
        ".controls{text-align:center;margin:20px 0;}"
        "button{background:#007bff;color:white;border:none;padding:12px 25px;margin:5px;border-radius:5px;cursor:pointer;font-size:14px;}"
        "button:hover{background:#0056b3;}"
        ".btn-red{background:#dc3545;}.btn-red:hover{background:#c82333;}"
        ".btn-green{background:#28a745;}.btn-green:hover{background:#218838;}"
        ".btn-blue{background:#007bff;}.btn-blue:hover{background:#0056b3;}"
        ".btn-purple{background:#6f42c1;}.btn-purple:hover{background:#5a32a3;}"
        ".rgb-controls{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;}"
        ".status{text-align:center;padding:10px;border-radius:5px;margin:10px 0;}"
        ".status-on{background:#d4edda;color:#155724;}.status-off{background:#f8d7da;color:#721c24;}"
        "</style></head><body>"
        "<div class='container'><h1>🌡️ DHT11 온습도 센서</h1>"
        "<div class='sensor-box'>"
        "<h3>실시간 데이터</h3>"
        "<div>온도: <span class='sensor-value' id='currentTemp'>-</span>°C</div>"
        "<div>습도: <span class='sensor-value' id='currentHum'>-</span>%</div>"
        "</div>"
        "<div class='sensor-box'>"
        "<h3>5분 평균</h3>"
        "<div>평균 온도: <span class='avg-value' id='avgTemp'>계산 중...</span>°C</div>"
        "<div>평균 습도: <span class='avg-value' id='avgHum'>계산 중...</span>%</div>"
        "</div></div>"
        "<div class='container'><h1>🎨 RGB LED 제어</h1>"
        "<div class='controls'>"
        "<div class='rgb-controls'>"
        "<button class='btn-red' onclick=\"setRGBColor('red')\">빨강</button>"
        "<button class='btn-green' onclick=\"setRGBColor('green')\">초록</button>"
        "<button class='btn-blue' onclick=\"setRGBColor('blue')\">파랑</button>"
        "<button onclick=\"setRGBColor('off')\">끄기</button>"
        "</div></div></div>"
        "<div class='container'><h1>🔊 부저 제어</h1>"
        "<div class='controls'>"
        "<button class='btn-purple' onclick='controlBuzzer(true)'>부저 ON</button>"
        "<button onclick='controlBuzzer(false)'>부저 OFF</button>"
        "</div>"
        "<div class='status status-off' id='buzzerStatus'>부저: OFF</div></div>"
        "<script>"
        "function updateSensor(){"
        "fetch('/sensor').then(r=>r.json()).then(d=>{"
        "if(d.current){"
        "document.getElementById('currentTemp').textContent=d.current.temperature.toFixed(1);"
        "document.getElementById('currentHum').textContent=d.current.humidity.toFixed(1);"
        "}"
        "if(d.average_5min){"
        "document.getElementById('avgTemp').textContent=d.average_5min.temperature.toFixed(1);"
        "document.getElementById('avgHum').textContent=d.average_5min.humidity.toFixed(1);"
        "}else{document.getElementById('avgTemp').textContent='데이터 부족';document.getElementById('avgHum').textContent='데이터 부족';}"
        "}).catch(e=>console.error('센서 읽기 실패:',e));"
        "}"
        "function setRGBColor(color){"
        "fetch('/rgb_led',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({color:color})})"
        ".then(r=>r.json()).then(d=>console.log('RGB LED 제어:',d)).catch(e=>alert('RGB LED 제어 실패!'));"
        "}"
        "function controlBuzzer(state){"
        "fetch('/buzzer',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({state:state})})"
        ".then(r=>r.json()).then(d=>{"
        "const status=document.getElementById('buzzerStatus');"
        "status.textContent='부저: '+(d.state?'ON':'OFF');"
        "status.className='status '+(d.state?'status-on':'status-off');"
        "}).catch(e=>alert('부저 제어 실패!'));"
        "}"
        "setInterval(updateSensor,2000);updateSensor();"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

// LED 상태 조회 핸들러
static esp_err_t status_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "상태 조회 요청");
    
    bool state = led_control_get_state();
    char response[100];
    snprintf(response, sizeof(response), "{\"state\": %s}", state ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// LED 제어 핸들러
static esp_err_t led_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "LED 제어 요청");
    
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    // JSON 파싱 (간단한 방식)
    char *state_str = strstr(buf, "\"state\":");
    if (state_str) {
        state_str += 8; // "state": 부분 건너뛰기
        esp_err_t err = ESP_OK;
        
        if (strstr(state_str, "true")) {
            err = led_control_on();
            ESP_LOGI(TAG, "LED 켜기 (웹 요청)");
            
            // MQTT로 상태 발행
            if (mqtt_manager_is_connected()) {
                mqtt_manager_publish(MQTT_TOPIC_LED, "on", 2, 0, 1);
                
                // 상태 토픽에도 발행
                char status_msg[50];
                snprintf(status_msg, sizeof(status_msg), "{\"led\":true}");
                mqtt_manager_publish(MQTT_TOPIC_STATUS, status_msg, strlen(status_msg), 0, 1);
            }
        } else if (strstr(state_str, "false")) {
            err = led_control_off();
            ESP_LOGI(TAG, "LED 끄기 (웹 요청)");
            
            // MQTT로 상태 발행
            if (mqtt_manager_is_connected()) {
                mqtt_manager_publish(MQTT_TOPIC_LED, "off", 3, 0, 1);
                
                // 상태 토픽에도 발행
                char status_msg[50];
                snprintf(status_msg, sizeof(status_msg), "{\"led\":false}");
                mqtt_manager_publish(MQTT_TOPIC_STATUS, status_msg, strlen(status_msg), 0, 1);
            }
        }
        
        if (err != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "LED 제어 실패");
            return ESP_FAIL;
        }
    }
    
    bool state = led_control_get_state();
    char response[100];
    snprintf(response, sizeof(response), "{\"state\": %s, \"success\": true}", state ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// 센서 데이터 조회 핸들러 (실시간 + 5분 평균)
static esp_err_t sensor_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "센서 데이터 조회 요청");
    
    float current_temp = 0.0f, current_hum = 0.0f;
    float avg_temp = 0.0f, avg_hum = 0.0f;
    
    bool has_current = sensor_average_get_current(&current_temp, &current_hum);
    bool has_avg = sensor_average_get_5min(&avg_temp, &avg_hum);
    
    char response[200];
    if (has_current && has_avg) {
        snprintf(response, sizeof(response), 
            "{\"current\":{\"temperature\":%.1f,\"humidity\":%.1f},\"average_5min\":{\"temperature\":%.1f,\"humidity\":%.1f}}",
            current_temp, current_hum, avg_temp, avg_hum);
    } else if (has_current) {
        snprintf(response, sizeof(response), 
            "{\"current\":{\"temperature\":%.1f,\"humidity\":%.1f},\"average_5min\":null}",
            current_temp, current_hum);
    } else {
        snprintf(response, sizeof(response), "{\"current\":null,\"average_5min\":null}");
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// 부저 제어 핸들러
static esp_err_t buzzer_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "부저 제어 요청");
    
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    esp_err_t err = ESP_OK;
    char *state_str = strstr(buf, "\"state\":");
    if (state_str) {
        state_str += 8;
        if (strstr(state_str, "true")) {
            err = buzzer_on();
            ESP_LOGI(TAG, "부저 켜기 (웹 요청)");
        } else if (strstr(state_str, "false")) {
            err = buzzer_off();
            ESP_LOGI(TAG, "부저 끄기 (웹 요청)");
        }
    }
    
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "부저 제어 실패");
        return ESP_FAIL;
    }
    
    bool state = buzzer_get_state();
    char response[100];
    snprintf(response, sizeof(response), "{\"state\": %s, \"success\": true}", state ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

// RGB LED 제어 핸들러
static esp_err_t rgb_led_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "RGB LED 제어 요청");
    
    char buf[200];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    esp_err_t err = ESP_OK;
    
    // JSON 파싱: {"color":"red"} 또는 {"r":255,"g":0,"b":0}
    if (strstr(buf, "\"color\"")) {
        if (strstr(buf, "\"red\"")) {
            err = rgb_led_set_red();
        } else if (strstr(buf, "\"green\"")) {
            err = rgb_led_set_green();
        } else if (strstr(buf, "\"blue\"")) {
            err = rgb_led_set_blue();
        } else if (strstr(buf, "\"off\"")) {
            err = rgb_led_off();
        }
    } else if (strstr(buf, "\"r\"") && strstr(buf, "\"g\"") && strstr(buf, "\"b\"")) {
        // RGB 값 직접 설정
        uint8_t r = 0, g = 0, b = 0;
        char *r_str = strstr(buf, "\"r\":");
        char *g_str = strstr(buf, "\"g\":");
        char *b_str = strstr(buf, "\"b\":");
        if (r_str && g_str && b_str) {
            unsigned int r_val = 0, g_val = 0, b_val = 0;
            sscanf(r_str + 4, "%u", &r_val);
            sscanf(g_str + 4, "%u", &g_val);
            sscanf(b_str + 4, "%u", &b_val);
            r = (uint8_t)(r_val & 0xFF);
            g = (uint8_t)(g_val & 0xFF);
            b = (uint8_t)(b_val & 0xFF);
            err = rgb_led_set_color(r, g, b);
        }
    }
    
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "RGB LED 제어 실패");
        return ESP_FAIL;
    }
    
    char response[100];
    snprintf(response, sizeof(response), "{\"success\": true}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

esp_err_t web_server_init(void)
{
    ESP_LOGI(TAG, "HTTP 웹서버 초기화");
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_SERVER_PORT;
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "서버 시작: 포트 %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "URI 핸들러 등록");
        
        // 루트 페이지
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        // 상태 조회
        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);
        
        // LED 제어
        httpd_uri_t led_uri = {
            .uri       = "/led",
            .method    = HTTP_POST,
            .handler   = led_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &led_uri);
        
        // 센서 데이터 조회
        httpd_uri_t sensor_uri = {
            .uri       = "/sensor",
            .method    = HTTP_GET,
            .handler   = sensor_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &sensor_uri);
        
        // 부저 제어
        httpd_uri_t buzzer_uri = {
            .uri       = "/buzzer",
            .method    = HTTP_POST,
            .handler   = buzzer_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &buzzer_uri);
        
        // RGB LED 제어
        httpd_uri_t rgb_led_uri = {
            .uri       = "/rgb_led",
            .method    = HTTP_POST,
            .handler   = rgb_led_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &rgb_led_uri);
        
        ESP_LOGI(TAG, "✅ HTTP 웹서버 시작 완료 (포트: %d)", config.server_port);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "❌ HTTP 웹서버 시작 실패");
    return ESP_FAIL;
}

void web_server_deinit(void)
{
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
        ESP_LOGI(TAG, "HTTP 웹서버 중지됨");
    }
}

