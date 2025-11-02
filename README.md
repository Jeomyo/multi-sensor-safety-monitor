# ESP32 MQTT 프로젝트

ESP32를 사용한 MQTT 통신 프로젝트입니다. WiFi를 통해 MQTT 브로커에 연결하고, LED 제어 및 센서 데이터를 주고받을 수 있습니다.

## 프로젝트 구성

- `testblink.py`: 기본 LED 블링크 예제
- `mqtttest.py`: ESP32 MQTT 클라이언트 (메인 코드)
- `mqtt_config.py`: MQTT 및 WiFi 설정 파일
- `mqtt_client_test.py`: PC용 MQTT 테스트 클라이언트
- `main/blink_example_main.c`: ESP-IDF C 코드 예제

## 주요 기능

### ESP32 MQTT 클라이언트 (`mqtttest.py`)
- WiFi 연결 및 MQTT 브로커 연결
- LED 원격 제어 (MQTT 메시지로 ON/OFF)
- 온도 센서 데이터 발행
- 실시간 상태 모니터링

### PC 테스트 클라이언트 (`mqtt_client_test.py`)
- ESP32와 MQTT 통신 테스트
- LED 제어 명령 전송
- 메시지 수신 모니터링

## 사용 방법

### 1. 설정 파일 수정

`mqtt_config.py` 파일에서 WiFi 및 MQTT 설정을 수정하세요:

```python
WIFI_CONFIG = {
    'ssid': 'your_wifi_ssid',        # 실제 WiFi 이름
    'password': 'your_wifi_password'  # 실제 WiFi 비밀번호
}
```

### 2. ESP32에 MicroPython 설치

ESP32에 MicroPython 펌웨어를 설치해야 합니다:

1. [MicroPython 다운로드](https://micropython.org/download/esp32/)
2. ESP32를 다운로드 모드로 설정 (GPIO0을 GND에 연결)
3. esptool을 사용하여 펌웨어 플래시:
   ```bash
   esptool.py --chip esp32 --port COM3 write_flash -z 0x1000 esp32-20231005-v1.21.0.bin
   ```

### 3. 필요한 라이브러리 설치

ESP32에 MQTT 라이브러리를 설치하세요:

```python
import upip
upip.install('umqtt.simple')
```

### 4. 코드 실행

ESP32에서 `mqtttest.py`를 실행하세요:

```python
exec(open('mqtttest.py').read())
```

### 5. PC 테스트 클라이언트 실행

PC에서 테스트 클라이언트를 실행하여 ESP32와 통신을 테스트하세요:

```bash
pip install paho-mqtt
python mqtt_client_test.py
```

## MQTT 토픽 구조

- `esp32/led/control`: LED 제어 명령 (JSON: `{"state": true/false}`)
- `esp32/led/status`: LED 상태 응답 (JSON: `{"state": true/false, "timestamp": ...}`)
- `esp32/sensor/temperature`: 온도 센서 데이터 (JSON: `{"temperature": 25.5, "unit": "celsius"}`)
- `esp32/system/status`: 시스템 상태 메시지
- `esp32/system/error`: 에러 메시지

## 하드웨어 요구사항

- ESP32 개발 보드
- WiFi 네트워크 접근 가능
- USB 케이블 (전원 공급 및 프로그래밍용)

## ESP-IDF C 코드 사용법

C 코드를 사용하려면 ESP-IDF 환경에서 빌드하고 플래시하세요:

```bash
idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor
```

## Example Output

As you run the example, you will see the LED blinking, according to the previously defined period. For the addressable LED, you can also change the LED color by setting the `led_strip_set_pixel(led_strip, 0, 16, 16, 16);` (LED Strip, Pixel Number, Red, Green, Blue) with values from 0 to 255 in the [source file](main/blink_example_main.c).

```text
I (315) example: Example configured to blink addressable LED!
I (325) example: Turning the LED OFF!
I (1325) example: Turning the LED ON!
I (2325) example: Turning the LED OFF!
I (3325) example: Turning the LED ON!
I (4325) example: Turning the LED OFF!
I (5325) example: Turning the LED ON!
I (6325) example: Turning the LED OFF!
I (7325) example: Turning the LED ON!
I (8325) example: Turning the LED OFF!
```

Note: The color order could be different according to the LED model.

The pixel number indicates the pixel position in the LED strip. For a single LED, use 0.

## Troubleshooting

* If the LED isn't blinking, check the GPIO or the LED type selection in the `Example Configuration` menu.

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
