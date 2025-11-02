"""
ESP32 서버 통합 테스트
"""
import pytest
import requests
import time

# 테스트 설정
ESP32_SERVER_URL = "http://localhost:3000"
ESP32_DEVICE_IP = "172.30.1.92"
TIMEOUT = 5

@pytest.fixture
def server_running():
    """서버가 실행 중인지 확인"""
    try:
        response = requests.get(f"{ESP32_SERVER_URL}/api/system/info", timeout=TIMEOUT)
        return response.status_code == 200
    except:
        return False

def test_server_health(server_running):
    """서버 상태 확인"""
    assert server_running, "서버가 실행되지 않았습니다"

def test_esp32_status_api(server_running):
    """ESP32 상태 조회 API 테스트"""
    if not server_running:
        pytest.skip("서버가 실행되지 않았습니다")
    
    response = requests.get(
        f"{ESP32_SERVER_URL}/api/esp32/status",
        timeout=TIMEOUT
    )
    assert response.status_code in [200, 500], "ESP32 상태 API 오류"
    # 500은 ESP32 연결 실패일 수 있음

def test_led_control_api(server_running):
    """LED 제어 API 테스트"""
    if not server_running:
        pytest.skip("서버가 실행되지 않았습니다")
    
    # LED 켜기
    response = requests.post(
        f"{ESP32_SERVER_URL}/api/esp32/led",
        json={"state": True},
        timeout=TIMEOUT
    )
    assert response.status_code == 200, "LED 켜기 실패"
    
    time.sleep(0.5)
    
    # LED 끄기
    response = requests.post(
        f"{ESP32_SERVER_URL}/api/esp32/led",
        json={"state": False},
        timeout=TIMEOUT
    )
    assert response.status_code == 200, "LED 끄기 실패"

def test_config_api(server_running):
    """설정 조회 API 테스트"""
    if not server_running:
        pytest.skip("서버가 실행되지 않았습니다")
    
    response = requests.get(
        f"{ESP32_SERVER_URL}/api/config",
        timeout=TIMEOUT
    )
    assert response.status_code == 200, "설정 조회 실패"
    data = response.json()
    assert "thresholds" in data, "임계치 정보 누락"

@pytest.mark.parametrize("endpoint", [
    "/api/system/info",
    "/api/config",
    "/api/zones"
])
def test_get_endpoints(server_running, endpoint):
    """GET 엔드포인트 테스트"""
    if not server_running:
        pytest.skip("서버가 실행되지 않았습니다")
    
    response = requests.get(
        f"{ESP32_SERVER_URL}{endpoint}",
        timeout=TIMEOUT
    )
    assert response.status_code == 200, f"{endpoint} 요청 실패"


