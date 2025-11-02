#!/usr/bin/env python3
"""
ESP32 C 코드 빌드 및 실행 가이드
"""

print("=" * 60)
print("🚀 ESP32 C 코드 WiFi & 웹서버 빌드 가이드")
print("=" * 60)

print("""
📋 준비사항:
1. ESP-IDF 환경 설치
2. ESP32 개발 보드
3. USB 케이블
4. WiFi 네트워크 접근

🔧 설정된 WiFi 정보:
- SSID: KT_GiGA_5G_2371
- Password: 6kg52ke294

📁 프로젝트 구조:
- main/blink_example_main.c - 메인 C 코드
- main/CMakeLists.txt - 빌드 설정
- sdkconfig.defaults - 프로젝트 설정

🚀 빌드 및 실행 방법:

1️⃣ ESP-IDF 환경 설정:
   # Windows에서 ESP-IDF 명령 프롬프트 실행
   # 또는 PowerShell에서:
   $env:IDF_PATH = "C:\esp\esp-idf"
   & "$env:IDF_PATH\export.ps1"

2️⃣ 프로젝트 빌드:
   idf.py set-target esp32s3
   idf.py build

3️⃣ ESP32에 플래시:
   idf.py -p COM3 flash

4️⃣ 시리얼 모니터 실행:
   idf.py -p COM3 monitor

5️⃣ 빌드와 플래시를 동시에:
   idf.py -p COM3 flash monitor

🎯 예상 출력:
I (1234) webserver: ESP32 WiFi & WebServer 시작
I (1235) webserver: WiFi 연결 중... SSID: KT_GiGA_5G_2371
I (2345) webserver: connected to ap SSID:KT_GiGA_5G_2371 password:6kg52ke294
I (2346) webserver: got ip:192.168.1.100
I (2347) webserver: Starting server on port: '80'
I (2348) webserver: Registering URI handlers
I (2349) webserver: ✅ 웹서버 시작 완료!
I (2350) webserver: 🌐 브라우저에서 접속하세요!

🌐 웹 브라우저 접속:
- 시리얼 모니터에서 IP 주소 확인
- 브라우저에서 http://[IP주소] 접속
- LED 제어 버튼 테스트

📱 주요 기능:
- WiFi 자동 연결
- 웹 인터페이스를 통한 LED 제어
- 실시간 상태 표시
- 모바일 지원
- JSON API 제공

❓ 문제 해결:
- WiFi 연결 실패: SSID/비밀번호 확인
- 빌드 오류: ESP-IDF 환경 확인
- 플래시 실패: 포트 번호 및 드라이버 확인
- 웹서버 접속 불가: 방화벽 설정 확인

🔧 추가 설정:
- WiFi 정보 변경: main/blink_example_main.c의 WIFI_SSID, WIFI_PASS 수정
- LED 핀 변경: LED_GPIO_PIN 값 수정
- 포트 변경: HTTPD_DEFAULT_CONFIG에서 설정

📊 API 엔드포인트:
- GET / - 메인 웹페이지
- GET /status - LED 상태 조회 (JSON)
- POST /led - LED 제어 (JSON: {"state": true/false})
""")

print("=" * 60)
print("✅ C 코드 구현 완료! 빌드하고 실행하세요.")
print("=" * 60)
