# 테스트 가이드

ESP32 프로젝트의 테스트 환경 설정 및 사용 방법을 안내합니다.

## 📋 목차

1. [테스트 환경 구성](#테스트-환경-구성)
2. [유닛 테스트](#유닛-테스트)
3. [통합 테스트](#통합-테스트)
4. [정적 분석](#정적-분석)
5. [테스트 커버리지](#테스트-커버리지)
6. [CI/CD 통합](#cicd-통합)

---

## 테스트 환경 구성

### 필수 도구 설치

#### 1. ESP-IDF 테스트 프레임워크
ESP-IDF는 Unity 테스트 프레임워크를 내장하고 있습니다.

```bash
# ESP-IDF 설치 확인
idf.py --version

# Unity 컴포넌트 확인
idf_component_manager list | grep unity
```

#### 2. 정적 분석 도구

**Windows (PowerShell):**
```powershell
# cppcheck 설치 (Chocolatey 사용)
choco install cppcheck

# 또는 직접 다운로드
# https://github.com/danmar/cppcheck/releases

# clang-tidy 설치
choco install llvm
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt-get update
sudo apt-get install -y cppcheck clang-tidy
```

**macOS:**
```bash
brew install cppcheck llvm
```

#### 3. 커버리지 도구

**Linux:**
```bash
sudo apt-get install -y lcov gcov
```

**Windows:**
- WSL2 사용 권장 또는 Jenkins/GitHub Actions 활용

---

## 유닛 테스트

### 테스트 구조

```
main/
├── test/
│   ├── CMakeLists.txt
│   └── test_led_control_simple.c
```

### 테스트 실행

```bash
# 프로젝트 루트에서
idf.py build
idf.py test
```

### 테스트 작성 예제

```c
#include "unity.h"
#include "led_control.h"

TEST_CASE("LED 켜기 테스트", "[led_control]")
{
    led_control_init();
    esp_err_t ret = led_control_on();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_TRUE(led_control_get_state());
}
```

### 테스트 태그 사용

ESP-IDF는 테스트 태그를 지원합니다:

```bash
# 특정 태그만 실행
idf.py test --test-component led_control

# 특정 테스트만 실행
idf.py test --test-name "LED 켜기 테스트"
```

---

## 통합 테스트

### Python 기반 통합 테스트

ESP32 서버와의 통합 테스트는 Python pytest를 사용합니다.

```bash
# 의존성 설치
pip install pytest requests

# 테스트 실행
pytest tests/ -v
```

### 테스트 항목

- 서버 상태 확인
- ESP32 상태 조회 API
- LED 제어 API
- 설정 조회/업데이트 API
- MQTT 메시지 발행/구독

---

## 정적 분석

### cppcheck 실행

```bash
# 기본 실행
cppcheck --enable=all main/*.c

# XML 리포트 생성
cppcheck --enable=all \
  --xml --xml-version=2 \
  --output-file=cppcheck_report.xml \
  -I main/include \
  main/*.c
```

### clang-tidy 실행

```bash
# 먼저 빌드하여 compile_commands.json 생성
idf.py build

# clang-tidy 실행
clang-tidy -p build main/*.c
```

### 자동화 스크립트

```bash
# Linux/macOS
./tools/run_tests.sh

# Windows
tools\run_tests.bat
```

---

## 테스트 커버리지

### 커버리지 활성화

1. **menuconfig에서 설정:**
```bash
idf.py menuconfig
# Component config -> Component -> Unit test configuration
# -> Enable coverage (ON)
```

2. **또는 sdkconfig에 직접 추가:**
```
CONFIG_APP_PROJECT_VER_FROM_CONFIG=y
CONFIG_COMPONENT_COVERAGE_ENABLED=y
```

### 커버리지 리포트 생성

```bash
# 빌드 및 테스트 실행
idf.py build
idf.py test

# 커버리지 정보 수집
lcov --capture \
  --directory build \
  --output-file coverage.info \
  --no-external

# HTML 리포트 생성
genhtml coverage.info --output-directory coverage_html

# 리포트 확인
open coverage_html/index.html  # macOS
xdg-open coverage_html/index.html  # Linux
```

### 커버리지 목표

- **최소 커버리지**: 60%
- **권장 커버리지**: 80%
- **핵심 모듈**: 90% 이상

---

## CI/CD 통합

### GitHub Actions

`.github/workflows/test.yml` 파일이 자동으로 다음을 실행합니다:

1. **정적 분석**
   - cppcheck
   - clang-tidy

2. **유닛 테스트**
   - ESP-IDF 테스트 프레임워크

3. **통합 테스트**
   - Python pytest

4. **커버리지 리포트**
   - Codecov 업로드

### 로컬에서 CI 스크립트 실행

```bash
# Linux/macOS
./tools/run_tests.sh

# Windows
tools\run_tests.bat
```

---

## 테스트 모범 사례

### 1. 테스트 네이밍
- 명확하고 설명적인 이름 사용
- 테스트 대상과 예상 결과를 명시

```c
// 좋은 예
TEST_CASE("LED 켜기 후 상태가 true인지 확인", "[led_control]")

// 나쁜 예
TEST_CASE("test1", "[led]")
```

### 2. 독립적인 테스트
- 각 테스트는 독립적으로 실행 가능해야 함
- setUp/tearDown을 활용하여 상태 초기화

```c
void setUp(void) {
    led_control_init();  // 각 테스트 전 초기화
}

void tearDown(void) {
    // 정리 작업
}
```

### 3. 경계값 테스트
- 정상 케이스뿐만 아니라 예외 케이스도 테스트

```c
TEST_CASE("NULL 포인터 처리", "[mqtt]")
TEST_CASE("빈 문자열 처리", "[web_server]")
```

### 4. 모의(Mock) 객체 활용
- 하드웨어 의존성 제거
- 함수 포인터나 매크로를 사용한 모의 구현

---

## 문제 해결

### 테스트가 실행되지 않음

```bash
# Unity 컴포넌트 확인
idf.py test --list

# 빌드 정리 후 재빌드
idf.py fullclean
idf.py build
idf.py test
```

### 커버리지가 0%로 표시됨

```bash
# 커버리지 설정 확인
grep CONFIG_COMPONENT_COVERAGE_ENABLED sdkconfig

# 재빌드 및 재테스트
idf.py fullclean
idf.py build
idf.py test
```

### 정적 분석 오류가 너무 많음

```bash
# 특정 경고만 활성화
cppcheck --enable=warning,style main/*.c

# 특정 경고 무시
cppcheck --suppress=unusedFunction main/*.c
```

---

## 추가 리소스

- [ESP-IDF 테스트 가이드](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-python-scripts.html#idf-py-test)
- [Unity 테스트 프레임워크](https://github.com/ThrowTheSwitch/Unity)
- [cppcheck 문서](https://cppcheck.sourceforge.io/)
- [pytest 문서](https://docs.pytest.org/)

---

## 테스트 체크리스트

코드 커밋 전 확인 사항:

- [ ] 모든 유닛 테스트 통과
- [ ] 통합 테스트 통과
- [ ] 정적 분석 경고 없음 (또는 허용된 경고만)
- [ ] 테스트 커버리지 60% 이상
- [ ] 새로운 기능에 대한 테스트 추가
- [ ] 문서 업데이트 (필요 시)


