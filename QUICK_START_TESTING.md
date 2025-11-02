# 테스트 빠른 시작 가이드

## 🚀 빠른 시작 (5분)

### 1. 유닛 테스트 실행

```bash
# 빌드
idf.py build

# 테스트 실행
idf.py test
```

### 2. 정적 분석 실행

**Windows:**
```powershell
# cppcheck 설치 (Chocolatey)
choco install cppcheck

# 실행
cppcheck --enable=all -I main/include main/*.c
```

**Linux/macOS:**
```bash
# 설치
sudo apt-get install cppcheck  # Ubuntu
brew install cppcheck          # macOS

# 실행
./tools/run_tests.sh
```

### 3. 통합 테스트 (Python)

```bash
# 의존성 설치
pip install pytest requests

# 테스트 실행
pytest tests/ -v
```

---

## 📊 테스트 커버리지 확인

### 커버리지 활성화

1. **menuconfig 열기:**
```bash
idf.py menuconfig
```

2. **설정 변경:**
   - `Component config` → `Component` → `Unit test configuration`
   - `Enable coverage` 체크

3. **빌드 및 테스트:**
```bash
idf.py build
idf.py test
```

4. **커버리지 리포트 생성 (Linux/macOS):**
```bash
# lcov 설치
sudo apt-get install lcov  # Ubuntu
brew install lcov          # macOS

# 리포트 생성
lcov --capture --directory build --output-file coverage.info --no-external
genhtml coverage.info --output-directory coverage_html

# 리포트 확인
open coverage_html/index.html  # macOS
xdg-open coverage_html/index.html  # Linux
```

---

## 🛠️ 사용 가능한 명령어

### Makefile 사용 (Linux/macOS)

```bash
make test              # 모든 테스트 실행
make test-unit         # 유닛 테스트만
make test-integration  # 통합 테스트만
make static-analysis   # 정적 분석
make coverage          # 커버리지 리포트
make clean-test        # 테스트 파일 정리
```

### 직접 실행

```bash
# 유닛 테스트
idf.py test

# 통합 테스트
pytest tests/

# 정적 분석
./tools/run_tests.sh    # Linux/macOS
tools\run_tests.bat     # Windows
```

---

## 📝 테스트 파일 위치

```
project-mqtt/
├── main/
│   └── test/
│       ├── CMakeLists.txt
│       └── test_led_control_simple.c    # LED 제어 테스트
├── tests/
│   └── test_integration.py              # 통합 테스트
├── tools/
│   ├── run_tests.sh                     # 테스트 스크립트 (Linux/macOS)
│   └── run_tests.bat                    # 테스트 스크립트 (Windows)
└── .github/
    └── workflows/
        └── test.yml                     # CI/CD 자동 테스트
```

---

## ✅ 체크리스트

코드 커밋 전:

- [ ] `idf.py test` 통과
- [ ] `pytest tests/` 통과 (서버 실행 시)
- [ ] 정적 분석 경고 확인
- [ ] 커버리지 60% 이상 (권장)

---

## 🐛 문제 해결

### 테스트가 실행되지 않음
```bash
idf.py fullclean
idf.py build
idf.py test
```

### Unity 컴포넌트 오류
```bash
idf_component_manager add-idf-component unity
idf.py build
```

### 커버리지가 0%
- menuconfig에서 `Enable coverage` 확인
- `idf.py fullclean` 후 재빌드

---

자세한 내용은 `TESTING.md`를 참조하세요.


