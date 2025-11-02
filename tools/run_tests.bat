@echo off
REM ESP32 프로젝트 테스트 실행 스크립트 (Windows)

set PROJECT_DIR=%~dp0..
set BUILD_DIR=%PROJECT_DIR%\build

echo ==========================================
echo ESP32 프로젝트 테스트 실행
echo ==========================================

REM 1. 정적 분석 실행
echo.
echo [1/4] 정적 분석 실행 (cppcheck)...
where cppcheck >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    cppcheck --enable=all ^
        --suppress=missingIncludeSystem ^
        --suppress=unusedFunction ^
        --xml --xml-version=2 ^
        --output-file=cppcheck_report.xml ^
        -I main\include ^
        main\*.c main\test\*.c 2>&1 | tee cppcheck_output.txt
    echo ✅ cppcheck 완료: cppcheck_report.xml
) else (
    echo ⚠️  cppcheck가 설치되지 않았습니다. 건너뜁니다.
)

REM 2. 빌드 및 테스트
echo.
echo [2/3] 빌드 및 테스트 실행...
cd /d "%PROJECT_DIR%"
if idf.py build 2>&1 | tee build_output.txt (
    echo ✅ 빌드 성공
    
    REM ESP-IDF 테스트 실행
    idf.py test 2>&1 | tee test_output.txt
    if %ERRORLEVEL% EQU 0 (
        echo ✅ 테스트 완료
    ) else (
        echo ❌ 테스트 실패
        exit /b 1
    )
) else (
    echo ❌ 빌드 실패
    exit /b 1
)

REM 3. 커버리지 (Windows에서는 제한적)
echo.
echo [3/3] 커버리지 정보 수집...
echo ⚠️  Windows에서는 커버리지 도구 사용이 제한적입니다.
echo     Linux/WSL 환경에서 실행하거나 Jenkins/GitHub Actions를 사용하세요.

echo.
echo ==========================================
echo ✅ 테스트 완료
echo ==========================================


