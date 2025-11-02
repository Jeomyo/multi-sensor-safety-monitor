#!/bin/bash
# ESP32 프로젝트 테스트 실행 스크립트

set -e

PROJECT_DIR=$(dirname $(dirname $(realpath $0)))
BUILD_DIR="${PROJECT_DIR}/build"

echo "=========================================="
echo "ESP32 프로젝트 테스트 실행"
echo "=========================================="

# 1. 정적 분석 실행
echo ""
echo "[1/4] 정적 분석 실행 (cppcheck)..."
if command -v cppcheck &> /dev/null; then
    cppcheck \
        --enable=all \
        --suppress=missingIncludeSystem \
        --suppress=unusedFunction \
        --xml --xml-version=2 \
        --output-file=cppcheck_report.xml \
        --includes-file=- \
        -I main/include \
        -I $IDF_PATH/components \
        main/*.c \
        main/test/*.c 2>&1 | tee cppcheck_output.txt
    echo "✅ cppcheck 완료: cppcheck_report.xml"
else
    echo "⚠️  cppcheck가 설치되지 않았습니다. 건너뜁니다."
fi

# 2. clang-tidy 실행
echo ""
echo "[2/4] 정적 분석 실행 (clang-tidy)..."
if command -v clang-tidy &> /dev/null; then
    if [ -f "${BUILD_DIR}/compile_commands.json" ]; then
        clang-tidy \
            -p "${BUILD_DIR}" \
            main/*.c \
            -- \
            -I main/include \
            -I $IDF_PATH/components 2>&1 | tee clang_tidy_output.txt || true
        echo "✅ clang-tidy 완료: clang_tidy_output.txt"
    else
        echo "⚠️  compile_commands.json이 없습니다. 먼저 빌드를 실행하세요."
    fi
else
    echo "⚠️  clang-tidy가 설치되지 않았습니다. 건너뜁니다."
fi

# 3. 유닛 테스트 실행
echo ""
echo "[3/4] 유닛 테스트 실행..."
cd "${PROJECT_DIR}"
if idf.py build 2>&1 | tee build_output.txt; then
    echo "✅ 빌드 성공"
    
    # 테스트 실행 (ESP-IDF 테스트 프레임워크 사용)
    if idf.py test 2>&1 | tee test_output.txt; then
        echo "✅ 테스트 완료"
    else
        echo "❌ 테스트 실패"
        exit 1
    fi
else
    echo "❌ 빌드 실패"
    exit 1
fi

# 4. 커버리지 생성 (gcov 사용)
echo ""
echo "[4/4] 테스트 커버리지 생성..."
if [ -d "${BUILD_DIR}" ]; then
    # 커버리지 정보 수집
    find "${BUILD_DIR}" -name "*.gcda" -exec gcov {} \; 2>/dev/null || true
    find "${BUILD_DIR}" -name "*.gcno" -exec ls -la {} \; 2>/dev/null || true
    
    # 커버리지 리포트 생성 (lcov 사용)
    if command -v lcov &> /dev/null; then
        lcov --capture \
             --directory "${BUILD_DIR}" \
             --output-file coverage.info \
             --no-external \
             --exclude '*/test/*' \
             --exclude '*/main/test/*' || true
        
        genhtml coverage.info --output-directory coverage_html || true
        echo "✅ 커버리지 리포트 생성: coverage_html/index.html"
    else
        echo "⚠️  lcov가 설치되지 않았습니다. 커버리지 리포트를 생성할 수 없습니다."
    fi
fi

echo ""
echo "=========================================="
echo "✅ 모든 테스트 완료"
echo "=========================================="
echo "결과 파일:"
echo "  - cppcheck_report.xml"
echo "  - clang_tidy_output.txt"
echo "  - test_output.txt"
echo "  - coverage_html/index.html (있는 경우)"


