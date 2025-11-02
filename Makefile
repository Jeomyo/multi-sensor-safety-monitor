# ESP32 프로젝트 테스트 Makefile

.PHONY: test test-unit test-integration coverage static-analysis clean-test help

# 기본 테스트 실행
test: test-unit static-analysis

# 유닛 테스트
test-unit:
	@echo "유닛 테스트 실행..."
	idf.py build
	idf.py test

# 통합 테스트 (Python)
test-integration:
	@echo "통합 테스트 실행..."
	pytest tests/ -v

# 정적 분석
static-analysis:
	@echo "정적 분석 실행..."
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --suppress=missingIncludeSystem \
			-I main/include main/*.c main/test/*.c; \
	else \
		echo "cppcheck가 설치되지 않았습니다."; \
	fi

# 커버리지 리포트
coverage:
	@echo "커버리지 리포트 생성..."
	@idf.py menuconfig
	@idf.py build
	@idf.py test
	@if command -v lcov >/dev/null 2>&1; then \
		lcov --capture --directory build --output-file coverage.info --no-external; \
		genhtml coverage.info --output-directory coverage_html; \
		echo "커버리지 리포트: coverage_html/index.html"; \
	else \
		echo "lcov가 설치되지 않았습니다."; \
	fi

# 테스트 정리
clean-test:
	rm -rf coverage_html coverage.info
	rm -f cppcheck_report.xml clang_tidy_output.txt test_output.txt

# 도움말
help:
	@echo "사용 가능한 명령어:"
	@echo "  make test            - 모든 테스트 실행"
	@echo "  make test-unit       - 유닛 테스트만 실행"
	@echo "  make test-integration - 통합 테스트만 실행"
	@echo "  make static-analysis - 정적 분석 실행"
	@echo "  make coverage        - 커버리지 리포트 생성"
	@echo "  make clean-test      - 테스트 파일 정리"


