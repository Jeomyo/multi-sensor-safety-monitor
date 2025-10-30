import sys
import ollama
import json
import io

# 1. UTF-8 인코딩 설정 (오류 방지)
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# 2. LLM에게 구체적인 임무와 답변 형식을 지시하는 프롬프트
SYSTEM_PROMPT = """
You are a helpful AI assistant for a factory manager.
Your task is to analyze sensor data and provide a brief, human-readable summary in Korean.
Your response MUST be ONLY in Korean and MUST follow the given format exactly. Do not add any other text, instructions, or comments.

---
### EXAMPLE
User Input: Final Temp: 55.2, Max Temp: 58.1, Max Vibration: 1.8

Your Correct Response:
- 최종 온도: 55.2 °C
- 최고 온도: 58.1 °C
- 특이사항: 온도가 50도를 초과했으며, 진동 수치가 1.5를 초과했습니다. 점검이 필요합니다.
---

Now, based on the rules and example above, create a report for the following data.
"""
# 3. C++로부터 받은 데이터를 가공하는 부분
try:
    raw_data_string = sys.argv[1]
    
    # 여러 줄의 JSON 문자열을 각각 파싱하여 온도 값만 리스트로 추출
    temperatures = []
    vibrations = []
    lines = raw_data_string.strip().split('\n')
    for line in lines:
        try:
            data = json.loads(line)
            if 'temperature' in data:
                temperatures.append(data['temperature'])
            if 'vibration' in data:
                vibrations.append(data['vibration'])
        except json.JSONDecodeError:
            continue # 파싱 실패 시 무시

    if not temperatures:
        raise ValueError("No valid temperature data found.")

    # 4. 분석에 필요한 값들 계산
    final_temp = temperatures[-1]
    max_temp = max(temperatures)
    max_vib = max(vibrations) if vibrations else 0
    
    # LLM에게 전달할 간결한 데이터 문자열 생성
    data_for_llm = f"Final Temp: {final_temp}, Max Temp: {max_temp}, Max Vibration: {max_vib}"

    # 5. Ollama LLM 호출
    response = ollama.chat(
        model="phi3:mini",
        messages=[
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': data_for_llm}
        ]
    )
    # 요약된 결과만 출력
    print(response['message']['content'])

except Exception as e:
    print(f"Error processing data: {e}")