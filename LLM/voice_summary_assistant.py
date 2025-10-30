# voice_summary_assistant.py
import ollama
import websocket
import whisper
import sounddevice as sd
import numpy as np
import paho.mqtt.client as mqtt # MQTT 추가
import json
import pyttsx3 # TTS 추가
import threading # MQTT 수신을 백그라운드에서 처리하기 위해 추가
import io, sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8') # 인코딩 설정

# --- 설정 ---
SAMPLE_RATE = 16000
DURATION = 5
WHISPER_MODEL = "base"
OLLAMA_MODEL = "phi3:mini"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_SENSOR_TOPIC = "factory/sensors"

# --- LLM 프롬프트 ---
SUMMARY_SYSTEM_PROMPT = """
You are a helpful AI assistant for a factory manager.
Analyze sensor data and provide a brief, human-readable summary in Korean.
Your response MUST be ONLY in Korean and follow the given format exactly.

---
### EXAMPLE
User Input: Final Temp: 55.2, Max Temp: 58.1, Max Vibration: 1.8
Your Correct Response:
- 최종 온도: 55.2 °C
- 최고 온도: 58.1 °C
- 특이사항: 온도가 50도를 초과했으며, 진동 수치가 1.5를 초과했습니다. 점검이 필요합니다.
---
Now, create a report for the following data.
"""

# --- 전역 변수 ---
sensor_data_history = [] # MQTT 데이터 저장 리스트
tts_engine = pyttsx3.init() # TTS 엔진 초기화

# --- 함수 정의 ---
def speak(text):
    """텍스트를 음성으로 출력하는 함수"""
    print(f"🔊 AI 답변: {text}")
    tts_engine.say(text)
    tts_engine.runAndWait()

def summarize_data():
    """누적된 데이터를 LLM으로 요약하는 함수"""
    if not sensor_data_history:
        return "요약할 데이터가 없습니다."

    temperatures = []
    vibrations = []
    for msg in sensor_data_history:
        try:
            data = json.loads(msg)
            if 'temperature' in data: temperatures.append(data['temperature'])
            if 'vibration' in data: vibrations.append(data['vibration'])
        except json.JSONDecodeError: continue

    if not temperatures: return "유효한 온도 데이터가 없습니다."

    final_temp = temperatures[-1]; max_temp = max(temperatures)
    max_vib = max(vibrations) if vibrations else 0
    data_for_llm = f"Final Temp: {final_temp}, Max Temp: {max_temp}, Max Vibration: {max_vib}"

    try:
        response = ollama.chat( model=OLLAMA_MODEL, messages=[ {'role': 'system', 'content': SUMMARY_SYSTEM_PROMPT}, {'role': 'user', 'content': data_for_llm} ] )
        return response['message']['content']
    except Exception as e:
        return f"LLM 요약 중 오류 발생: {e}"

# --- MQTT 콜백 함수 ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("MQTT 브로커에 연결되었습니다. 센서 데이터를 수신합니다...")
        client.subscribe(MQTT_SENSOR_TOPIC)
    else:
        print(f"MQTT 연결 실패 (코드: {rc})")

def on_message(client, userdata, msg):
    """MQTT 메시지 수신 시 데이터 저장"""
    payload = msg.payload.decode('utf-8')
    print(f"Received MQTT: {payload}") # 디버깅용 로그
    sensor_data_history.append(payload)
    # 메모리 관리를 위해 오래된 데이터 삭제 (예: 최근 100개만 저장)
    if len(sensor_data_history) > 100:
        del sensor_data_history[0]

# --- MQTT 클라이언트 설정 및 백그라운드 실행 ---
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
try:
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    # MQTT 클라이언트를 별도의 스레드에서 실행하여 메인 프로그램(음성인식)을 방해하지 않도록 함
    mqtt_thread = threading.Thread(target=mqtt_client.loop_forever)
    mqtt_thread.daemon = True # 메인 프로그램 종료 시 함께 종료되도록 설정
    mqtt_thread.start()
except Exception as e:
    print(f"MQTT 연결 시도 중 오류 발생: {e}")

# --- Whisper 모델 로딩 ---
print("Whisper 모델을 로딩합니다...")
asr_model = whisper.load_model(WHISPER_MODEL)
print("AI 어시스턴트가 준비되었습니다.")

# --- 메인 루프 (음성 명령 처리) ---
try:
    while True:
        input("\n🎙️ 엔터 키를 누르고 5초간 명령을 말씀하세요...")
        print("듣는 중...")
        recording = sd.rec(int(DURATION * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1)
        sd.wait()
        print("음성 인식 중...")
        audio_data = recording.flatten().astype(np.float32)
        transcribed_result = asr_model.transcribe(audio_data, language='ko', fp16=False)
        user_command = transcribed_result["text"].strip().lower() # 소문자로 변환하여 비교

        if not user_command:
            print("인식된 음성이 없습니다.")
            continue
        
        print(f"👤 당신의 명령: {user_command}")

        # --- 명령 분석 및 실행 ---
        if "요약" in user_command or "상태" in user_command or "보고" in user_command:
            # '요약' 관련 명령 처리
            summary_text = summarize_data()
            speak(summary_text) # 요약 결과를 음성으로 출력
            
        else:
            speak("죄송합니다. 이해하지 못했습니다. '상태 요약' 또는 '보고해 줘' 라고 말씀해 보세요.")

except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")
    if mqtt_client.is_connected():
        mqtt_client.disconnect()