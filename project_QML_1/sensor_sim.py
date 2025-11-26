import time
import random
import paho.mqtt.client as mqtt

# ✅ MQTT 설정
#BROKER = "test.mosquitto.org"
BROKER = "broker.hivemq.com"
PORT = 1883

TOPIC_TEMPERATURE = "factory/temperature"
TOPIC_HUMIDITY = "factory/humidity"
TOPIC_DUST = "factory/dust"

# ✅ MQTT 클라이언트 생성
client = mqtt.Client()
client.connect(BROKER, PORT, 60)

print("✅ MQTT 연결 완료. 센서 데이터 발행 시작...\n")

# ✅ 무한 루프: 주기적으로 데이터 발행
try:
    while True:
        # 샘플 센서값 (랜덤 변동)
        temperature = 27 + random.uniform(-1.5, 1.5)
        humidity = 44 + random.uniform(-3, 3)
        dust = 6.5 + random.uniform(-1, 1)

        # MQTT 메시지 발행
        client.publish(TOPIC_TEMPERATURE, f"{temperature:.2f}")
        client.publish(TOPIC_HUMIDITY, f"{humidity:.2f}")
        client.publish(TOPIC_DUST, f"{dust:.2f}")

        print(f"📡 발행됨 → T={temperature:.2f}°C, H={humidity:.2f}%, D={dust:.2f}㎍/㎥")

        time.sleep(2.0)  # 2초 간격으로 발행

except KeyboardInterrupt:
    print("\n❌ 발행 중단됨.")
    client.disconnect()
