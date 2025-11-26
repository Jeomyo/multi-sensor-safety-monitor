import json, math, random, time
import paho.mqtt.client as mqtt

# MQTT 설정
#BROKER = "test.mosquitto.org"
BROKER = "broker.hivemq.com"
TOPIC_MAP = "/map/data"
TOPIC_WORKER = "/worker/data"

# 컨투어(공장 윤곽선) 불러오기
with open("C:\QT_QML\project_QML_1\contour.json", "r") as f:
    contour = json.load(f)

# 다각형 내부 판정 함수 (Ray casting)
def point_in_polygon(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]["x"], poly[i]["y"]
        x2, y2 = poly[(i+1) % n]["x"], poly[(i+1) % n]["y"]
        if ((y1 > y) != (y2 > y)):
            x_intersect = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-9) + x1
            if x < x_intersect:
                inside = not inside
    return inside

# 다각형의 중심 구하기 (시작 위치용)
cx = sum(p["x"] for p in contour) / len(contour)
cy = sum(p["y"] for p in contour) / len(contour)

# MQTT 연결
client = mqtt.Client()
client.connect(BROKER, 1883, 60)
print("✅ MQTT 브로커 연결 완료")

# 맵(윤곽선) 1회 발행
client.publish(TOPIC_MAP, json.dumps(contour))
print("🗺️ 맵 데이터 발행 완료")

# 작업자 초기 상태 (3명)
workers = []
for i, name in enumerate(["A", "B", "C"]):
    # 컨투어 내부에서 랜덤 좌표 생성
    while True:
        x = random.uniform(min(p["x"] for p in contour), max(p["x"] for p in contour))
        y = random.uniform(min(p["y"] for p in contour), max(p["y"] for p in contour))
        if point_in_polygon(x, y, contour):
            break
    workers.append({
        "name": name,
        "x": x,
        "y": y,
        "vx": random.uniform(-0.1, 0.1),  # 속도 벡터
        "vy": random.uniform(-0.1, 0.1),
        "safe": random.choice([True, False])
    })

# 이동 시뮬레이션
try:
    while True:
        updated = []
        for w in workers:
            # 새 위치 계산
            new_x = w["x"] + w["vx"]
            new_y = w["y"] + w["vy"]

            # 만약 벽 밖이면 반사
            if not point_in_polygon(new_x, new_y, contour):
                w["vx"] *= -1
                w["vy"] *= -1
                new_x = w["x"] + w["vx"]
                new_y = w["y"] + w["vy"]

            # 상태 갱신
            w["x"] = new_x
            w["y"] = new_y

            # 안전모 착용 상태는 10% 확률로 변경
            if random.random() < 0.1:
                w["safe"] = not w["safe"]

            updated.append({
                "name": w["name"],
                "x": round(w["x"], 2),
                "y": round(w["y"], 2),
                "safe": w["safe"]
            })

        # MQTT 발행
        client.publish(TOPIC_WORKER, json.dumps(updated))
        print("📡 작업자 위치:", updated)

        time.sleep(1.0)

except KeyboardInterrupt:
    client.disconnect()
    print("❌ 시뮬레이터 종료")
