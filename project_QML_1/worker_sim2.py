import json, random, time, math
import paho.mqtt.client as mqtt

# MQTT 설정
BROKER = "broker.hivemq.com"
TOPIC_WORKER = "/worker/data"

# 랜덤 상태변경 기능 On/Off
RANDOM_STATE_CHANGE = False

# ============================
# 월드 사각형 좌표
# ============================
world_poly = [
    {"x": 5.46, "y": 3.50},
    {"x": 2.62, "y": 1.37},
    {"x": -9.30, "y": -1.39},
    {"x": -6.52, "y": 0.74},
]

# ============================
# 점이 다각형 내부인지 판정
# ============================
def point_in_polygon(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]["x"], poly[i]["y"]
        x2, y2 = poly[(i + 1) % n]["x"], poly[(i + 1) % n]["y"]

        if ((y1 > y) != (y2 > y)):
            x_intersect = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-9) + x1
            if x < x_intersect:
                inside = not inside
    return inside

# ============================
# 작업자 초기 설정 + 목표좌표 추가
# ============================
workers = [
    {
        "name": "작업자A", "track_id": 12,
        "x": -1.14, "y": 1.67,
        "helmet": True, "vest": True,
        "vx": 0.0, "vy": 0.0,
        "target_x": 1.33, "target_y": 1.88,   # ⭐ 목표 위치
        "speed": 0.25                        # 이동 속도(m/step)
    },
    {
        "name": "작업자B", "track_id": 7,
        "x": -1.65, "y": 1.02,
        "helmet": False, "vest": True,
        "vx": 0.0, "vy": 0.0,
        "target_x": -1.65, "target_y": 1.02,
        "speed": 0.02
    },

    {
        "name": "신원미상", "track_id": 21,
        "x": -3.42, "y": 0.04,
        "helmet": True, "vest": False,
        "vx": 0.0, "vy": 0.0,
        "target_x": -3.42, "target_y": -3.42,
        "speed": 0.025
    },
    
]

# 안전 상태 자동 계산
for w in workers:
    w["safe"] = w["helmet"] and w["vest"]

# MQTT 연결
client = mqtt.Client()
client.connect(BROKER, 1883, 60)
print("✅ MQTT 연결 성공 (Worker Simulation with Target Movement)")

# ============================
# 시뮬레이션 루프
# ============================
try:
    while True:
        timestamp_ns = int(time.time() * 1e9)
        updated = []

        for w in workers:
            # ============================
            # ★ 목표 지점으로 이동하는 로직
            # ============================
            dx = w["target_x"] - w["x"]
            dy = w["target_y"] - w["y"]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > 0.5:  # 목표와 5cm 이상 → 계속 이동
                w["vx"] = (dx / dist) * w["speed"]
                w["vy"] = (dy / dist) * w["speed"]
            else:
                # 도착 → 정지
                w["vx"] = 0
                w["vy"] = 0

            new_x = w["x"] + w["vx"]
            new_y = w["y"] + w["vy"]

            # 벽 충돌 반사
            if not point_in_polygon(new_x, new_y, world_poly):
                w["vx"] *= -1
                w["vy"] *= -1
                new_x = w["x"] + w["vx"]
                new_y = w["y"] + w["vy"]

            # 위치 업데이트
            w["x"] = new_x
            w["y"] = new_y

            # 랜덤 상태 변경 (옵션)
            if RANDOM_STATE_CHANGE:
                if random.random() < 0.1: w["helmet"] = not w["helmet"]
                if random.random() < 0.1: w["vest"] = not w["vest"]

            w["safe"] = w["helmet"] and w["vest"]

            if w["name"] == "신원미상":
                continue  # MQTT로 보내지 않음

            updated.append({
                "name": w["name"],
                "track_id": w["track_id"],
                "x": round(w["x"], 2),
                "y": round(w["y"], 2),
                "helmet": w["helmet"],
                "vest": w["vest"],
                "safe": w["safe"],
                "stamp": timestamp_ns,
            })

        client.publish(TOPIC_WORKER, json.dumps(updated))
        print("📡 Worker:", updated)

        time.sleep(1)

except KeyboardInterrupt:
    client.disconnect()
    print("❌ 시뮬레이터 종료")
