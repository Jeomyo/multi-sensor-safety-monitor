#!/usr/bin/env python3
# ==============================================================================
# SCRIPT: report_generator.py  (Admin + User Wellness Dual Report)
#
# ROLE:
#   - PROCESSED_JOBS_DIR 에 쌓인 incident JSON을 읽어
#     1) 관리자 분석 리포트 (Admin Report)
#     2) 사용자 건강관리 리포트 (User Wellness Report)
#     두 종류의 txt 파일을 각각 생성한다.
#
#   - 생성된 리포트는 daily_reports/ 에 저장된다.
#   - 처리한 JSON은 archived_jobs/ 로 이동한다.
#   - 모든 작업이 정상 종료되면 MQTT /system/report/job 에 "done" 발행.
# ==============================================================================

import os
import json
import sys
from datetime import datetime

import ollama
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

# --- Configuration ---------------------------------------------------------------

load_dotenv()

OLLAMA_MODEL = os.getenv("OLLAMA_MODEL", "gemma3:1b-it-qat")

PROCESSED_JOBS_DIR = "/home/pi/project/hailo-rpi5-examples/smart-fall-detection/processed_jobs"
REPORTS_DIR = "/home/pi/project/hailo-rpi5-examples/smart-fall-detection/daily_reports"
ARCHIVE_DIR = "/home/pi/project/hailo-rpi5-examples/smart-fall-detection/archived_jobs"

MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC_REPORT_JOB = os.getenv("MQTT_TOPIC_REPORT_JOB", "/system/report/job")
MQTT_CLIENT_ID = os.getenv("MQTT_REPORT_CLIENT_ID", "report_generator_notifier")


# --- Check Ollama ----------------------------------------------------------------

def check_ollama_service() -> bool:
    print("--- Ollama 서비스 상태 점검 ---")
    try:
        ollama.list()
        print("✅ Ollama 서비스 정상 동작.")
        return True
    except Exception:
        print("❌ Ollama 연결 실패.")
        return False


# --- MQTT helper -----------------------------------------------------------------

def send_mqtt_report_done():
    print(f"[MQTT] /system/report/job → done 발행")
    try:
        client = mqtt.Client(client_id=MQTT_CLIENT_ID)
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        info = client.publish(MQTT_TOPIC_REPORT_JOB, "done", qos=1)
        info.wait_for_publish(timeout=5)
        client.disconnect()
        print("[MQTT] 보고 완료 메시지 발행 성공")
    except Exception as e:
        print(f"[MQTT ERROR] {e}")


# --- Incident formatting ----------------------------------------------------------

def format_incident_block(idx: int, job: dict) -> str:
    event_type = job.get("event_type", "unknown")
    trigger = job.get("trigger", "unknown")
    created_at = job.get("created_at", "N/A")
    snapshot_path = job.get("snapshot_path")

    meta = job.get("meta", {}) or {}
    algo = meta.get("algo_state", meta)

    alg_event = algo.get("event")
    reason = algo.get("reason")
    status = algo.get("status")
    frame = algo.get("frame")
    ts_meta = algo.get("timestamp")
    velocity = algo.get("velocity")
    hip_y = algo.get("hip_y")
    spread = algo.get("smoothed_vertical_spread")
    is_horz = algo.get("is_horizontal")
    h_dwell = algo.get("h_dwell_frames")
    prolonged = algo.get("prolonged_frames")
    recovery = algo.get("recovery_frames")

    if event_type == "recovered":
        reason = "recovered"

    lines = []
    lines.append(f"--- Incident {idx} ---")
    lines.append(f"- Event type     : {event_type}")
    lines.append(f"- Trigger        : {trigger}")
    lines.append(f"- Created at     : {created_at}")
    lines.append(f"- Snapshot       : {'Yes' if snapshot_path else 'No'}")

    if alg_event: lines.append(f"- Algo event     : {alg_event}")
    if reason:    lines.append(f"- Reason         : {reason}")
    if frame:     lines.append(f"- Frame index    : {frame}")
    if ts_meta:   lines.append(f"- Algo timestamp : {ts_meta}")
    if velocity is not None: lines.append(f"- Velocity       : {velocity}")
    if hip_y is not None:    lines.append(f"- Hip Y          : {hip_y}")
    if spread is not None:   lines.append(f"- Spread(EMA)    : {spread}")
    if is_horz is not None:  lines.append(f"- Is horizontal  : {is_horz}")
    if status:               lines.append(f"- Status         : {status}")
    if h_dwell is not None:  lines.append(f"- H dwell frames : {h_dwell}")
    if prolonged is not None:lines.append(f"- Prolonged frms : {prolonged}")
    if recovery is not None: lines.append(f"- Recovery frms  : {recovery}")

    return "\n".join(lines)


# --- LLM Report Generation --------------------------------------------------------

def make_admin_report(stats: str, details: str) -> str:
    """관리자(전문가) 분석용 리포트."""
    prompt = f"""
다음은 하루 동안의 fall, manual, recovered 이벤트 데이터이다.

요구사항:
- 한국어 전문 분석 리포트
- 안전관리자/의료진을 위한 설명
- fall → recovered 흐름 등 패턴 분석
- sudden_fall / prolonged_horizontal / recovered 원인 요약
- 행동 권고 포함

[DAILY STATS]
{stats}

[INCIDENT DETAILS]
{details}

위 데이터를 기반으로 전문적인 관리자용 분석 리포트를 작성하라.
"""
    res = ollama.chat(model=OLLAMA_MODEL, messages=[{"role":"user","content":prompt}])
    return res["message"]["content"].strip()


def make_user_wellness_report(stats: str, details: str) -> str:
    """사용자 건강관리 리포트 (일반인용)."""
    prompt = f"""
다음은 하루 동안 한 사람에게 발생한 이벤트 데이터이다.

목적:
- 일반 사용자가 쉽게 이해할 수 있는 '건강관리 리포트' 작성
- 넘어짐, 회복 여부 등을 바탕으로 오늘의 신체 상태를 설명
- 숫자 나열 금지, 쉬운 한국어
- 심리적 압박 없는 톤
- 일상 실천 팁 포함 (자세, 휴식, 수분, 주변 환경 정리 등)
- 의료 경고처럼 쓰지 말 것

[DAILY STATS]
{stats}

[INCIDENT DETAILS]
{details}

이 데이터를 기반으로 사용자가 스스로 이해하기 쉬운 '건강관리 리포트'를 한국어로 작성하라.
"""
    res = ollama.chat(model=OLLAMA_MODEL, messages=[{"role":"user","content":prompt}])
    return res["message"]["content"].strip()


# --- Main -------------------------------------------------------------------------

if __name__ == "__main__":
    print("--- Dual Report Generator (Admin + User) ---")

    if not check_ollama_service():
        sys.exit(1)

    os.makedirs(PROCESSED_JOBS_DIR, exist_ok=True)
    os.makedirs(REPORTS_DIR, exist_ok=True)
    os.makedirs(ARCHIVE_DIR, exist_ok=True)

    try:
        job_files = [f for f in os.listdir(PROCESSED_JOBS_DIR) if f.endswith(".json")]
        if not job_files:
            print("새로운 incident 없음. 종료.")
            sys.exit(0)

        incidents = []
        for filename in job_files:
            path = os.path.join(PROCESSED_JOBS_DIR, filename)
            try:
                with open(path, "r", encoding="utf-8") as f:
                    incidents.append(json.load(f))
            except:
                print(f"ERROR: {filename} 읽기 불가")

        # Stats
        total = len(incidents)
        fall_count = sum(1 for d in incidents if d.get("event_type") == "fall")
        recovered_count = sum(1 for d in incidents if d.get("event_type") == "recovered")

        summary_stats = (
            f"- Total incidents : {total}\n"
            f"- Fall events     : {fall_count}\n"
            f"- Recovery events : {recovered_count}\n"
        )

        # Details
        blocks = []
        for i, data in enumerate(incidents, start=1):
            blocks.append(format_incident_block(i, data))
        incident_details = "\n\n".join(blocks)

        # --- generate admin report ---
        admin_text = make_admin_report(summary_stats, incident_details)
        admin_name = f"admin_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(os.path.join(REPORTS_DIR, admin_name), "w", encoding="utf-8") as f:
            f.write(admin_text)
        print(f"[OK] 관리자 리포트 생성 → {admin_name}")

        # --- generate user wellness report ---
        wellness_text = make_user_wellness_report(summary_stats, incident_details)
        wellness_name = f"wellness_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(os.path.join(REPORTS_DIR, wellness_name), "w", encoding="utf-8") as f:
            f.write(wellness_text)
        print(f"[OK] 사용자 건강관리 리포트 생성 → {wellness_name}")

        # archive JSONs
        for filename in job_files:
            os.rename(
                os.path.join(PROCESSED_JOBS_DIR, filename),
                os.path.join(ARCHIVE_DIR, filename)
            )

        # MQTT notify
        send_mqtt_report_done()

    except Exception as e:
        print(f"Unexpected ERROR: {e}")
        sys.exit(1)