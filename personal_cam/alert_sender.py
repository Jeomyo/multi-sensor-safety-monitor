# ==============================================================================
# SCRIPT: alert_sender.py
# ROLE: The Messenger (Absolute Path Fixed)
# ==============================================================================

import os
import json
import time
import requests
from dotenv import load_dotenv

# 프로젝트 루트 .env 로드 (절대 경로)
load_dotenv(dotenv_path="/home/pi/project/hailo-rpi5-examples/.env")

# --- Configuration ---
# 1. .env 파일 로드 (현재 스크립트와 같은 위치에 있다고 가정)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ENV_PATH = os.path.join(BASE_DIR, ".env")
load_dotenv(ENV_PATH)

TELEGRAM_BOT_TOKEN = os.getenv("TELEGRAM_BOT_TOKEN")
TELEGRAM_CHAT_ID = os.getenv("TELEGRAM_CHAT_ID")

# 2. 절대 경로 고정
ANALYSIS_QUEUE_DIR = (
    "/home/pi/project/hailo-rpi5-examples/smart-fall-detection/analysis_queue"
)
PROCESSED_JOBS_DIR = (
    "/home/pi/project/hailo-rpi5-examples/smart-fall-detection/processed_jobs"
)

POLLING_INTERVAL_SECONDS = 2


# ==============================================================================
# MESSAGE BUILDER (관리자 친화적 한국어 메시지)
# ==============================================================================
def create_simple_alert_message(job_data: dict) -> str:
    """관리자에게 보내는 알림 메시지 (한국어)."""

    # 기본 정보
    event_type = job_data.get("event_type", "이벤트")
    trigger = job_data.get("trigger", "unknown")
    created_at = job_data.get("created_at", "N/A")
    snapshot_path = job_data.get("snapshot_path")

    meta = job_data.get("meta", {}) or {}
    if isinstance(meta, dict) and "algo_state" in meta:
        algo = meta["algo_state"]
    else:
        algo = meta

    reason = algo.get("reason")
    status = algo.get("status")

    # 한국어 이벤트 이름 매핑
    if event_type == "fall":
        event_kor = "낙상"
    elif event_type == "manual_capture":
        event_kor = "수동 캡처"
    elif event_type == "recovered":
        event_kor = "회복"
    else:
        event_kor = event_type

    # readable reason
    if event_type == "recovered":
        readable_reason = "낙상 후 정상 자세 복귀"
    elif reason == "sudden_fall":
        readable_reason = "갑작스러운 낙상"
    elif reason == "prolonged_horizontal":
        readable_reason = "장시간 수평 자세 유지"
    elif trigger == "manual":
        readable_reason = "수동 촬영 요청"
    else:
        readable_reason = "상세 사유 없음"

    # 메시지 구성
    lines = []
    lines.append(f"🚨 {event_kor} 감지됨")
    lines.append("")
    lines.append("[기본 정보]")
    lines.append(f"- 발생 시각: {created_at}")
    lines.append(f"- 이벤트 유형: {event_kor}")
    lines.append(f"- 판정 이유: {readable_reason}")
    lines.append("")
    lines.append("[상태]")
    lines.append(f"- 현재 상태: {status}")

    if snapshot_path and event_type != "recovered":
        lines.append("")
        lines.append("사진이 첨부되었습니다.")

    return "\n".join(lines)


# ==============================================================================
# TELEGRAM SENDERS
# ==============================================================================
def send_telegram_text_alert(message: str):
    """텍스트 알림 전송."""
    if not all([TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID]):
        print("  - [Error] Telegram credentials missing.")
        return False

    api_url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendMessage"
    payload = {"chat_id": TELEGRAM_CHAT_ID, "text": message}

    try:
        response = requests.post(api_url, data=payload, timeout=10)
        response.raise_for_status()
        print("  - [Success] Telegram text alert sent.")
        return True
    except Exception as e:
        print(f"  - [Fail] Could not send Telegram text: {e}")
        return False


def send_telegram_photo_alert(caption: str, image_path: str):
    """사진 + caption 전송."""
    if not all([TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID]):
        return False

    api_url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendPhoto"

    try:
        with open(image_path, "rb") as photo_file:
            files = {"photo": photo_file}
            payload = {"chat_id": TELEGRAM_CHAT_ID, "caption": caption}

            response = requests.post(api_url, data=payload, files=files, timeout=15)
            response.raise_for_status()

            print("  - [Success] Telegram photo alert sent.")
            return True

    except Exception as e:
        print(f"  - [Fail] Photo send failed ({e}). Sending text instead.")
        send_telegram_text_alert(caption)
        return False


# ==============================================================================
# JOB PROCESSOR
# ==============================================================================
def process_alert_job(job_path: str):
    job_basename = os.path.basename(job_path)
    print(f"\n>>> Found job: {job_basename}")

    try:
        with open(job_path, "r", encoding="utf-8") as f:
            job_data = json.load(f)

        event_type = job_data.get("event_type")
        snapshot_path = job_data.get("snapshot_path")

        # 메시지 생성
        alert_message = create_simple_alert_message(job_data)

        # =========== RECOVERED 이벤트 전용 처리 ===========
        if event_type == "recovered":
            print("  - Sending RECOVERY alert (text only).")
            send_telegram_text_alert(alert_message)

            # 처리 완료 이동
            destination_path = os.path.join(PROCESSED_JOBS_DIR, job_basename)
            os.rename(job_path, destination_path)
            print("  - Recovery job processed and moved.")
            return

        # =========== FALL or MANUAL_CAPTURE ===========
        if snapshot_path and os.path.exists(snapshot_path):
            print(f"  - Sending photo alert: {snapshot_path}")
            send_telegram_photo_alert(alert_message, snapshot_path)
        else:
            print("  - No snapshot available. Sending text alert.")
            send_telegram_text_alert(alert_message)

        # 처리 완료 이동
        destination_path = os.path.join(PROCESSED_JOBS_DIR, job_basename)
        os.rename(job_path, destination_path)
        print("  - Job processed and moved.")

    except Exception as e:
        print(f"  - FATAL ERROR processing {job_basename}: {e}")
        try:
            error_dest = os.path.join(PROCESSED_JOBS_DIR, f"error_{job_basename}")
            os.rename(job_path, error_dest)
        except Exception:
            pass


# ==============================================================================
# MAIN LOOP
# ==============================================================================
if __name__ == "__main__":
    print("--- Real-time Alert Sender v2.3 (Fall + Recovery Aware) ---")

    os.makedirs(ANALYSIS_QUEUE_DIR, exist_ok=True)
    os.makedirs(PROCESSED_JOBS_DIR, exist_ok=True)

    print(f"Target Queue Directory: {ANALYSIS_QUEUE_DIR}")

    if not all([TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID]):
        print("\n[CRITICAL] Telegram credentials are NOT set in .env file!")
    else:
        print("Validating Telegram connection...")
        send_telegram_text_alert("✅ System Online: Watching job directory.")

    print("\nWatching for jobs... Press Ctrl+C to exit.")

    while True:
        try:
            job_files = [
                f for f in os.listdir(ANALYSIS_QUEUE_DIR) if f.endswith(".json")
            ]
            if job_files:
                job_files.sort()
                job_to_process = os.path.join(ANALYSIS_QUEUE_DIR, job_files[0])
                process_alert_job(job_to_process)
            else:
                time.sleep(POLLING_INTERVAL_SECONDS)

        except KeyboardInterrupt:
            break

        except Exception as e:
            print(f"Loop Error: {e}")
            time.sleep(2)
