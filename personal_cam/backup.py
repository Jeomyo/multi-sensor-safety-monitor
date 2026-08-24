#!/usr/bin/env python3
# ==============================================================================
# SCRIPT: backup.py
# ROLE : 하루 리포트 & 사건 로그 백업 + 로컬 큐 정리 + MQTT 완료 알림
#
# 기능 요약:
# 1) 관리자용 백업:
#    - archived_jobs 안의 JSON
#    - daily_reports 안의 admin_report_*.txt
#    - fall_snapshots 안의 이미지
#    → 하나의 tar.gz 로 묶어서
#    → REMOTE_HOST:/REMOTE_DIR 로 전송(scp)
#
# 2) 사용자용 리포트:
#    - daily_reports 안의 wellness_report_*.txt 중 가장 최신 파일 1개를
#    - SMTP 로 사용자의 이메일로 전송
#
# 3) scp 전송이 "성공"한 경우에만:
#    - daily_reports
#    - analysis_queue
#    - archived_jobs
#    - processed_jobs
#    - fall_snapshots
#    디렉터리 내부 파일들을 모두 삭제
#
# 4) 모든 작업이 정상적으로 끝나면
#    - MQTT topic: /system/backup
#    - payload: "done"
#    를 로컬 브로커로 퍼블리시
# ==============================================================================

import os
import sys
import tarfile
import subprocess
from datetime import datetime
import smtplib

from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.header import Header



import paho.mqtt.client as mqtt  # MQTT 알림용
from dotenv import load_dotenv

# ----------------------------------------------------------------------
# 환경 변수 로드 (.env 사용)
# ----------------------------------------------------------------------
load_dotenv()

# ----------------------------------------------------------------------
# 경로 설정 (환경에 맞게 필요하면 수정)
# ----------------------------------------------------------------------
# 프로젝트 기반 디렉터리 (smart-fall-detection 기준)
BASE_DIR = "/home/pi/project/hailo-rpi5-examples/smart-fall-detection"

ARCHIVED_JOBS_DIR   = os.path.join(BASE_DIR, "archived_jobs")
DAILY_REPORTS_DIR   = os.path.join(BASE_DIR, "daily_reports")
ANALYSIS_QUEUE_DIR  = os.path.join(BASE_DIR, "analysis_queue")
PROCESSED_JOBS_DIR  = os.path.join(BASE_DIR, "processed_jobs")
FALL_SNAPSHOTS_DIR  = os.path.join(BASE_DIR, "fall_snapshots")

# 백업 결과 tar.gz 를 잠깐 저장해둘 위치 (로컬)
LOCAL_BACKUP_DIR    = os.path.join(BASE_DIR, "backup_temp")
os.makedirs(LOCAL_BACKUP_DIR, exist_ok=True)

# 리모트 서버 정보
REMOTE_HOST = os.getenv("BACKUP_REMOTE_HOST", "192.168.0.59")
REMOTE_USER = os.getenv("BACKUP_REMOTE_USER", "sunmaan")
REMOTE_DIR  = os.getenv("BACKUP_REMOTE_DIR", "/home/sunmaan/back_up_logs")

# MQTT 설정 (로컬 브로커 기준)
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT   = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC_BACKUP = os.getenv("MQTT_TOPIC_BACKUP", "/system/backup")
MQTT_PAYLOAD_DONE = "done"

# SMTP / 이메일 설정 (사용자용 리포트 전송)
SMTP_SERVER   = os.getenv("SMTP_SERVER", "smtp.gmail.com")
SMTP_PORT     = int(os.getenv("SMTP_PORT", "587"))
SMTP_USER     = os.getenv("SMTP_USER")        # 발신 계정
SMTP_PASSWORD = os.getenv("SMTP_PASSWORD")    # 앱 비밀번호 등
SMTP_TO       = os.getenv("SMTP_TO")          # 수신 이메일
SMTP_SUBJECT  = os.getenv("SMTP_SUBJECT", "Watchful 건강 관리 리포트")


# ----------------------------------------------------------------------
# 이메일 전송 (사용자용 wellness_report)
# ----------------------------------------------------------------------
def send_wellness_report_email():
    """
    daily_reports 디렉터리에서 wellness_report_*.txt 중
    가장 최신 파일 하나를 찾아서 SMTP로 전송한다.
    실패하더라도 백업 전체 플로우는 계속 진행된다.
    """
    try:
        if not (SMTP_USER and SMTP_PASSWORD and SMTP_TO):
            print("[EMAIL] SMTP 환경변수(SMTP_USER/SMTP_PASSWORD/SMTP_TO)가 설정되지 않아 이메일 전송을 건너뜀.")
            return False

        if not os.path.isdir(DAILY_REPORTS_DIR):
            print("[EMAIL] daily_reports 디렉터리가 존재하지 않음.")
            return False

        # wellness_report_* 만 필터링
        candidates = []
        for name in os.listdir(DAILY_REPORTS_DIR):
            if name.startswith("wellness_report_") and name.endswith(".txt"):
                full_path = os.path.join(DAILY_REPORTS_DIR, name)
                candidates.append(full_path)

        if not candidates:
            print("[EMAIL] 보낼 wellness_report_*.txt 가 없음. 이메일 전송 스킵.")
            return False

        # 가장 최근 파일 선택
        latest_file = max(candidates, key=os.path.getmtime)
        print(f"[EMAIL] Sending wellness report: {latest_file}")

        with open(latest_file, "r", encoding="utf-8") as f:
            content = f.read()

        msg = MIMEMultipart()
        msg["From"] = SMTP_USER
        msg["To"] = SMTP_TO
        msg["Subject"] = str(Header(SMTP_SUBJECT or "Watchful 건강 관리 리포트", "utf-8"))
        msg.attach(MIMEText(content, "plain", "utf-8"))


        server = smtplib.SMTP(SMTP_SERVER, SMTP_PORT)
        server.starttls()
        server.login(SMTP_USER, SMTP_PASSWORD)
        server.sendmail(SMTP_USER, [SMTP_TO], msg.as_string())
        server.quit()

        print("[EMAIL] 사용자 wellness 리포트 이메일 전송 완료.")
        return True

    except Exception as e:
        print(f"[EMAIL] 이메일 전송 실패: {e}")
        return False


# ----------------------------------------------------------------------
# 백업용 파일 수집
# ----------------------------------------------------------------------
def collect_files_for_backup():
    """
    tar 로 묶을 파일 목록 수집:
      - archived_jobs 안의 모든 .json
      - daily_reports 안의 admin_report_*.txt
      - fall_snapshots 안의 모든 파일(이미지 등)
    """
    archived_jsons = []
    admin_reports  = []
    snapshots      = []

    # archived_jobs/*.json
    if os.path.isdir(ARCHIVED_JOBS_DIR):
        for root, dirs, files in os.walk(ARCHIVED_JOBS_DIR):
            for name in files:
                if name.endswith(".json"):
                    full_path = os.path.join(root, name)
                    archived_jsons.append(full_path)

    # daily_reports/admin_report_*.txt
    if os.path.isdir(DAILY_REPORTS_DIR):
        for root, dirs, files in os.walk(DAILY_REPORTS_DIR):
            for name in files:
                if name.startswith("admin_report_") and name.endswith(".txt"):
                    full_path = os.path.join(root, name)
                    admin_reports.append(full_path)

    # fall_snapshots/* (모든 파일)
    if os.path.isdir(FALL_SNAPSHOTS_DIR):
        for root, dirs, files in os.walk(FALL_SNAPSHOTS_DIR):
            for name in files:
                full_path = os.path.join(root, name)
                snapshots.append(full_path)

    return archived_jsons, admin_reports, snapshots


def create_tar_archive(archived_jsons, admin_reports, snapshots):
    """
    수집된 파일들을 하나의 tar.gz 로 묶는다.
    tar 안에는 BASE_DIR 기준 상대경로 형태로 저장한다.
    """
    if not archived_jsons and not admin_reports and not snapshots:
        print("[BACKUP] 묶을 파일이 없어서 tar 생성 스킵.")
        return None

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tar_name = f"watchful_backup_{timestamp}.tar.gz"
    tar_path = os.path.join(LOCAL_BACKUP_DIR, tar_name)

    print(f"[BACKUP] Creating tar archive: {tar_path}")
    with tarfile.open(tar_path, "w:gz") as tar:
        # archived_jobs 의 json들
        for path in archived_jsons:
            arcname = os.path.relpath(path, BASE_DIR)
            tar.add(path, arcname=arcname)
            print(f"  - added (archived_jobs): {arcname}")

        # daily_reports 의 admin_report들
        for path in admin_reports:
            arcname = os.path.relpath(path, BASE_DIR)
            tar.add(path, arcname=arcname)
            print(f"  - added (admin_reports): {arcname}")

        # fall_snapshots 의 이미지들
        for path in snapshots:
            arcname = os.path.relpath(path, BASE_DIR)
            tar.add(path, arcname=arcname)
            print(f"  - added (fall_snapshots): {arcname}")

    return tar_path


def send_to_remote(tar_path):
    """
    scp 를 이용해 tar.gz 파일을 리모트 서버로 전송한다.
    성공하면 True, 실패하면 False.
    """
    if tar_path is None:
        print("[BACKUP] tar 파일이 없으므로 전송할 것이 없음.")
        return False

    remote_target = f"{REMOTE_USER}@{REMOTE_HOST}:{REMOTE_DIR}"
    print(f"[BACKUP] Sending {tar_path} -> {remote_target}")

    try:
        result = subprocess.run(
            ["scp", tar_path, remote_target],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        if result.returncode != 0:
            print("[ERROR] scp 전송 실패")
            print("  stdout:", result.stdout.strip())
            print("  stderr:", result.stderr.strip())
            return False

        print("[BACKUP] scp 전송 성공")
        return True

    except Exception as e:
        print(f"[ERROR] scp 도중 예외 발생: {e}")
        return False


def clear_directory(path):
    """
    주어진 디렉터리 내부의 파일/디렉터리를 모두 삭제.
    디렉터리 자체는 남겨둔다.
    """
    if not os.path.isdir(path):
        return

    print(f"[CLEANUP] Clearing directory: {path}")
    for name in os.listdir(path):
        full_path = os.path.join(path, name)
        try:
            if os.path.isfile(full_path) or os.path.islink(full_path):
                os.remove(full_path)
            elif os.path.isdir(full_path):
                import shutil
                shutil.rmtree(full_path)
        except Exception as e:
            print(f"  - Failed to remove {full_path}: {e}")


def cleanup_local_dirs():
    """
    전송 성공 후, 다음 디렉토리들의 내용 모두 삭제:
      - daily_reports
      - analysis_queue
      - archived_jobs
      - processed_jobs
      - fall_snapshots
    """
    clear_directory(DAILY_REPORTS_DIR)
    clear_directory(ANALYSIS_QUEUE_DIR)
    clear_directory(ARCHIVED_JOBS_DIR)
    clear_directory(PROCESSED_JOBS_DIR)
    clear_directory(FALL_SNAPSHOTS_DIR)


def publish_backup_done():
    """
    백업 작업이 정상적으로 끝난 뒤,
    /system/backup 토픽에 "done" 메시지 퍼블리시.
    실패해도 백업 자체에는 영향 안 주도록 예외는 로그만 찍고 무시.
    """
    print(f"[MQTT] Publishing backup done → {MQTT_BROKER}:{MQTT_PORT} topic={MQTT_TOPIC_BACKUP}")
    try:
        client = mqtt.Client(client_id="backup_notifier")
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
        client.loop_start()

        result = client.publish(MQTT_TOPIC_BACKUP, MQTT_PAYLOAD_DONE, qos=1)
        result.wait_for_publish()

        client.loop_stop()
        client.disconnect()
        print("[MQTT] Backup done message published.")
    except Exception as e:
        print(f"[MQTT] Failed to publish backup done message: {e}")


# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def main():
    print("--- WATCHFUL BACKUP SCRIPT START ---")
    print(f"BASE_DIR = {BASE_DIR}")

    # 1) 사용자용 wellness_report 이메일 전송 시도
    send_wellness_report_email()

    # 2) 관리자용 백업 대상 수집
    archived_jsons, admin_reports, snapshots = collect_files_for_backup()

    print(f"[BACKUP] archived_jobs json 개수: {len(archived_jsons)}")
    print(f"[BACKUP] admin_report 파일 개수: {len(admin_reports)}")
    print(f"[BACKUP] fall_snapshots 파일 개수: {len(snapshots)}")

    if not archived_jsons and not admin_reports and not snapshots:
        print("[BACKUP] 백업 대상 파일이 전혀 없음. 종료.")
        return 0

    tar_path = create_tar_archive(archived_jsons, admin_reports, snapshots)

    # tar 생성 실패하면 바로 종료
    if tar_path is None or not os.path.exists(tar_path):
        print("[BACKUP] tar 파일 생성 실패. 종료.")
        return 1

    # 3) 리모트 전송
    success = send_to_remote(tar_path)

    if not success:
        print("[BACKUP] 전송 실패. 로컬 데이터는 유지합니다.")
        return 1

    # 4) 전송 성공 시 로컬 큐/리포트/아카이브/스냅샷 정리
    cleanup_local_dirs()

    # 5) 여기까지 왔으면 전체 플로우 성공 → MQTT로 done 알림
    publish_backup_done()

    # tar 자체는 남겨둘지 말지는 선택.
    # 삭제하고 싶으면 아래 주석 해제.
    try:
     os.remove(tar_path)
    except Exception as e:
     print(f"[WARN] 로컬 tar 삭제 실패: {e}")

    print("[BACKUP] 모든 작업 완료.")
    return 0


if __name__ == "__main__":
    sys.exit(main())