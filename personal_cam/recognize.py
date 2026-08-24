import argparse
import sys
import time
import os
import subprocess
import tempfile

import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt
from gtts import gTTS

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Global variables to calculate FPS
COUNTER, FPS = 0, 0
START_TIME = time.time()

# ----- MQTT & TTS 설정 -----
MQTT_KEEPALIVE = 60

# 로컬 브로커 (라즈베리파이 내부 통신용)
MQTT_LOCAL_HOST = "localhost"
MQTT_LOCAL_PORT = 1883

# 원격 브로커 (예: PC 서버 / 다른 라즈베리파이)
MQTT_REMOTE_HOST = os.getenv("MQTT_REMOTE_HOST", "192.168.0.59")
MQTT_REMOTE_PORT = int(os.getenv("MQTT_REMOTE_PORT", "1883"))


def tts_say(text: str) -> None:
    # 디버깅용 출력
    print(f"[TTS] {text}")

    tmp_path = None

    try:
        # 1) 임시 mp3 파일 생성
        with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as fp:
            tmp_path = fp.name

        # 2) gTTS로 한국어 음성 파일 생성
        tts = gTTS(text=text, lang="ko")
        tts.save(tmp_path)

        # 3) mpg123로 재생 (-q는 조용히)
        subprocess.run(
            ["mpg123", "-q", tmp_path],
            check=False
        )

    except Exception as e:
        print("[TTS ERROR]", e)

    finally:
        # 4) 임시 파일 정리
        if tmp_path is not None and os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception:
                pass


def create_mqtt_client(host: str, port: int, client_id: str):
    try:
        client = mqtt.Client(client_id=client_id)
        client.connect(host, port, MQTT_KEEPALIVE)
        client.loop_start()
        print(f"[MQTT] Connected to {host}:{port} ({client_id})")
        return client
    except Exception as e:
        print(f"[MQTT] 연결 실패 ({host}:{port}, {client_id}):", e)
        return None


# 로컬 / 원격 클라이언트 두 개 생성
MQTT_LOCAL_CLIENT = create_mqtt_client(MQTT_LOCAL_HOST, MQTT_LOCAL_PORT, "gesture_local")
MQTT_REMOTE_CLIENT = create_mqtt_client(MQTT_REMOTE_HOST, MQTT_REMOTE_PORT, "gesture_remote")


def mqtt_publish(topic: str, payload: str) -> None:
    def _publish_one(client, label: str):
        if client is None:
            print(f"[MQTT:{label}] 클라이언트 없음. topic={topic}, payload={payload}")
            return
        result = client.publish(topic, payload)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"[MQTT:{label}] 발행 실패 rc={result.rc} topic={topic}, payload={payload}")

    # 로컬 브로커로 발행
    _publish_one(MQTT_LOCAL_CLIENT, "local")
    # 원격 브로커로도 발행
    _publish_one(MQTT_REMOTE_CLIENT, "remote")


def run(model: str, num_hands: int,
        min_hand_detection_confidence: float,
        min_hand_presence_confidence: float, min_tracking_confidence: float,
        camera_id: int, width: int, height: int) -> None:
  """Continuously run inference on images acquired from the camera.

  Args:
      model: Name of the gesture recognition model bundle.
      num_hands: Max number of hands can be detected by the recognizer.
      min_hand_detection_confidence: The minimum confidence score for hand
        detection to be considered successful.
      min_hand_presence_confidence: The minimum confidence score of hand
        presence score in the hand landmark detection.
      min_tracking_confidence: The minimum confidence score for the hand
        tracking to be considered successful.
      camera_id: The camera id to be passed to OpenCV.
      width: The width of the frame captured from the camera.
      height: The height of the frame captured from the camera.
  """

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 50  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 0)  # black
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  # Label box parameters
  label_text_color = (255, 255, 255)  # white
  label_font_size = 1
  label_thickness = 2

  recognition_frame = None
  recognition_result_list = []

  # ----- 제스처 기반 모드 선택 상태 변수 -----
  YES_GESTURE = "Thumb_Up"
  NO_GESTURE = "Thumb_Down"

  OPEN_PALM = "Open_Palm"
  RAISED_FIST = "Closed_Fist"
  INDEX_POINTING_UP = "Pointing_Up"
  VICTORY_HAND = "Victory"

  pending_action = None  # None or "go_work", "leave_work", "call_manager", "alert_error"
  last_gesture = None
  last_gesture_time = 0.0
  GESTURE_DEBOUNCE_SECONDS = 0.8

  # ----- 시간대 인사 멘트 -----
  def get_time_greeting() -> str:
      hour = time.localtime().tm_hour
      if hour < 12:
          return "좋은 아침입니다."
      elif hour < 18:
          return "좋은 오후입니다."
      else:
          return "좋은 저녁입니다."

  def reset_to_mode_select():
      nonlocal pending_action
      pending_action = None
      tts_say("알겠습니다. 다시 모드를 선택해 주세요. 맞지 않다면 엄지손가락을 내린 제스처를 해 주세요.")

  def process_yes():
      nonlocal pending_action
      now_greeting = get_time_greeting()

      if pending_action == "go_work":
          # 시간대 인사 + 출근 처리 멘트
          tts_say(f"{now_greeting} 출근으로 기록했습니다.")
          mqtt_publish("/system/mode/gotowork", "1")
          pending_action = None

      elif pending_action == "leave_work":
          # 퇴근 멘트는 시간대와 무관하게 하루 정리 느낌으로
          tts_say("오늘도 수고 많으셨습니다. 퇴근으로 기록했습니다.")
          mqtt_publish("/system/mode/leavework", "1")
          pending_action = None

      elif pending_action == "call_manager":
          tts_say("관리자를 호출하겠습니다.")
          mqtt_publish("/system/msg", "call")
          pending_action = None

      elif pending_action == "alert_error":
          tts_say("관리자에게 장애 발생을 알리겠습니다.")
          mqtt_publish("/system/msg", "Problem Occured")
          pending_action = None

  def handle_gesture(category_name: str):
      nonlocal pending_action, last_gesture, last_gesture_time

      # 인식 안 된 상태나 의미 없는 값은 무시
      if not category_name or category_name == "None":
          return

      now = time.time()

      # 디바운스: 같은 제스처가 너무 짧은 시간에 반복되면 무시
      if category_name == last_gesture and (now - last_gesture_time) < GESTURE_DEBOUNCE_SECONDS:
          return

      last_gesture = category_name
      last_gesture_time = now

      print(f"[DEBUG] 인식된 제스처: {category_name}")

      # -------------------------------
      # 1) 질문 대기 중인 상태 (YES/NO만 허용)
      # -------------------------------
      if pending_action is not None:
          if category_name == YES_GESTURE:
              # YES → 해당 액션 수행
              process_yes()
          elif category_name == NO_GESTURE:
              # NO → 모드 선택으로 복귀
              reset_to_mode_select()
          else:
              # 다른 제스처는 전부 무시
              print("[DEBUG] 질문 대기 중이라 이 제스처는 무시:", category_name)
          return

      # -------------------------------
      # 2) 평상시: 모드 트리거 제스처 처리
      # -------------------------------
      if category_name == OPEN_PALM:
          pending_action = "go_work"
          tts_say("출근하시는 거라면, 엄지손가락을 들어 올려서 맞다고 알려 주세요.")
      elif category_name == RAISED_FIST:
          pending_action = "leave_work"
          tts_say("퇴근하시는 거라면, 엄지손가락을 들어 올려서 맞다고 알려 주세요.")
      elif category_name == INDEX_POINTING_UP:
          pending_action = "call_manager"
          tts_say("관리자를 호출할까요?")
      elif category_name == VICTORY_HAND:
          pending_action = "alert_error"
          tts_say("관리자에게 장애 발생을 알릴까요?")

  def save_result(result: vision.GestureRecognizerResult,
                  unused_output_image: mp.Image, timestamp_ms: int):
      global FPS, COUNTER, START_TIME

      # Calculate the FPS
      if COUNTER % fps_avg_frame_count == 0:
          FPS = fps_avg_frame_count / (time.time() - START_TIME)
          START_TIME = time.time()

      recognition_result_list.append(result)
      COUNTER += 1

  # Initialize the gesture recognizer model
  base_options = python.BaseOptions(model_asset_path=model)
  options = vision.GestureRecognizerOptions(base_options=base_options,
                                          running_mode=vision.RunningMode.LIVE_STREAM,
                                          num_hands=num_hands,
                                          min_hand_detection_confidence=min_hand_detection_confidence,
                                          min_hand_presence_confidence=min_hand_presence_confidence,
                                          min_tracking_confidence=min_tracking_confidence,
                                          result_callback=save_result)
  recognizer = vision.GestureRecognizer.create_from_options(options)

  # Continuously capture images from the camera and run inference
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Unable to read from webcam. Please verify your webcam settings.'
      )

    image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

    # Run gesture recognizer using the model.
    recognizer.recognize_async(mp_image, time.time_ns() // 1_000_000)

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(FPS)
    text_location = (left_margin, row_size)
    current_frame = image
    cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                font_size, text_color, font_thickness, cv2.LINE_AA)

    if recognition_result_list:
      # Draw landmarks and write the text for each hand.
      for hand_index, hand_landmarks in enumerate(
          recognition_result_list[0].hand_landmarks):
        # Calculate the bounding box of the hand
        x_min = min([landmark.x for landmark in hand_landmarks])
        y_min = min([landmark.y for landmark in hand_landmarks])
        y_max = max([landmark.y for landmark in hand_landmarks])

        # Convert normalized coordinates to pixel values
        frame_height, frame_width = current_frame.shape[:2]
        x_min_px = int(x_min * frame_width)
        y_min_px = int(y_min * frame_height)
        y_max_px = int(y_max * frame_height)

        # Get gesture classification results
        if recognition_result_list[0].gestures:
          gesture = recognition_result_list[0].gestures[hand_index]
          category_name = gesture[0].category_name
          score = round(gesture[0].score, 2)
          result_text = f'{category_name} ({score})'

          # 제스처 디버깅 및 모드 선택 로직 처리 (첫 번째 손에 대해서만)
          if hand_index == 0:
              handle_gesture(category_name)

          # Compute text size
          text_size = \
          cv2.getTextSize(result_text, cv2.FONT_HERSHEY_DUPLEX, label_font_size,
                          label_thickness)[0]
          text_width, text_height = text_size

          # Calculate text position (above the hand)
          text_x = x_min_px
          text_y = y_min_px - 10  # Adjust this value as needed

          # Make sure the text is within the frame boundaries
          if text_y < 0:
            text_y = y_max_px + text_height

          # Draw the text
          cv2.putText(current_frame, result_text, (text_x, text_y),
                      cv2.FONT_HERSHEY_DUPLEX, label_font_size,
                      label_text_color, label_thickness, cv2.LINE_AA)

        # Draw hand landmarks on the frame
        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend([
          landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y,
                                          z=landmark.z) for landmark in
          hand_landmarks
        ])
        mp_drawing.draw_landmarks(
          current_frame,
          hand_landmarks_proto,
          mp_hands.HAND_CONNECTIONS,
          mp_drawing_styles.get_default_hand_landmarks_style(),
          mp_drawing_styles.get_default_hand_connections_style())

      recognition_frame = current_frame
      recognition_result_list.clear()

    if recognition_frame is not None:
        cv2.imshow('gesture_recognition', recognition_frame)

    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
        break

  recognizer.close()
  cap.release()
  cv2.destroyAllWindows()


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Name of gesture recognition model.',
      required=False,
      default='gesture_recognizer.task')
  parser.add_argument(
      '--numHands',
      help='Max number of hands that can be detected by the recognizer.',
      required=False,
      default=1)
  parser.add_argument(
      '--minHandDetectionConfidence',
      help='The minimum confidence score for hand detection to be considered '
           'successful.',
      required=False,
      default=0.5)
  parser.add_argument(
      '--minHandPresenceConfidence',
      help='The minimum confidence score of hand presence score in the hand '
           'landmark detection.',
      required=False,
      default=0.5)
  parser.add_argument(
      '--minTrackingConfidence',
      help='The minimum confidence score for the hand tracking to be '
           'considered successful.',
      required=False,
      default=0.5)
  # Finding the camera ID can be very reliant on platform-dependent methods.
  # One common approach is to use the fact that camera IDs are usually indexed sequentially by the OS, starting from 0.
  # Here, we use OpenCV and create a VideoCapture object for each potential ID with 'cap = cv2.VideoCapture(i)'.
  # If 'cap' is None or not 'cap.isOpened()', it indicates the camera ID is not available.
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      default=480)
  args = parser.parse_args()

  run(args.model, int(args.numHands), args.minHandDetectionConfidence,
      args.minHandPresenceConfidence, args.minTrackingConfidence,
      int(args.cameraId), args.frameWidth, args.frameHeight)


if __name__ == '__main__':
  main()
