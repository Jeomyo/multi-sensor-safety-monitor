# ros_async_yolo.py
import os
import cv2
import json
import time
import copy
import queue
import threading
import argparse
import numpy as np
import torch
import torchvision
from ultralytics.utils import ops

from dx_engine import InferenceEngine
from dx_engine import version as dx_version

# ====== ROS2 ======
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Header

# -----------------------
# 기존 유틸/디코딩 함수들 (원본 유지; 필요 최소 수정만)
# -----------------------
PPU_TYPES = ["BBOX", "POSE"]

q = queue.Queue()
callback_lock = threading.Lock()

callback_cnt = 0
g_shutdown = False

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def letter_box(image_src, new_shape=(512, 512), fill_color=(114, 114, 114), format=None):
    src_shape = image_src.shape[:2]  # (H, W)
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / src_shape[0], new_shape[1] / src_shape[1])
    ratio = (r, r)
    new_unpad = (int(round(src_shape[1] * r)), int(round(src_shape[0] * r)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2

    if src_shape[::-1] != new_unpad:
        image_src = cv2.resize(image_src, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    image_new = cv2.copyMakeBorder(image_src, top, bottom, left, right, cv2.BORDER_CONSTANT, value=fill_color)
    if format is not None:
        image_new = cv2.cvtColor(image_new, format)
    return image_new, ratio, (dw, dh)

def onnx_decode(ie_outputs, cpu_onnx_path):
    import onnxruntime as ort
    sess = ort.InferenceSession(cpu_onnx_path)
    input_names = [i.name for i in sess.get_inputs()]
    input_dict = {}
    for i, name in enumerate(input_names):
        input_dict[name] = ie_outputs[i]
    ort_output = sess.run(None, input_dict)
    return ort_output[0][0]

def all_decode(ie_outputs, layer_config, n_classes):
    outputs = []
    outputs.append(ie_outputs[0][..., :(n_classes + 5) * len(layer_config[0]["anchor_width"])])
    outputs.append(ie_outputs[1][..., :(n_classes + 5) * len(layer_config[0]["anchor_width"])])
    outputs.append(ie_outputs[2][..., :(n_classes + 5) * len(layer_config[0]["anchor_width"])])

    decoded_tensor = []
    for i, output in enumerate(outputs):
        for l in range(len(layer_config[i]["anchor_width"])):
            start = l * (n_classes + 5)
            end = start + n_classes + 5
            layer = layer_config[i]
            stride = layer["stride"]
            grid_size = output.shape[2]
            meshgrid_x = np.arange(0, grid_size)
            meshgrid_y = np.arange(0, grid_size)
            grid = np.stack([np.meshgrid(meshgrid_y, meshgrid_x)], axis=-1)[..., 0]
            output[..., start+4:end] = sigmoid(output[..., start+4:end])
            cxcy = output[..., start+0:start+2]
            wh   = output[..., start+2:start+4]
            cxcy[..., 0] = (sigmoid(cxcy[..., 0]) * 2 - 0.5 + grid[0]) * stride
            cxcy[..., 1] = (sigmoid(cxcy[..., 1]) * 2 - 0.5 + grid[1]) * stride
            wh[..., 0]   = ((sigmoid(wh[..., 0]) * 2) ** 2) * layer["anchor_width"][l]
            wh[..., 1]   = ((sigmoid(wh[..., 1]) * 2) ** 2) * layer["anchor_height"][l]
            decoded_tensor.append(output[..., start:end].reshape(-1, n_classes + 5))
    decoded_output = np.concatenate(decoded_tensor, axis=0)
    return decoded_output

def ppu_decode_pose(ie_outputs, layer_config, n_classes):
    ie_output = ie_outputs[0][0]
    num_det = ie_output.shape[0]
    decoded_tensor = []
    for detected_idx in range(num_det):
        tensor = np.zeros((n_classes + 5), dtype=float)
        data = ie_output[detected_idx].tobytes()
        box = np.frombuffer(data[0:16], np.float32)
        gy, gx, anchor, layer = np.frombuffer(data[16:20], np.uint8)
        score = np.frombuffer(data[20:24], np.float32)
        label = 0
        # kpts = np.frombuffer(data[28:232], np.float32)

        if layer >= len(layer_config):  # 안전 가드
            break
        w = layer_config[layer]["anchor_width"][anchor]
        h = layer_config[layer]["anchor_height"][anchor]
        s = layer_config[layer]["stride"]

        grid = np.array([gx, gy], np.float32)
        anchor_wh = np.array([w, h], np.float32)
        xc = (grid - 0.5 + (box[0:2] * 2)) * s
        wh = box[2:4] ** 2 * 4 * anchor_wh
        box = np.concatenate([xc, wh], axis=0)
        tensor[:4] = box
        tensor[4] = score
        tensor[4 + 1 + label] = score
        decoded_tensor.append(tensor)
    if len(decoded_tensor) == 0:
        decoded_tensor = np.zeros((n_classes + 5), dtype=float)
    decoded_output = np.stack(decoded_tensor)
    return decoded_output

def ppu_decode(ie_outputs, layer_config, n_classes):
    ie_output = ie_outputs[0][0]
    num_det = ie_output.shape[0]
    decoded_tensor = []
    for detected_idx in range(num_det):
        tensor = np.zeros((n_classes + 5), dtype=float)
        data = ie_output[detected_idx].tobytes()
        box = np.frombuffer(data[0:16], np.float32)
        gy, gx, anchor, layer = np.frombuffer(data[16:20], np.uint8)
        score = np.frombuffer(data[20:24], np.float32)
        label = np.frombuffer(data[24:28], np.uint32)
        if layer >= len(layer_config):  # 안전 가드
            break
        w = layer_config[layer]["anchor_width"][anchor]
        h = layer_config[layer]["anchor_height"][anchor]
        s = layer_config[layer]["stride"]

        grid = np.array([gx, gy], np.float32)
        anchor_wh = np.array([w, h], np.float32)
        xc = (grid - 0.5 + (box[0:2] * 2)) * s
        wh = box[2:4] ** 2 * 4 * anchor_wh
        box = np.concatenate([xc, wh], axis=0)
        tensor[:4] = box
        tensor[4] = score
        tensor[4 + 1 + label] = score
        decoded_tensor.append(tensor)
    if len(decoded_tensor) == 0:
        decoded_tensor = np.zeros((n_classes + 5), dtype=float)
    decoded_output = np.stack(decoded_tensor)
    return decoded_output


class YoloConfig:
    def __init__(self, model_path, classes, score_threshold, iou_threshold, layers, input_size, output_type, decode_type):
        self.model_path = model_path
        self.classes = classes
        self.score_threshold = score_threshold
        self.iou_threshold = iou_threshold
        self.layers = layers
        self.input_size = (input_size, input_size)
        self.output_type = output_type
        self.decode_type = decode_type
        self.colors = np.random.randint(0, 256, [len(self.classes), 3], np.uint8).tolist()


class AsyncYolo:
    def __init__(self, ie: InferenceEngine, yolo_config: YoloConfig, classes, score_threshold, iou_threshold, layers):
        self.ie = ie
        self.config = yolo_config
        self.classes = classes
        self.score_threshold = score_threshold
        self.iou_threshold = iou_threshold
        self.layers = layers
        input_resolution = np.sqrt(self.ie.get_input_size() / 3)
        self.input_size = (input_resolution, input_resolution)
        self.videomode = False
        self.result_output = queue.Queue()
        # DeepX 콜백 등록 (버전별 명칭 주의)
        if hasattr(self.ie, "RegisterCallBack"):
            self.ie.RegisterCallBack(self.pp_callback)
        else:
            self.ie.register_callback(self.pp_callback)

    def run(self, image):
        self.image = copy.deepcopy(image)
        self.input_image, _, _ = letter_box(self.image, self.config.input_size, fill_color=(114, 114, 114), format=cv2.COLOR_BGR2RGB)
        self.req = self.ie.run_async([self.input_image], self)
        q.put(self.req)
        return self.req

    def set_videomode(self, video_mode: bool):
        self.videomode = video_mode

    def deepcopy(self, outputs):
        self.result_output = copy.deepcopy(outputs)

    @staticmethod
    def pp_callback(ie_outputs, user_args):
        with callback_lock:
            if dx_version.__version__ == '1.1.0':
                value: AsyncYolo = user_args
            else:
                value: AsyncYolo = user_args.value
            value.result_output.put(ie_outputs)
            q.put(value.req)


class PostProcessingRun:
    def __init__(self, config: YoloConfig, output_task_order):
        self.video_mode = False
        self.config = config
        self.inputsize_w = int(self.config.input_size[0])
        self.inputsize_h = int(self.config.input_size[1])
        self._queue = queue.Queue()
        self.task_order = output_task_order

    def run(self, result_output):
        self.result_bbox = self.postprocessing(result_output)
        self._queue.put(self.result_bbox)

    def postprocessing(self, outputs):
        # 디코딩
        if self.config.output_type == "BBOX":
            decoded_tensor = ppu_decode(outputs, self.config.layers, len(self.config.classes))
        elif self.config.output_type == "POSE":
            decoded_tensor = ppu_decode_pose(outputs, self.config.layers, len(self.config.classes))
        elif len(outputs) > 1:
            if self.config.decode_type in ["yolov8", "yolov9"]:
                raise ValueError(
                    f"Decode type '{self.config.decode_type}' requires USE_ORT=ON. "
                    "Please enable ONNX Runtime support to use this decode type."
                )
            decoded_tensor = all_decode(outputs, self.config.layers, len(self.config.classes))
        elif len(outputs) == 1:
            decoded_tensor = outputs[0]
        else:
            raise ValueError(f"[Error] Output Size {len(outputs)} is not supported !!")

        # 후처리
        x = np.squeeze(decoded_tensor)
        if self.config.decode_type in ["yolov8", "yolov9"]:
            x = x.T
            box = ops.xywh2xyxy(x[..., :4])
            conf = np.max(x[..., 4:], axis=-1, keepdims=True)
            j    = np.argmax(x[..., 4:], axis=-1, keepdims=True)
        else:
            x = x[x[..., 4] > self.config.score_threshold]
            box = ops.xywh2xyxy(x[..., :4])
            x[:, 5:] *= x[:, 4:5]
            conf = np.max(x[..., 5:], axis=-1, keepdims=True)
            j    = np.argmax(x[..., 5:], axis=-1, keepdims=True)

        mask = conf.flatten() > self.config.score_threshold
        filtered = np.concatenate((box, conf, j.astype(np.float32)), axis=1)[mask]
        sorted_indices = np.argsort(-filtered[:, 4])
        x = torch.tensor(filtered[sorted_indices])
        # NMS의 세 번째 인자는 IoU threshold여야 함
        x = x[torchvision.ops.nms(x[:, :4], x[:, 4], self.config.iou_threshold)]
        return x

    def draw_on_image(self, input_image):
        image = cv2.cvtColor(input_image, cv2.COLOR_RGB2BGR)  # letterboxed RGB → BGR
        results = self._queue.get().numpy()
        for r in results:
            pt1, pt2, conf, label = r[0:2].astype(int), r[2:4].astype(int), r[4], int(r[5])
            image = cv2.rectangle(image, pt1, pt2, self.config.colors[label], 2)
        self._queue.task_done()
        return image


# -----------------------
# ★ ROS2 어댑터 (입력/출력만 ROS로 교체; run_async는 그대로)
# -----------------------
class RosAsyncRunner(Node):
    def __init__(self, json_config, image_topic: str, pub_topic: str):
        super().__init__('dx_yolo_async_node')

        # QoS: gscam 기본값과 맞춤 (BEST_EFFORT + VOLATILE, depth=1)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(RosImage, image_topic, self.image_cb, qos_sub)
        self.pub = self.create_publisher(RosImage, pub_topic, qos_pub)

        # DeepX + YOLO 준비
        model_path = json_config["model"]["path"]
        classes = json_config["output"]["classes"]
        score_threshold = json_config["model"]["param"]["score_threshold"]
        iou_threshold = json_config["model"]["param"]["iou_threshold"]
        layers = json_config["model"]["param"]["layer"]
        decode_type = json_config["model"]["param"]["decoding_method"]

        self.ie = InferenceEngine(model_path)
        task_order = self.ie.task_order()
        input_sz = int(np.sqrt(self.ie.get_input_size() / 3))
        yolo_cfg = YoloConfig(model_path, classes, score_threshold, iou_threshold, layers,
                              input_sz, self.ie.get_output_tensors_info()[0]['dtype'], decode_type)
        self.async_yolo = AsyncYolo(self.ie, yolo_cfg, classes, score_threshold, iou_threshold, layers)
        self.pp = PostProcessingRun(yolo_cfg, task_order)

        # 최신 프레임 저장소 (큐 길이 1 느낌)
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_header = None
        self._busy = False  # 추론 중 플래그

        # 주기적 워커: 놀고 있으면 최신 프레임으로 run_async
        self.timer = self.create_timer(0.0, self.worker_tick)  # 가능한 한 바로바로

    # ROS 이미지 → numpy (cv_bridge 없이)
    @staticmethod
    def rosimg_to_numpy(msg: RosImage) -> np.ndarray:
        # 일반적으로 gscam은 bgr8
        if msg.encoding.lower() not in ["bgr8", "rgb8"]:
            # 필요하면 다른 인코딩(YUYV 등) 처리 추가
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        h, w = msg.height, msg.width
        # step = bytes per row
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        # 일부 드라이버는 패딩(step) 포함 → step 기준으로 reshape
        arr = arr.reshape((h, msg.step))
        arr = arr[:, :w * 3]  # 3채널까지만 슬라이스
        img = arr.reshape((h, w, 3))
        if msg.encoding.lower() == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img  # BGR

    # numpy BGR → ROS 이미지
    @staticmethod
    def numpy_to_rosimg(img_bgr: np.ndarray, header: Header = None) -> RosImage:
        msg = RosImage()
        msg.height, msg.width = img_bgr.shape[0], img_bgr.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = img_bgr.shape[1] * 3
        msg.data = img_bgr.tobytes()
        msg.header = header if header is not None else Header()
        return msg

    def image_cb(self, msg: RosImage):
        try:
            frame_bgr = self.rosimg_to_numpy(msg)
        except Exception as e:
            self.get_logger().warning(f"image convert failed: {e}")
            return
        # 최신 프레임로 덮어쓰기 (큐 길이 1 전략)
        with self._frame_lock:
            self._latest_frame = frame_bgr
            self._latest_header = msg.header  # 타임스탬프 유지

    def worker_tick(self):
        if self._busy:
            return
        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame
            header = self._latest_header
            # 사용 후 즉시 버려서 최신만 유지
            self._latest_frame = None
            self._latest_header = None

        try:
            self._busy = True
            # run_async → 결과 수거 → 후처리 → 그리기 → 퍼블리시
            self.async_yolo.run(frame)
            if q.qsize() > 0:
                self.pp.run(self.async_yolo.result_output.get())
                q.get()
                q.task_done()
                self.async_yolo.result_output.task_done()

            if self.pp._queue.qsize() > 0:
                result_frame = self.pp.draw_on_image(self.async_yolo.input_image)  # letterboxed 프레임 기준
                # 퍼블리시 (원하면 원본 크기로 역보정 적용 가능)
                rosimg = self.numpy_to_rosimg(result_frame, header)
                self.pub.publish(rosimg)
        except Exception as e:
            self.get_logger().error(f"inference error: {e}")
        finally:
            self._busy = False


# -----------------------
# 엔트리포인트: ROS 모드로 실행
# -----------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='./example/run_detector/yolov7_example.json', type=str,
                        help='yolo json config path')
    parser.add_argument('--image_topic', default='/camera/image_raw', type=str,
                        help='ROS2 image topic to subscribe')
    parser.add_argument('--pub_topic', default='/dx_yolo/result_image', type=str,
                        help='ROS2 image topic to publish results')
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print("config not found")
        return

    with open(args.config, "r") as f:
        json_config = json.load(f)

    rclpy.init()
    node = RosAsyncRunner(json_config, args.image_topic, args.pub_topic)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
