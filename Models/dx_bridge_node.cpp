#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>

#pragma pack(push,1)
struct DxHdr {
  uint32_t magic;
  uint16_t version;
  uint16_t flags;
  uint64_t seq_id;
  int64_t  stamp_nsec;
  uint32_t img_w, img_h;
  uint32_t count;
  uint32_t reserved;
};
struct DxObjV2 {
  float x, y, w, h;
  float score;
  uint32_t label;
  uint32_t track_id; // ← V2에서 추가된 필드
};
#pragma pack(pop)

static inline rclcpp::Time nsec_to_ros(int64_t ns, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME) {
  return rclcpp::Time(ns, RCL_SYSTEM_TIME);
}

class DxBridge : public rclcpp::Node {
public:
  DxBridge()
  : Node("dx_bridge")
  {
    declare_parameter<std::string>("socket_path_dets", "/tmp/dx_det.sock");
    declare_parameter<std::string>("socket_path_preview", "/tmp/dx_det_preview.sock");
    declare_parameter<std::string>("frame_id", "camera_link");
    declare_parameter<int>("recv_buf_bytes", 1<<20); // 1MB
    // publishers
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/dx/detections", rclcpp::SensorDataQoS());
    img_pub_ = create_publisher<sensor_msgs::msg::Image>("/dx/annotated_image", rclcpp::SensorDataQoS());

    // open sockets
    auto det_path = get_parameter("socket_path_dets").as_string();
    auto prev_path = get_parameter("socket_path_preview").as_string();
    int rcv = get_parameter("recv_buf_bytes").as_int();

    det_fd_ = open_udsd(det_path.c_str(), rcv);
    prev_fd_ = open_udsd(prev_path.c_str(), rcv);

    if (det_fd_ < 0) RCLCPP_FATAL(get_logger(), "Failed to bind %s", det_path.c_str());
    if (prev_fd_ < 0) RCLCPP_WARN(get_logger(), "Preview socket not bound: %s", prev_path.c_str());

    // timers (non-blocking poll)
    det_timer_ = create_wall_timer(std::chrono::milliseconds(2), [this](){ poll_detections(); });
    prev_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this](){ poll_preview(); });

    frame_id_ = get_parameter("frame_id").as_string();
  }

  ~DxBridge() override {
    if (det_fd_>=0) ::close(det_fd_);
    if (prev_fd_>=0) ::close(prev_fd_);
  }

private:
  int open_udsd(const char* path, int rcvbuf) {
    // cleanup previous
    ::unlink(path);
    int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) return -1;
    if (rcvbuf > 0) ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      ::close(fd);
      return -1;
    }
    RCLCPP_INFO(get_logger(), "UDS bound: %s", path);
    return fd;
  }

  void poll_detections() {
    if (det_fd_ < 0) return;

    // 한 타이머 틱에서 최대 8개까지 비블로킹 수신
    for (int i = 0; i < 8; ++i) {
      ssize_t n = ::recv(det_fd_, buf_, sizeof(buf_), MSG_DONTWAIT);
      if (n <= 0) break;  // 더 이상 읽을 게 없음

      if (static_cast<size_t>(n) < sizeof(DxHdr)) continue;
      auto* H = reinterpret_cast<DxHdr*>(buf_);

      if (H->magic != 0xABCD1204 || H->count > 10000) continue;

      size_t need = sizeof(DxHdr) + static_cast<size_t>(H->count) * sizeof(DxObjV2);
      if (static_cast<size_t>(n) != need) continue;

      auto* D = reinterpret_cast<DxObjV2*>(buf_ + sizeof(DxHdr));
      publish_detections(*H, D);
    }
  }


  // 기존: void publish_detections(const DxHdr& H, const DxDet* D)
  void publish_detections(const DxHdr& H, const DxObjV2* D) {
    vision_msgs::msg::Detection2DArray arr;
    arr.header.stamp = nsec_to_ros(H.stamp_nsec);
    arr.header.frame_id = frame_id_;
    arr.detections.resize(H.count);

    for (uint32_t i = 0; i < H.count; i++) {
      auto& out = arr.detections[i];
      out.header = arr.header;

      // ▸ bbox 세팅
      out.bbox.center.position.x = D[i].x + 0.5 * D[i].w;
      out.bbox.center.position.y = D[i].y + 0.5 * D[i].h;
      out.bbox.size_x = D[i].w;
      out.bbox.size_y = D[i].h;

      // ▸ class / score
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = std::to_string(D[i].label);
      hyp.hypothesis.score    = D[i].score;
      out.results.push_back(hyp);

      // ▸ **여기서 track_id를 Detection2D.id에 세팅**
      //    0xFFFFFFFF는 "유효하지 않은 트랙"이라 비워두는 편이 깔끔
      if (D[i].track_id != 0xFFFFFFFFu) {
        out.id = std::to_string(D[i].track_id);
      } else {
        out.id.clear();   // 또는 그냥 아무 것도 안 해도 됨
      }
    }

    det_pub_->publish(arr);
  }



  void poll_preview() {
    if (prev_fd_ < 0) return;
    for (int i=0;i<4;i++) {
      ssize_t n = ::recv(prev_fd_, buf_, sizeof(buf_), MSG_DONTWAIT);
      if (n <= 0) break;
      if (static_cast<size_t>(n) < sizeof(DxHdr)+sizeof(uint32_t)) continue;
      auto* H = reinterpret_cast<DxHdr*>(buf_);
      if (H->magic != 0xABCD1204) continue;

      uint32_t jpeg_size{};
      std::memcpy(&jpeg_size, buf_+sizeof(DxHdr), sizeof(uint32_t));
      size_t need = sizeof(DxHdr)+sizeof(uint32_t)+jpeg_size;
      if (need != static_cast<size_t>(n)) continue;

      auto* jpeg_ptr = reinterpret_cast<uchar*>(buf_+sizeof(DxHdr)+sizeof(uint32_t));
      cv::Mat img = cv::imdecode(cv::Mat(1, jpeg_size, CV_8UC1, jpeg_ptr), cv::IMREAD_COLOR);
      if (img.empty()) continue;

      std_msgs::msg::Header hdr;
      hdr.stamp = nsec_to_ros(H->stamp_nsec);
      hdr.frame_id = frame_id_;
      auto msg = cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg();
      img_pub_->publish(*msg);
    }
  }


  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::TimerBase::SharedPtr det_timer_, prev_timer_;
  std::string frame_id_;

  int det_fd_{-1}, prev_fd_{-1};
  alignas(8) uint8_t buf_[2<<20]; // 2MB buffer
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxBridge>());
  rclcpp::shutdown();
  return 0;
}

