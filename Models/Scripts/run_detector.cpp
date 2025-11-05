#include "detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace rapidjson;

const char *usage =
    "detector template\n"
    "  -c, --config       use config json file for run application\n"
    "  -h, --help         show help\n";

void help()
{
    std::cout << usage << std::endl;
}

int main(int argc, char *argv[])
{
    DXRT_TRY_CATCH_BEGIN
    int arg_idx = 1;
    std::string configPath = "";
    char key;

    if (argc == 1)
    {
        std::cout << "Error: no arguments." << std::endl;
        help();
        exit(-1);
    }

    while (arg_idx < argc) {
        std::string arg(argv[arg_idx++]);
        if (arg == "-c" || arg == "--config")
                        configPath = strdup(argv[arg_idx++]);
        else if (arg == "-h" || arg == "--help")
                        help(), exit(0);
        else
                        help(), exit(0);
    }
    if(configPath.empty())
    {
        std::cout << "error : no config json file arguments. " << std::endl;
        help();
        exit(-1);
    }

    dxapp::AppConfig appConfig(configPath);
    Detector detector(appConfig);

    // --- ROS2 초기화 & 퍼블리셔 생성 (여기!) ---
    rclcpp::init(argc, argv);
    auto node    = rclcpp::Node::make_shared("dx_detector");
    auto img_pub = node->create_publisher<sensor_msgs::msg::Image>("/dx/result_image", 10);
    auto det_pub = node->create_publisher<vision_msgs::msg::Detection2DArray>("/dx/detections", 10);

    detector.makeThread();
    detector.startThread();
    while(true)
    {
        if(detector.status() != true)
        {
            detector.quitThread();
            break;
        }
#if __riscv
        key = getchar();
#else
        // 1) 결과 이미지 가져오기
        cv::Mat view = detector.totalView();

        // 2) 이미지 퍼블리시
        {
            std_msgs::msg::Header hdr;
            hdr.stamp = node->get_clock()->now();
            hdr.frame_id = "camera_frame"; // TF에 맞춰 변경
            auto img_msg = cv_bridge::CvImage(hdr, "bgr8", view).toImageMsg();
            img_pub->publish(*img_msg);
        }

        // 3) 디텍션(클래스/박스) 퍼블리시
        {
            auto dets = detector.allDetections(); // <- 이전에 안내한 getLatestDetections()/allDetections 추가되어 있어야 함
            vision_msgs::msg::Detection2DArray arr;
            arr.header.stamp = node->get_clock()->now();
            arr.header.frame_id = "camera_frame";

            for (size_t ch = 0; ch < dets.size(); ++ch) {
                for (auto &obj : dets[ch]._detections) {
                    vision_msgs::msg::Detection2D d;
                    d.header = arr.header;

                    float xmin = obj._bbox._xmin;
                    float ymin = obj._bbox._ymin;
                    float w    = obj._bbox._width;
                    float h    = obj._bbox._height;

                    d.bbox.center.x = xmin + w * 0.5f;
                    d.bbox.center.y = ymin + h * 0.5f;
                    d.bbox.size_x   = w;
                    d.bbox.size_y   = h;

                    vision_msgs::msg::ObjectHypothesisWithPose hyp;
                    hyp.hypothesis.class_id = std::to_string(obj._classId);
                    hyp.hypothesis.score    = obj._conf;
                    d.results.push_back(hyp);

                    arr.detections.push_back(std::move(d));
                }
            }
            det_pub->publish(arr);
        }

        if(appConfig.appType == REALTIME)
        {
            cv::imshow("result", view);
            key = (char)cv::waitKey(1);
        }
        else
        {
            if(detector.is_all_image && appConfig.appType == OFFLINE)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(100000));
            }
            else
            {
                std::cout << "press 'q' to quit. " << std::endl;
                key = (char)getchar();
                std::cout << "pressed key " << key << std::endl;
            }
        }
#endif
        switch (key)
        {
        case 'q':
        case 0x1B:
            detector.quitThread();
            break;
        default:
            break;
        }
        rclcpp::spin_some(node);
    }

    detector.joinThread();
    rclcpp::shutdown();

    std::cout << " detector application End. " << std::endl;
DXRT_TRY_CATCH_END    
    return 0;
}
