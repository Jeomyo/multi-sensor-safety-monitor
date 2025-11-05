# template Example : Classification

set(run_det_src run_detector.cpp)
add_executable(run_detector ${run_det_src})

# 기존 매크로
add_dxrt_lib()
add_opencv()

# === (1) ROS2 패키지 찾기 ===
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# === (2) include 경로(일부 환경에서 필요) ===
target_include_directories(run_detector
  PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${vision_msgs_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

# === (3) 라이브러리 링크 ===
# 우선 modern exported targets가 있으면 그걸 쓰고,
# 없으면 변수를 통해 링크하는 두 가지 경로를 모두 대비.
if(TARGET rclcpp::rclcpp)
  target_link_libraries(run_detector
    PRIVATE
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs
      vision_msgs::vision_msgs
      cv_bridge::cv_bridge
  )
else()
  target_link_libraries(run_detector
    PRIVATE
      ${rclcpp_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
      ${vision_msgs_LIBRARIES}
      ${cv_bridge_LIBRARIES}
  )
endif()

# 기존 매크로 (설치/옵션 처리 등)
add_target(run_detector)
