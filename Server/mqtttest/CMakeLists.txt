cmake_minimum_required(VERSION 3.16)
project(mqtttest VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick Mqtt)
qt_standard_project_setup(REQUIRES 6.8)

# ✅ 실행 파일 + 백엔드 포함
qt_add_executable(appmqtttest
    main.cpp
    backend.cpp
    backend.h
    #mqttmanager.cpp
    #mqttmanager.h
)

# ✅ QML 모듈
qt_add_qml_module(appmqtttest
    URI mqtttest
    VERSION 1.0
    RESOURCE_PREFIX "/"
    QML_FILES
        qml/AppWindow.qml
        qml/LoginPage.qml
        qml/Main.qml
        qml/DashboardPage.qml
        qml/NavigationButton.qml
        qml/GaugeView.qml

)

target_link_libraries(appmqtttest
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Qml
        Qt6::Quick
        Qt6::Mqtt
)

set_target_properties(appmqtttest PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)
