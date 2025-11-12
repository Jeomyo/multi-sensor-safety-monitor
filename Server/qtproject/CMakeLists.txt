cmake_minimum_required(VERSION 3.16)
project(qtproject VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON) # 자동으로 사진 찾아서 등록

find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick Mqtt)
qt_standard_project_setup(REQUIRES 6.8)

# ✅ 실행 파일 + 백엔드 포함
qt_add_executable(appqtproject
    main.cpp
    backend.cpp
    backend.h
    images/resources.qrc
)

# ✅ QML 모듈
qt_add_qml_module(appqtproject
    URI qtproject
    VERSION 1.0
    RESOURCE_PREFIX "/"
    QML_FILES
        qml/AppWindow.qml
        qml/LoginPage.qml
        qml/Main.qml
        qml/DashboardPage.qml
        qml/NavigationButton.qml
        qml/GaugeView.qml
        QML_FILES qml/MapPage.qml
        QML_FILES qml/AnalysisPage.qml

)

#qt_add_resources(appmqtttest "resources"
#    FILES
#        images/resources.qrc
#)

target_link_libraries(appqtproject
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Qml
        Qt6::Quick
        Qt6::Mqtt
)

set_target_properties(appqtproject PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)
