cmake_minimum_required(VERSION 3.16)

project(project_QML_1 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON)

# Qt 모듈
find_package(Qt6 REQUIRED COMPONENTS Quick Mqtt Multimedia)

qt_standard_project_setup(REQUIRES 6.8)

# 실행파일
qt_add_executable(appproject_QML_1
    main.cpp
    sensor_data_provider.cpp
    sensor_data_provider.h
    SystemMonitor.cpp
    SystemMonitor.h
    LoginManager.cpp
    LoginManager.h
    icons/resources.qrc        # ★ QRC 포함
)

# QML 모듈
qt_add_qml_module(appproject_QML_1
    URI project_QML_1
    VERSION 1.0
    RESOURCE_PREFIX "/"
    QML_FILES
        Main.qml
        Gauge.qml
        MapPage.qml
        Streaming.qml
        DashBoard.qml
        MapPage_2.qml
        SensorGraph.qml
        InsightsPanel.qml
        BottomNavigationBar.qml
        LoginPage.qml
)

# 실행 속성
set_target_properties(appproject_QML_1 PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# 라이브러리 링크
target_link_libraries(appproject_QML_1
    PRIVATE Qt6::Quick Qt6::Mqtt Qt6::Multimedia
)

# 설치 경로
include(GNUInstallDirs)
install(TARGETS appproject_QML_1
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
