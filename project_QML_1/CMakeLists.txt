cmake_minimum_required(VERSION 3.16)

project(project_QML_1 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON)

# ✅ Qt 모듈 포함
find_package(Qt6 REQUIRED COMPONENTS Quick Mqtt Multimedia)

qt_standard_project_setup(REQUIRES 6.8)

# ✅ 실행 파일 지정
qt_add_executable(appproject_QML_1
    main.cpp
    sensor_data_provider.cpp   # ✅ 여기로 이동
    sensor_data_provider.h
    SystemMonitor.h
    SystemMonitor.cpp
    icons/resources.qrc
)

# ✅ QML 모듈 등록
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
        RESOURCES icons/resources.qrc
        QML_FILES InsightsPanel.qml
        QML_FILES BottomNavigationBar.qml
)

# ✅ 속성 설정
set_target_properties(appproject_QML_1 PROPERTIES
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# ✅ 링크
target_link_libraries(appproject_QML_1
    PRIVATE Qt6::Quick Qt6::Mqtt Qt6::Multimedia
)

# ✅ 설치 경로
include(GNUInstallDirs)
install(TARGETS appproject_QML_1
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
