cmake_minimum_required(VERSION 3.16)
project(logintest VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick)
qt_standard_project_setup(REQUIRES 6.8)

# ✅ 실행 파일 + 백엔드 포함
qt_add_executable(applogintest
    main.cpp
    backend.cpp
    backend.h
)

# ✅ QML 모듈 (URI 이름만 logintest로)
qt_add_qml_module(applogintest
    URI logintest
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

target_link_libraries(applogintest
    PRIVATE Qt6::Core Qt6::Gui Qt6::Qml Qt6::Quick
)

set_target_properties(applogintest PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)
