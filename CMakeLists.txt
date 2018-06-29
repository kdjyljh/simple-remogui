cmake_minimum_required(VERSION 3.9.6 FATAL_ERROR)
project(simple-remogui)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_SOURCE_DIR})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if (UNIX)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -std=c99")
    #不进行编译优化，方便调试
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g  -O0 -Wall -Wno-reorder -std=c++11 -Werror=return-type")

    set(CMAKE_PREFIX_PATH
            $ENV{HOME}/Qt5.10.0/5.10.0/gcc_64
            ${CMAKE_PREFIX_PATH})

    set(remoguiroot $ENV{HOME}/remogui)
endif ()

find_package(Qt5 COMPONENTS Core Gui Concurrent Widgets Network Multimedia REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED)

set(SOURCE_FILES
        main.cpp
        mainwindow.cpp
        MediaStreamProc.cpp
#        commlog.cpp
        MediaStreamWidget.cpp)

include_directories(
        custom-ffmpeg
        ${include_directories})

set(CUSTOM_LIBS
        avdevice avfilter avformat avcodec swresample swscale avutil
        vdpau va X11 va-drm va-x11
        dl pthread m
        boost_system boost_thread boost_filesystem boost_regex boost_chrono boost_date_time boost_atomic
        glog
        protobuf
        Qt5::Gui Qt5::Concurrent Qt5::Core Qt5::Widgets Qt5::Network Qt5::Multimedia)

add_executable(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${CMAKE_PROJECT_NAME} ${CUSTOM_LIBS})