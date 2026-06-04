# vim: syntax=cmake
cmake_minimum_required(VERSION 3.5)

project(kdemo_main)

set(CMAKE_SYSTEM_NAME Linux)
#set(CMAKE_C_COMPILER aarch64-linux-gcc)
#set(CMAKE_CXX_COMPILER aarch64-linux-c++)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# skip 3rd-party lib dependencies
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../release)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

#rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/3rdparty/librknn_api)
set(RKNN_RT_LIB ${RKNN_API_PATH}/lib/librknnrt.so)

#opencv
set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/3rdparty/opencv/share/OpenCV)
find_package(OpenCV REQUIRED)

#rga
set(RGA_PATH ${CMAKE_SOURCE_DIR}/3rdparty/rga)
set(RGA_LIB ${RGA_PATH}/lib/librga.so)

#encode
set(ENC_PATH ${CMAKE_SOURCE_DIR}/3rdparty/mpp)
set(ENC_LIB ${ENC_PATH}/lib/librockchip_mpp.so)

set(FFMPEG_PATH ${CMAKE_SOURCE_DIR}/3rdparty/ffmpeg)
set(FFMPEG_LIB_DIR ${FFMPEG_PATH}/lib)

set(FFMPEG_LIBS
    ${FFMPEG_LIB_DIR}/libavformat.so
    ${FFMPEG_LIB_DIR}/libavcodec.so
    ${FFMPEG_LIB_DIR}/libavutil.so
    ${FFMPEG_LIB_DIR}/libswscale.so
)

#include
include_directories(.)
include_directories(./v4l2camera)
include_directories(./detect)
include_directories(./bytetrack)
include_directories(./encode)
include_directories(${ENC_PATH}/include)
include_directories(${RKNN_API_PATH}/include)
include_directories(${RGA_PATH}/include)
include_directories(${FFMPEG_PATH}/include)
include_directories(./utils)
include_directories(./serial)
include_directories(./eis)
#source
set(V4L2CAMERA_SRC
	v4l2camera/v4l2camera.c
)

set(DETECT_SRC
	detect/detect.cpp
	detect/postprocess.cc
        utils/dma_alloc.cpp
        bytetrack/BYTETracker.cpp
        bytetrack/kalmanFilter.cpp
        bytetrack/lapjv.cpp
        bytetrack/STrack.cpp
        bytetrack/utils.cpp
        eis/eis.cpp
        eis/sim_motion.cpp
        eis/motion_filter.cpp
)

set(ENCODE_SRC
	encode/encode.c
)

set(SERIAL_SRC
        serial/cserialmngr.cpp
)

set(DEMO_SRC
	${V4L2CAMERA_SRC}
	${DETECT_SRC}
	${ENCODE_SRC}
    ${SERIAL_SRC}
    main.cpp
)

#exec
add_compile_options(-Werror -fPIC -Os)

add_executable(kdemo_main ${DEMO_SRC})

target_link_libraries(kdemo_main
	libv4l2.so
    libdrm.so
    libz.so
	${RKNN_RT_LIB}
	${RGA_LIB}
	${OpenCV_LIBS}
	${ENC_LIB}
    ${FFMPEG_LIBS}
)

set(TEST_SRC
        ${V4L2CAMERA_SRC}
        ${DETECT_SRC}
        ${ENCODE_SRC}
    ${SERIAL_SRC}
    kdemo_test.cpp
)

add_executable(kdemo_test ${TEST_SRC})

target_link_libraries(kdemo_test
        libv4l2.so
    libdrm.so
    libz.so
        ${RKNN_RT_LIB}
        ${RGA_LIB}
        ${OpenCV_LIBS}
        ${ENC_LIB}
    ${FFMPEG_LIBS}
)

# install target and libraries
#install(TARGETS fusion_rknn DESTINATION ./lib)
#install(DIRECTORY model DESTINATION .)
# install(FILES fusion_detect.h DESTINATION ./include) # 已移除
