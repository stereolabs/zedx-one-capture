cmake_minimum_required(VERSION 3.5)
project(raw_camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

set(TegraMM_ROOT /usr/src/jetson_multimedia_api)
set(TegraMM_FOUND FALSE)

if(EXISTS ${TegraMM_ROOT})
  # set packages
  set(TegraMM_INCLUDE_DIRS ${TegraMM_ROOT}/include ${TegraMM_ROOT}/include/libjpeg-8b /usr/include/libdrm)
  set(TegraMM_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/tegra /usr/lib/aarch64-linux-gnu)
  set(TegraMM_LIBRARIES nvargus_socketclient nvjpeg drm nvbufsurftransform nvbufsurface nvosd EGL v4l2 GLESv2 X11 pthread vulkan)
  file(GLOB TegraMM_COMMON_SOURCES ${TegraMM_ROOT}/samples/common/classes/*.cpp)
  include_directories(${TegraMM_INCLUDE_DIRS})
  link_directories(${TegraMM_LIBRARY_DIRS})
  set(TegraMM_FOUND TRUE)
endif()

add_definitions(-std=c++14 -O3)
IF (BUILD_WITH_DEBUGINFOS)
    message("!! Building with -g !!")
    add_definitions(-g)
    set(ARGUS_LIB arguscapture)
ELSE()
    set(ARGUS_LIB arguscapture)
ENDIF()

add_executable(raw_camera main.cpp)
target_link_libraries(raw_camera ${ARGUS_LIB} pthread ${OpenCV_LIBRARIES} ${TegraMM_LIBRARIES})

