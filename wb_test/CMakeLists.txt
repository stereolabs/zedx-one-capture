cmake_minimum_required(VERSION 3.5)
project(wb_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Argus capture library (built from parent lib/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib/build)

# Tegra MM
set(TegraMM_ROOT /usr/src/jetson_multimedia_api)
if(EXISTS ${TegraMM_ROOT})
  include_directories(
    ${TegraMM_ROOT}/include
    ${TegraMM_ROOT}/include/libjpeg-8b
    /usr/include/libdrm
  )
  link_directories(
    /usr/lib/aarch64-linux-gnu/tegra
    /usr/lib/aarch64-linux-gnu
    /usr/lib/aarch64-linux-gnu/nvidia
  )
  set(TegraMM_LIBRARIES
    nvargus_socketclient nvjpeg drm nvbufsurftransform
    nvbufsurface nvosd EGL v4l2 GLESv2 X11 pthread vulkan
  )
endif()

add_executable(wb_test main.cpp)
target_link_libraries(wb_test arguscapture pthread ${OpenCV_LIBRARIES} ${TegraMM_LIBRARIES})
