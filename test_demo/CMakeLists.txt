# CMakeList.txt: stereo_match 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.0.0)

# 找到海康威视摄像头的引用包 
set(HIKI_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(CAMERA_CONTROL_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/hikivision_camera")
include_directories(${HIKI_INCLUDE_DIR} ${CAMERA_CONTROL_INCLUDE_DIR})

set(HIKI_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")
set(CAMERA_CONTROL_LIB_DIR "${PROJECT_SOURCE_DIR}/dist/lib")
link_directories(${HIKI_LIB_DIR} ${CAMERA_CONTROL_LIB_DIR})

# 将源代码添加到此项目的可执行文件。


# TODO: 如有需要，请添加测试并安装目标。



SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
