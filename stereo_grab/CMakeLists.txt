# CMakeList.txt: stereo_match 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.0.0)

# 将源代码添加到此项目的可执行文件。
add_executable (stereo_grab "stereo_grab.cpp")
# TODO: 如有需要，请添加测试并安装目标。


# 找到海康威视摄像头的引用包 
set(HIKI_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(HIKI_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")
set(CAMERA_CONTROL_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/hikivision_camera")
set(CAMERA_CONTROL_LIB_DIR "${PROJECT_SOURCE_DIR}/include")
message("HIKI_INCLUDE_DIR: ${HIKI_INCLUDE_DIR}")
message("HIKI_LIB_DIR: ${HIKI_LIB_DIR}")
include_directories(${HIKI_INCLUDE_DIR})
link_directories(${HIKI_LIB_DIR})



# 链接OpenCV库文件
target_link_libraries(stereo_grab ${OpenCV_LIBS})