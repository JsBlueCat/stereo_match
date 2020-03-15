# CMakeList.txt: stereo_match 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.0.0)
set(HIKI_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(HIKI_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/")
# 将源代码添加到此项目的可执行文件。
include_directories(. ../hikivision_camera ${HIKI_INCLUDE_DIR})
link_directories(${HIKI_LIB_DIR} ${LIBRARY_OUTPUT_DIRECTORY})

add_executable (stereo_match "stereo_match.cpp" "stereo_match.h")

# TODO: 如有需要，请添加测试并安装目标。
# 链接OpenCV库文件
target_link_libraries(stereo_match ${OpenCV_LIBS} hikivision_camera)