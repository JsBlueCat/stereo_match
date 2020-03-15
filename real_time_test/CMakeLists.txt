# CMakeList.txt: stereo_match 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.0.0)

# 将源代码添加到此项目的可执行文件。
add_executable (real_time_test "real_time_test.cpp")
# TODO: 如有需要，请添加测试并安装目标。
# 链接OpenCV库文件
target_link_libraries(real_time_test ${OpenCV_LIBS})