# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.0.0)

project ("fine_tuning_railway")
# 添加c++11的支持
set(CMAKE_CXX_FLAGS "-std=c++11")
# 添加编译模式
# set(CMAKE_BUILD_TYPE Release)#compile mode

# 设定编译输出的位置
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/dist/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/dist/lib)

##############################################################################################################
###################################     有关第三方库的使用     ###############################################

############################################## OpenCV ########################################################
# 查找 OpenCV3.4.7的头文件和库文件路径以及库文件
set(OpenCV_DIR "D:/opencv-3.4.7/build/install")
find_package(OpenCV 3.4.7 REQUIRED)
# 包含OpenCV头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBS: ${OpenCV_LIBS}")



# 包含子项目。
add_subdirectory ("stereo_clib")
add_subdirectory ("stereo_match")
add_subdirectory ("hikivision_camera")
add_subdirectory ("coodinate_trans")
add_subdirectory ("real_time_test")
add_subdirectory ("stereo_grab")
add_subdirectory ("test_demo")