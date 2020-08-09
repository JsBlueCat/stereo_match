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
add_executable (stereo_grab "stereo_grab.cpp")
add_executable (stereo_clib_grab "stereo_clib_grab.cpp")
add_executable (stereo_clib_grab_xy "stereo_clib_grab_xy.cpp")

# TODO: 如有需要，请添加测试并安装目标。

# 链接OpenCV库文件
target_link_libraries(stereo_grab ${OpenCV_LIBS} hikivision_camera.lib MvCameraControl.lib)
target_link_libraries(stereo_clib_grab ${OpenCV_LIBS} hikivision_camera.lib MvCameraControl.lib)
target_link_libraries(stereo_clib_grab_xy ${OpenCV_LIBS} hikivision_camera.lib MvCameraControl.lib)



SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_library(stereo_camera_lib SHARED stereo_lib.h stereo_lib.cpp)
target_link_libraries (stereo_camera_lib ${OpenCV_LIBS} hikivision_camera.lib MvCameraControl.lib)
INSTALL(TARGETS stereo_camera_lib RUNTIME DESTINATION F://luo_cmake)  
SET_TARGET_PROPERTIES(stereo_camera_lib PROPERTIES LINKER_LANGUAGE C)
