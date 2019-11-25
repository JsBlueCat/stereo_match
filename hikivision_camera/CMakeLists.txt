cmake_minimum_required(VERSION 3.0.0)

# 找到海康威视摄像头的引用包 
set(HIKI_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set(HIKI_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/MvCameraControl.lib")
message("HIKI_INCLUDE_DIR: ${HIKI_INCLUDE_DIR}")
message("HIKI_LIB_DIR: ${HIKI_LIB_DIR}")
include_directories(${HIKI_INCLUDE_DIR})



# 链接海康威视的摄像头驱动
# 设置控制类所在路径
set(HIKI_SRCS hikivision_camera.cpp)
set(HIKI_INCLUDE hikivision_camera.h)
add_library(hikivision_camera STATIC ${HIKI_SRCS} ${HIKI_INCLUDE})

set_target_properties(hikivision_camera PROPERTIES
                        OUTPUT_NAME "hikivision_camera" #设置输出库的名字
                        CLEAN_DIRECT_OUTPUT 1
                        ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/dist/lib" #设置静态库输出路径
                      )

# 将源代码添加到此项目的可执行文件。
add_executable (demo "demo.cpp" "demo.h")
# TODO: 如有需要，请添加测试并安装目标。
# 链接OpenCV库文件
target_link_libraries(demo 
${OpenCV_LIBS}
${HIKI_LIB_DIR}
)