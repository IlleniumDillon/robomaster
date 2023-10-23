# CMakeList.txt: IntelligentUAVChampionship-vision 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#

set(OpenCV_DIR C:/OpenCV/opencv/build/)
find_package(OpenCV REQUIRED)

set(EIGEN3_INCLUDE_DIR C:/eigen-3.4.0/eigen-3.4.0/build/)
#find_package(Eigen3 REQUIRED)

message(STATUS "opencv include path is: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "opencv lib path is: " ${OpenCV_LIBS})

message(STATUS "eigen include path is: " ${EIGEN3_INCLUDE_DIR})
message(STATUS "eigen lib path is: " ${EIGEN3_LIBS})

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

# 将源代码添加到此项目的可执行文件。
add_executable (IntelligentUAVChampionship-vision "IntelligentUAVChampionship-vision.cpp" "IntelligentUAVChampionship-vision.h" "main.cpp")

target_link_libraries(IntelligentUAVChampionship-vision ${OpenCV_LIBS})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET IntelligentUAVChampionship-vision PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。