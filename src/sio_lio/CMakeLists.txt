cmake_minimum_required(VERSION 3.10)
project(sio_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 包内头文件以 "sio_lio/..." 前缀引用，与 ndtlio 的组织方式一致，便于逐文件对照
include_directories(${PROJECT_SOURCE_DIR}/..)

# 阶段0验证程序：验证 Sophus 编译链接，数值核对第一课练习3
add_executable(stage0_se2_check src/stage0_se2_check.cpp)

install(TARGETS stage0_se2_check
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
