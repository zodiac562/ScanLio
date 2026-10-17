cmake_minimum_required(VERSION 3.10)
project(sio_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 导出 compile_commands.json 到 build 目录，供 clangd 读取真实编译路径
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 包内头文件以 "sio_lio/..." 前缀引用，与 ndtlio 的组织方式一致，便于逐文件对照
include_directories(${PROJECT_SOURCE_DIR}/..)

# 阶段2/3：2D NDT 配准与 ESKF 滤波核心（静态库，后续前端模块复用）
add_library(sio_lio_core STATIC src/ndt_2d.cpp src/eskf.cpp)
target_include_directories(sio_lio_core PUBLIC ${PROJECT_SOURCE_DIR}/..)

# 阶段0验证程序：验证 Sophus 编译链接，数值核对第一课练习3
add_executable(stage0_se2_check src/stage0_se2_check.cpp)

# 阶段3冒烟测试：手工构造数据走通 ESKF 五接口（不依赖数据集）
add_executable(stage3_smoke_test ${PROJECT_SOURCE_DIR}/../../test/stage3_smoke_test.cpp)
target_link_libraries(stage3_smoke_test sio_lio_core)

# 阶段3统计与实测测试：蒙特卡洛无偏性/噪声标度律/真实静止数据（读 test/data）
add_executable(stage3_eskf_test ${PROJECT_SOURCE_DIR}/../../test/stage3_eskf_test.cpp)
target_link_libraries(stage3_eskf_test sio_lio_core)

install(TARGETS stage0_se2_check stage3_smoke_test stage3_eskf_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
