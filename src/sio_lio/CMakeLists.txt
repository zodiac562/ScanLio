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

# yaml-cpp：统一配置文件的读取（来源 libyaml-cpp-dev，或 ROS 的 yaml-cpp-vendor）
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

# ROS 2 节点依赖。只有 node/ 下的可执行文件使用这些依赖；
# 核心静态库 sio_lio_core 不链接任何 ROS 库，离线验证程序仍可在纯环境中构建。
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(origincar_msg REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 包内头文件以 "sio_lio/..." 前缀引用，与 ndtlio 的组织方式一致，便于逐文件对照
include_directories(${PROJECT_SOURCE_DIR}/..)

# include/ 目录下的头文件（frame.h、map.h 等）以裸名引用，需单独加入搜索路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 阶段2/3/4：2D NDT 配准与 ESKF 滤波核心 + 地图管理 + 前端调度（静态库，后续前端模块复用）
add_library(sio_lio_core STATIC
  src/ndt_2d.cpp src/eskf.cpp src/map.cpp src/occupancy_map.cpp src/frontend.cpp src/config.cpp)
target_include_directories(sio_lio_core PUBLIC ${PROJECT_SOURCE_DIR}/..)
target_link_libraries(sio_lio_core PUBLIC yaml-cpp)

# ============================================================================
# 配置文件安装
#
# 作用：让 sio_lio.yaml 在「安装后的目录」中也存在一份，而不是只存在于源码树。
# 安装后的位置为 install/sio_lio/share/sio_lio/config/sio_lio.yaml，
# 该位置是 ROS 2 软件包共享目录的标准结构。将来把算法包装成 ROS 2 节点时，
# 节点可以用 ament_index_cpp::get_package_share_directory("sio_lio") 得到
# share/sio_lio 的绝对路径，再拼上 config/sio_lio.yaml，即可在任意工作目录启动。
#
# 配置期检查文件存在：若文件被改名或删除，cmake 配置阶段直接报错，
# 而不是等安装完成后才发现 share 目录里没有配置文件。
# ============================================================================
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/config/sio_lio.yaml")
  message(FATAL_ERROR "sio_lio: 缺少 ${PROJECT_SOURCE_DIR}/config/sio_lio.yaml，无法安装配置文件")
endif()
install(FILES config/sio_lio.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# ============================================================================
# ROS 2 节点
#
# 职责边界：本可执行文件只做「话题 ↔ 算法结构体」的转换与发布，
# 不含任何算法逻辑（配准、滤波、地图、去畸变全部在 sio_lio_core 内）。
# 部署形态：ros2 run sio_lio sio_lio_node，或 ros2 launch sio_lio sio_lio_node.launch.py
# ============================================================================
add_executable(sio_lio_node node/sio_lio_node.cpp)

# ----------------------------------------------------------------------------
# 依赖链接：两个发行版导出的目标不同，必须分情况处理。
#
#   较新发行版（主机 Lyrical）：每个包都导出聚合导入目标 <包>::<包>，
#     message 包（sensor_msgs、origincar_msg 等）也导出，target_link_libraries 直接可用。
#
#   Humble（板载 /opt/tros/humble）：message 包由本机 rosidl 生成，其 cmake 配置
#     只导出 <包>::<包>__rosidl_generator_c、<包>::<包>__rosidl_typesupport_cpp
#     一类目标，不导出聚合目标 <包>::<包>。此时引用聚合目标会在 Generate 阶段报
#     「Target links to target "X::X" but the target was not found」。
#     该发行版仍提供 ament_target_dependencies 宏，由它按变量拼接 include 与库。
#
# 两级判定：先看聚合目标是否已由 find_package 导出，再退回 ament_target_dependencies。
# 该宏在较新发行版中不存在，所以不能无条件调用。
# ----------------------------------------------------------------------------
function(sio_lio_link_dep target_name pkg)
  if(TARGET ${pkg}::${pkg})
    target_link_libraries(${target_name} ${pkg}::${pkg})
  elseif(COMMAND ament_target_dependencies)
    ament_target_dependencies(${target_name} ${pkg})
  else()
    message(FATAL_ERROR
      "sio_lio: ${pkg} 既未导出聚合目标 ${pkg}::${pkg}，也不存在 ament_target_dependencies 宏，无法链接")
  endif()
endfunction()

target_link_libraries(sio_lio_node sio_lio_core)
foreach(_sio_dep
    rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros origincar_msg ament_index_cpp)
  sio_lio_link_dep(sio_lio_node ${_sio_dep})
endforeach()
install(TARGETS sio_lio_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ============================================================================
# 验证程序（test/ 目录不入库）
#
# test/ 被 .gitignore 排除，因此他人 clone 后该目录不存在。若直接
# add_executable 指向不存在的源文件，CMake 会在配置阶段报错，导致整个包
# 无法构建。这里改为「逐个检测源文件是否存在，缺失则跳过并给出提示」：
#   - 本地开发：test/ 存在 → 全部验证程序正常构建
#   - 他人 clone：test/ 缺失 → 只构建 sio_lio_core，不报错
# 也可用 -DSIO_LIO_WITH_TESTS=OFF 强制跳过。
# ============================================================================
option(SIO_LIO_WITH_TESTS "构建 test/ 下的验证程序（源文件存在时才构建）" ON)
set(SIO_LIO_TEST_DIR ${PROJECT_SOURCE_DIR}/../../test)
set(SIO_LIO_TEST_TARGETS "")

# 登记一个验证程序：源文件存在才 add_executable，否则打印提示并跳过
function(sio_lio_add_test name)
  if(NOT EXISTS "${SIO_LIO_TEST_DIR}/${name}.cpp")
    message(STATUS "sio_lio: 跳过验证程序 ${name}（缺 ${SIO_LIO_TEST_DIR}/${name}.cpp）")
    return()
  endif()
  add_executable(${name} "${SIO_LIO_TEST_DIR}/${name}.cpp")
  target_link_libraries(${name} sio_lio_core)
  set(SIO_LIO_TEST_TARGETS ${SIO_LIO_TEST_TARGETS} ${name} PARENT_SCOPE)
endfunction()

if(SIO_LIO_WITH_TESTS)
  message(STATUS "sio_lio: 验证程序目录 ${SIO_LIO_TEST_DIR}")
  # 阶段0：验证 Sophus 编译链接
  sio_lio_add_test(stage0_se2_check)
  # 阶段1：静态初始化（零偏与噪声标定）
  sio_lio_add_test(stage1_static_init_check)
  # 阶段2：运动数据上的 NDT 配准
  sio_lio_add_test(stage2_motion_check)
  sio_lio_add_test(stage2_scan_match_check)
  # 阶段3：ESKF 冒烟与统计测试
  sio_lio_add_test(stage3_smoke_test)
  sio_lio_add_test(stage3_eskf_test)
  # 零偏作为滤波器状态的在线收敛（给错误初值，看是否朝真值靠近）
  sio_lio_add_test(stage3_bias_convergence_test)
  # 阶段4：NDT 增强、地图管理、管线验收、可视化
  sio_lio_add_test(stage4_ndt_enhance_check)
  sio_lio_add_test(stage4_map_check)
  sio_lio_add_test(stage4_map_gen)
  sio_lio_add_test(stage4_lio_check)
  sio_lio_add_test(stage4_ab_5way)
  sio_lio_add_test(stage4_traj_dump)
  sio_lio_add_test(stage4_undistort_effect_check)
  # 只读接口：ESKF 协方差 / 体素中心 / 占据栅格
  sio_lio_add_test(stage4_api_check)
  # 统一配置解析器
  sio_lio_add_test(config_check)
else()
  message(STATUS "sio_lio: SIO_LIO_WITH_TESTS=OFF，跳过全部验证程序")
endif()

if(SIO_LIO_TEST_TARGETS)
  list(LENGTH SIO_LIO_TEST_TARGETS _n)
  message(STATUS "sio_lio: 已登记 ${_n} 个验证程序")
  install(TARGETS ${SIO_LIO_TEST_TARGETS}
    RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "sio_lio: 未登记任何验证程序，仅构建 sio_lio_core")
endif()

ament_package()
