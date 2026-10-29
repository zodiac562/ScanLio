cmake_minimum_required(VERSION 3.10)
project(scanlio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 导出 compile_commands.json 到 build 目录，供 clangd 读取真实编译路径
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# yaml-cpp：统一配置文件的读取（来源 libyaml-cpp-dev，或 ROS 的 yaml-cpp-vendor）
# 变量为单数：yaml-cpp 的 cmake 配置只定义 YAML_CPP_INCLUDE_DIR（见
# /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp/yaml-cpp-config.cmake）。复数名从未定义，
# 展开后是 include_directories()，登记零个目录（2026-10-29 修复 S-01）。
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

# ROS 2 节点依赖。只有 node/ 下的可执行文件使用这些依赖；
# 核心静态库 scanlio_core 不链接任何 ROS 库，离线验证程序仍可在纯环境中构建。
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(origincar_msg REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 包内头文件以 "scanlio/..." 前缀引用，与 ndtlio 的组织方式一致，便于逐文件对照
include_directories(${PROJECT_SOURCE_DIR}/..)

# include/ 目录下的头文件（frame.h、map.h 等）以裸名引用，需单独加入搜索路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 阶段2/3/4：2D NDT 配准与 ESKF 滤波核心 + 地图管理 + 前端调度（静态库，后续前端模块复用）
add_library(scanlio_core STATIC
  src/ndt_2d.cpp src/eskf.cpp src/map.cpp src/occupancy_map.cpp src/frontend.cpp src/config.cpp)

# 头文件搜索路径挂在目标上而不是只靠目录级 include_directories：
#   上一级（src/）供 "scanlio/xxx.h" 形式引用，
#   include/ 供 "frame.h"、"map.h" 这类裸名引用。
# 导出目标时 BUILD_INTERFACE 与 INSTALL_INTERFACE 必须分开写：CMake 不允许导出
# 目标的接口里出现源码目录路径，否则 Generate 阶段报「prefixed in the source directory」。
# 安装后的布局镜像源码结构：install/scanlio/include/scanlio/{include,src}/...，
# 因此使用方的两个搜索路径分别是 include 与 include/scanlio/include。
target_include_directories(scanlio_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/..>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/scanlio/include>)
# Eigen 必须作为 PUBLIC 链接项挂在目标上：公共头文件（eigen_types.h）直接 include
# <Eigen/Core>，而目录级 include_directories 不会随导出目标传给使用者。
target_link_libraries(scanlio_core PUBLIC yaml-cpp Eigen3::Eigen)

# 头文件安装：保持 "scanlio/xxx.h" 与 "scanlio/src/xxx.hpp" 的引用形式不变。
# 源码树里 "scanlio/common/eigen_types.h" 由 -I <包目录>/.. 命中，
# 安装后需把 common/ 一并放到位，否则使用方在 frame.h 处即失败。
install(DIRECTORY include/ DESTINATION include/scanlio/include)
install(DIRECTORY common/ DESTINATION include/scanlio/common)
# 3rdparty/sophus 以 "scanlio/3rdparty/sophus/xxx.hpp" 形式被 eigen_types.h 引用
install(DIRECTORY 3rdparty/ DESTINATION include/scanlio/3rdparty)
install(DIRECTORY src/ DESTINATION include/scanlio/src
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")

# 安装并导出该库。原文件缺少 EXPORT 与 ament_export_*，find_package(scanlio)
# 生成的配置文件里没有目标定义，引用 scanlio::scanlio_core 会报目标不存在。
install(TARGETS scanlio_core
  EXPORT scanlioTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ============================================================================
# 配置文件安装
#
# 作用：让 scanlio.yaml 在「安装后的目录」中也存在一份，而不是只存在于源码树。
# 安装后的位置为 install/scanlio/share/scanlio/config/scanlio.yaml，
# 该位置是 ROS 2 软件包共享目录的标准结构。将来把算法包装成 ROS 2 节点时，
# 节点可以用 ament_index_cpp::get_package_share_directory("scanlio") 得到
# share/scanlio 的绝对路径，再拼上 config/scanlio.yaml，即可在任意工作目录启动。
#
# 配置期检查文件存在：若文件被改名或删除，cmake 配置阶段直接报错，
# 而不是等安装完成后才发现 share 目录里没有配置文件。
# ============================================================================
if(NOT EXISTS "${PROJECT_SOURCE_DIR}/config/scanlio.yaml")
  message(FATAL_ERROR "scanlio: 缺少 ${PROJECT_SOURCE_DIR}/config/scanlio.yaml，无法安装配置文件")
endif()
install(FILES config/scanlio.yaml
  DESTINATION share/${PROJECT_NAME}/config)

# ============================================================================
# ROS 2 节点
#
# 职责边界：本可执行文件只做「话题 ↔ 算法结构体」的转换与发布，
# 不含任何算法逻辑（配准、滤波、地图、去畸变全部在 scanlio_core 内）。
# 部署形态：ros2 run scanlio scanlio_node，或 ros2 launch scanlio scanlio_node.launch.py
# ============================================================================
add_executable(scanlio_node node/scanlio_node.cpp)

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
function(scanlio_link_dep target_name pkg)
  if(TARGET ${pkg}::${pkg})
    target_link_libraries(${target_name} ${pkg}::${pkg})
  elseif(COMMAND ament_target_dependencies)
    ament_target_dependencies(${target_name} ${pkg})
  else()
    message(FATAL_ERROR
      "scanlio: ${pkg} 既未导出聚合目标 ${pkg}::${pkg}，也不存在 ament_target_dependencies 宏，无法链接")
  endif()
endfunction()

target_link_libraries(scanlio_node scanlio_core)
foreach(_sio_dep
    rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros origincar_msg ament_index_cpp)
  scanlio_link_dep(scanlio_node ${_sio_dep})
endforeach()
install(TARGETS scanlio_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

# ============================================================================
# 验证程序（test/ 目录不入库）
#
# test/ 被 .gitignore 排除，因此他人 clone 后该目录不存在。若直接
# add_executable 指向不存在的源文件，CMake 会在配置阶段报错，导致整个包
# 无法构建。这里改为「逐个检测源文件是否存在，缺失则跳过并给出提示」：
#   - 本地开发：test/ 存在 → 全部验证程序正常构建
#   - 他人 clone：test/ 缺失 → 只构建 scanlio_core，不报错
# 也可用 -DSCANLIO_WITH_TESTS=OFF 强制跳过。
# ============================================================================
option(SCANLIO_WITH_TESTS "构建 test/ 下的验证程序（源文件存在时才构建）" ON)
set(SCANLIO_TEST_DIR ${PROJECT_SOURCE_DIR}/../../test)
set(SCANLIO_TEST_TARGETS "")

# 登记一个验证程序：源文件存在才 add_executable，否则打印提示并跳过
function(scanlio_add_test name)
  if(NOT EXISTS "${SCANLIO_TEST_DIR}/${name}.cpp")
    message(STATUS "scanlio: 跳过验证程序 ${name}（缺 ${SCANLIO_TEST_DIR}/${name}.cpp）")
    return()
  endif()
  add_executable(${name} "${SCANLIO_TEST_DIR}/${name}.cpp")
  target_link_libraries(${name} scanlio_core)
  set(SCANLIO_TEST_TARGETS ${SCANLIO_TEST_TARGETS} ${name} PARENT_SCOPE)
endfunction()

if(SCANLIO_WITH_TESTS)
  message(STATUS "scanlio: 验证程序目录 ${SCANLIO_TEST_DIR}")
  # 阶段0：验证 Sophus 编译链接
  scanlio_add_test(stage0_se2_check)
  # 阶段1：静态初始化（零偏与噪声标定）
  scanlio_add_test(stage1_static_init_check)
  # 阶段2：运动数据上的 NDT 配准
  scanlio_add_test(stage2_motion_check)
  scanlio_add_test(stage2_scan_match_check)
  # 阶段3：ESKF 冒烟与统计测试
  scanlio_add_test(stage3_smoke_test)
  scanlio_add_test(stage3_eskf_test)
  # 零偏作为滤波器状态的在线收敛（给错误初值，看是否朝真值靠近）
  scanlio_add_test(stage3_bias_convergence_test)
  # 阶段4：NDT 增强、地图管理、管线验收、可视化
  scanlio_add_test(stage4_ndt_enhance_check)
  scanlio_add_test(stage4_map_check)
  scanlio_add_test(stage4_map_gen)
  scanlio_add_test(stage4_lio_check)
  scanlio_add_test(stage4_ab_5way)
  scanlio_add_test(stage4_traj_dump)
  scanlio_add_test(stage4_undistort_effect_check)
  # 四路对照（以直墙几何作参照）：每路落盘 traj 与 occ，指标由 stage4_wall_check.py 计算
  scanlio_add_test(stage4_wall_check)
  # 只读接口：ESKF 协方差 / 体素中心 / 占据栅格
  scanlio_add_test(stage4_api_check)
  # 统一配置解析器
  scanlio_add_test(config_check)
else()
  message(STATUS "scanlio: SCANLIO_WITH_TESTS=OFF，跳过全部验证程序")
endif()

if(SCANLIO_TEST_TARGETS)
  list(LENGTH SCANLIO_TEST_TARGETS _n)
  message(STATUS "scanlio: 已登记 ${_n} 个验证程序")
  install(TARGETS ${SCANLIO_TEST_TARGETS}
    RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "scanlio: 未登记任何验证程序，仅构建 scanlio_core")
endif()

# 导出声明必须在 ament_package() 之前。
# HAS_LIBRARY_TARGET 不可省：缺它时使用者能链接，但安装后运行时找不到库。
# 相对路径被 ament 拼成 ${prefix}/<路径>（不加 include 前缀），故需显式写 include。
ament_export_targets(scanlioTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include include/scanlio/include)
# 导出目标里含有 Eigen3::Eigen 与 yaml-cpp，使用方必须先 find_package 才能解析，
# 否则 CMake 在读取 scanlioTargetsExport.cmake 时报「target was not found」。
ament_export_dependencies(Eigen3 yaml-cpp)

ament_package()
