#include "sio_lio/include/config.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <sstream>

namespace sad {

namespace {

// 校验一个 section 内没有未知键。拼错的键名若被静默忽略，参数就不会生效，
// 因此这里选择报错而非容忍。
bool CheckKeys(const YAML::Node& node, const std::string& section,
               std::initializer_list<const char*> allowed, std::string* err) {
    // 段不存在（未定义）或存在但留空（Null，如只写了「ndt:」）都算缺省，
    // 全部沿用代码默认值。注意 Null 节点的 IsMap() 为 false，若不先判空
    // 会被误判为格式错误。
    if (!node || node.IsNull()) {
        return true;
    }
    if (!node.IsMap()) {
        if (err) *err = section + " 必须是键值列表（形如 key: value）";
        return false;
    }
    for (const auto& kv : node) {
        if (!kv.first.IsScalar()) {
            if (err) *err = section + " 中存在非标量键";
            return false;
        }
        const std::string key = kv.first.as<std::string>();
        bool known = false;
        for (const char* a : allowed) {
            if (key == a) {
                known = true;
                break;
            }
        }
        if (!known) {
            if (err) *err = section + " 中存在未知键：" + key;
            return false;
        }
    }
    return true;
}

double GetD(const YAML::Node& n, const char* k, double dflt) {
    return n[k] ? n[k].as<double>() : dflt;
}

int GetI(const YAML::Node& n, const char* k, int dflt) {
    return n[k] ? n[k].as<int>() : dflt;
}

bool GetB(const YAML::Node& n, const char* k, bool dflt) {
    return n[k] ? n[k].as<bool>() : dflt;
}

}  // namespace

Frontend::Options Config::ToFrontendOptions() const {
    Frontend::Options fo = frontend;      // 取 5 个自有字段
    fo.eskf_opts_ = eskf;                 // 注入滤波参数
    fo.map_opts_.ndt_opts_ = ndt;         // 注入配准参数
    return fo;
}

bool Config::LoadFromYaml(const std::string& path, std::string* err) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        if (err) *err = std::string("无法读取 yaml 文件：") + e.what();
        return false;
    }
    if (!root || !root.IsMap()) {
        if (err) *err = "yaml 根节点必须是键值列表";
        return false;
    }

    // 顶层只允许 init / eskf / ndt / frontend 四个段
    if (!CheckKeys(root, "根节点", {"init", "eskf", "ndt", "frontend"}, err)) {
        return false;
    }

    // ---- init：初始条件 ----
    if (!CheckKeys(root["init"], "init", {"bg", "ba_x", "ba_y"}, err)) {
        return false;
    }
    const YAML::Node ni = root["init"];
    init_bg = GetD(ni, "bg", init_bg);
    init_ba.x() = GetD(ni, "ba_x", init_ba.x());
    init_ba.y() = GetD(ni, "ba_y", init_ba.y());

    // ---- eskf：滤波参数 ----
    if (!CheckKeys(root["eskf"], "eskf",
                   {"gyro_var", "acce_var", "bias_gyro_var", "bias_acce_var",
                    "odom_var", "lidar_pos_noise", "lidar_ang_noise", "imu_dt"},
                   err)) {
        return false;
    }
    const YAML::Node ne = root["eskf"];
    eskf.gyro_var_ = GetD(ne, "gyro_var", eskf.gyro_var_);
    eskf.acce_var_ = GetD(ne, "acce_var", eskf.acce_var_);
    eskf.bias_gyro_var_ = GetD(ne, "bias_gyro_var", eskf.bias_gyro_var_);
    eskf.bias_acce_var_ = GetD(ne, "bias_acce_var", eskf.bias_acce_var_);
    eskf.odom_var_ = GetD(ne, "odom_var", eskf.odom_var_);
    eskf.lidar_pos_noise_ = GetD(ne, "lidar_pos_noise", eskf.lidar_pos_noise_);
    eskf.lidar_ang_noise_ = GetD(ne, "lidar_ang_noise", eskf.lidar_ang_noise_);
    eskf.imu_dt_ = GetD(ne, "imu_dt", eskf.imu_dt_);

    // ---- ndt：配准与地图参数 ----
    if (!CheckKeys(root["ndt"], "ndt",
                   {"voxel_size", "min_pts_num", "max_pts_num", "max_capacity",
                    "max_iteration", "eps", "res_outlier_th", "use_svd_inverse"},
                   err)) {
        return false;
    }
    const YAML::Node nn = root["ndt"];
    ndt.voxel_size_ = GetD(nn, "voxel_size", ndt.voxel_size_);
    ndt.min_pts_num_ = GetI(nn, "min_pts_num", ndt.min_pts_num_);
    ndt.max_pts_num_ = GetI(nn, "max_pts_num", ndt.max_pts_num_);
    ndt.max_capacity_ = GetI(nn, "max_capacity", ndt.max_capacity_);
    ndt.max_iteration_ = GetI(nn, "max_iteration", ndt.max_iteration_);
    ndt.eps_ = GetD(nn, "eps", ndt.eps_);
    ndt.res_outlier_th_ = GetD(nn, "res_outlier_th", ndt.res_outlier_th_);
    ndt.use_svd_inverse_ = GetB(nn, "use_svd_inverse", ndt.use_svd_inverse_);

    // ---- frontend：调度参数 ----
    if (!CheckKeys(root["frontend"], "frontend",
                   {"keyframe_dist_th", "keyframe_rot_th_deg", "imu_buffer_size",
                    "use_filtered_pose_for_map", "enable_undistort"},
                   err)) {
        return false;
    }
    const YAML::Node nf = root["frontend"];
    frontend.keyframe_dist_th_ = GetD(nf, "keyframe_dist_th", frontend.keyframe_dist_th_);
    // 角度以「度」书写，便于读数；结构体内为弧度
    const double rot_deg = GetD(nf, "keyframe_rot_th_deg",
                                frontend.keyframe_rot_th_ * 180.0 / M_PI);
    frontend.keyframe_rot_th_ = rot_deg * M_PI / 180.0;
    if (nf["imu_buffer_size"]) {
        frontend.imu_buffer_size_ = (size_t)nf["imu_buffer_size"].as<int>();
    }
    frontend.use_filtered_pose_for_map_ =
        GetB(nf, "use_filtered_pose_for_map", frontend.use_filtered_pose_for_map_);
    frontend.enable_undistort_ = GetB(nf, "enable_undistort", frontend.enable_undistort_);

    // ---- 数值合法性（明显错误的配置尽早暴露）----
    if (ndt.voxel_size_ <= 0.0) {
        if (err) *err = "ndt.voxel_size 必须为正";
        return false;
    }
    if (ndt.min_pts_num_ < 3) {
        // 协方差估计至少需要 3 个点，否则 sigma_ 退化
        if (err) *err = "ndt.min_pts_num 不得小于 3（协方差估计的下限）";
        return false;
    }
    if (ndt.max_pts_num_ < ndt.min_pts_num_) {
        if (err) *err = "ndt.max_pts_num 不得小于 min_pts_num";
        return false;
    }
    if (eskf.odom_var_ <= 0.0 || eskf.lidar_pos_noise_ <= 0.0 ||
        eskf.lidar_ang_noise_ <= 0.0) {
        if (err) *err = "eskf 中 odom_var / lidar_pos_noise / lidar_ang_noise 必须为正";
        return false;
    }
    if (frontend.imu_buffer_size_ < 2) {
        // 去畸变插值需要至少两条位姿
        if (err) *err = "frontend.imu_buffer_size 不得小于 2（插值需要两条位姿）";
        return false;
    }
    return true;
}

std::string Config::ToYamlString() const {
    std::ostringstream os;
    os << "init:\n";
    os << "  bg: " << init_bg << "\n";
    os << "  ba_x: " << init_ba.x() << "\n";
    os << "  ba_y: " << init_ba.y() << "\n";
    os << "eskf:\n";
    os << "  gyro_var: " << eskf.gyro_var_ << "\n";
    os << "  acce_var: " << eskf.acce_var_ << "\n";
    os << "  bias_gyro_var: " << eskf.bias_gyro_var_ << "\n";
    os << "  bias_acce_var: " << eskf.bias_acce_var_ << "\n";
    os << "  odom_var: " << eskf.odom_var_ << "\n";
    os << "  lidar_pos_noise: " << eskf.lidar_pos_noise_ << "\n";
    os << "  lidar_ang_noise: " << eskf.lidar_ang_noise_ << "\n";
    os << "  imu_dt: " << eskf.imu_dt_ << "\n";
    os << "ndt:\n";
    os << "  voxel_size: " << ndt.voxel_size_ << "\n";
    os << "  min_pts_num: " << ndt.min_pts_num_ << "\n";
    os << "  max_pts_num: " << ndt.max_pts_num_ << "\n";
    os << "  max_capacity: " << ndt.max_capacity_ << "\n";
    os << "  max_iteration: " << ndt.max_iteration_ << "\n";
    os << "  eps: " << ndt.eps_ << "\n";
    os << "  res_outlier_th: " << ndt.res_outlier_th_ << "\n";
    os << "  use_svd_inverse: " << (ndt.use_svd_inverse_ ? "true" : "false") << "\n";
    os << "frontend:\n";
    os << "  keyframe_dist_th: " << frontend.keyframe_dist_th_ << "\n";
    os << "  keyframe_rot_th_deg: " << frontend.keyframe_rot_th_ * 180.0 / M_PI << "\n";
    os << "  imu_buffer_size: " << frontend.imu_buffer_size_ << "\n";
    os << "  use_filtered_pose_for_map: "
       << (frontend.use_filtered_pose_for_map_ ? "true" : "false") << "\n";
    os << "  enable_undistort: " << (frontend.enable_undistort_ ? "true" : "false") << "\n";
    return os.str();
}

}  // namespace sad
