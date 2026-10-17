// 阶段 3 ESKF 统计与实测测试（与 stage3_smoke_test 互补）
//
// 套件 B：蒙特卡洛无偏性——200 次独立运行，陀螺注入固定种子高斯噪声，
//         统计终点 theta 误差的均值（应近零）与标准差（应稳定）
// 套件 C：噪声标度律——噪声标准差 1 倍与 2 倍两组，经验标准差之比应近 2
//         （theta 随机游走：标准差 = sigma·sqrt(T)，与 sigma 线性同比）
// 套件 D：真实静止数据——imu_static 实测 CSV（133.7s），
//         物理真值 = 车静止（p=0, v=0, theta=0）；
//         D1 带雷达+轮速观测：p/v/theta 应保持近零
//         D2 纯 IMU 航位推算：theta 漂移应在随机游走理论界内，p/v 报告数值
//
// 蒙特卡洛场景设计说明：仅轮速+陀螺、无雷达观测。原因：雷达角度观测会把
// theta 钉死在观测值附近，陀螺噪声的统计效果无法显现；去掉雷达后 theta
// 误差由陀螺噪声随机游走主导，理论标准差可解析预测。
//
// 通过标准：全部 CHECK 输出 PASS；任何 FAIL 退出码非 0

#include "sio_lio/src/eskf.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace sad;

namespace {

const double kDt = 0.01;
const double kPi = 3.14159265358979323846;
const int kMcSteps = 2000;   // 蒙特卡洛单次步数（20 秒）

SE2 MakePose(double x, double y, double theta) {
    return SE2(SO2(theta), Vec2d(x, y));
}

int g_fail_count = 0;

void CheckTrue(const char* name, bool ok) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) {
        g_fail_count++;
    }
}

Imu MakeImu(double t, double gyro_z, double ax, double ay) {
    Imu imu;
    imu.timestamp_ = t;
    imu.gyro_ = Vec3d(0.0, 0.0, gyro_z);
    imu.accel_ = Vec3d(ax, ay, 9.81);
    return imu;
}

Odom MakeOdom(double t, double v) {
    Odom odom;
    odom.timestamp_ = t;
    odom.vel_ = v;
    return odom;
}

// 单次蒙特卡洛运行：静止真值 + 陀螺高斯噪声 + 无噪声轮速观测（10Hz），
// 无雷达观测。返回终点 theta。
double McSingleRun(double sigma_gyro, std::mt19937::result_type seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, sigma_gyro);

    ESKF::Options opt;
    ESKF eskf(opt);
    eskf.SetInitCondition(0.0, Vec2d::Zero());

    const int kSteps = kMcSteps;   // 20 秒
    double t = 0.0;
    for (int i = 0; i < kSteps; ++i) {
        double gz = noise(rng);   // 真实角速度为 0，读数 = 纯噪声
        eskf.Predict(MakeImu(t, gz, 0.0, 0.0));
        if (i % 10 == 0) {
            eskf.ObserveOdom(MakeOdom(t, 0.0));
        }
        t += kDt;
    }
    return eskf.GetNominalPose().so2().log();
}

void SuiteB_MonteCarlo() {
    std::printf("--- Suite B: Monte Carlo unbiasedness (200 runs, sigma=0.001 rad/s) ---\n");
    const int kRuns = 200;
    const double kSigma = 0.001;

    std::vector<double> errs(kRuns);
    double sum = 0.0, sum_sq = 0.0;
    for (int k = 0; k < kRuns; ++k) {
        errs[k] = McSingleRun(kSigma, 42 + k);   // 固定种子，可复现
        sum += errs[k];
        sum_sq += errs[k] * errs[k];
    }
    double mean = sum / kRuns;
    double std_dev = std::sqrt(sum_sq / kRuns - mean * mean);

    // 理论：theta 随机游走，噪声以每步角增量 noise·dt 注入（theta_ += w*dt），
    // 单步方差 = (sigma·dt)²，N 步累积标准差 = sigma·sqrt(N)·dt
    // = 0.001·sqrt(2000)·0.01 ≈ 0.000447
    double theory = kSigma * std::sqrt((double)kMcSteps) * kDt;

    std::printf("       mean=%.6f  std=%.6f  theory_std=%.6f\n", mean, std_dev, theory);

    CheckTrue("B: |mean| < 0.002 (unbiased)", std::fabs(mean) < 0.002);
    CheckTrue("B: std < 2·theory (bounded)", std_dev < 2.0 * theory);
    CheckTrue("B: std > 0.25·theory (noise visible)", std_dev > 0.25 * theory);
}

void SuiteC_NoiseScaling() {
    std::printf("--- Suite C: noise scaling (sigma vs 2·sigma) ---\n");
    const int kRuns = 200;

    auto RunSet = [](double sigma, std::mt19937::result_type base_seed) {
        double sum_sq = 0.0;
        for (int k = 0; k < kRuns; ++k) {
            double th = McSingleRun(sigma, base_seed + k);
            sum_sq += th * th;
        }
        return std::sqrt(sum_sq / kRuns);   // RMS 误差
    };

    double rms1 = RunSet(0.001, 1000);
    double rms2 = RunSet(0.002, 1000);
    double ratio = rms2 / rms1;

    std::printf("       rms(sigma)=%.6f  rms(2·sigma)=%.6f  ratio=%.4f (expect ~2)\n",
                rms1, rms2, ratio);
    CheckTrue("C: ratio in [1.4, 2.6]", ratio > 1.4 && ratio < 2.6);
}

// CSV 读取：imu.csv = t,gx,gy,gz,ax,ay,az；odom.csv = t,vel
struct CsvImu {
    double t;
    double gz;
    double ax;
    double ay;
};
struct CsvOdom {
    double t;
    double vel;
};

bool LoadCsv(const std::string& path, std::vector<CsvImu>* imus, std::vector<CsvOdom>* odoms) {
    {
        std::ifstream fin(path + "/imu.csv");
        if (!fin) {
            return false;
        }
        std::string line;
        std::getline(fin, line);   // 表头
        while (std::getline(fin, line)) {
            std::stringstream ss(line);
            std::string cell;
            CsvImu rec{};
            std::getline(ss, cell, ',');
            rec.t = std::stod(cell);
            std::getline(ss, cell, ',');   // gx 忽略
            std::getline(ss, cell, ',');   // gy 忽略
            std::getline(ss, cell, ',');
            rec.gz = std::stod(cell);
            std::getline(ss, cell, ',');
            rec.ax = std::stod(cell);
            std::getline(ss, cell, ',');
            rec.ay = std::stod(cell);
            imus->push_back(rec);
        }
    }
    {
        std::ifstream fin(path + "/odom.csv");
        if (!fin) {
            return false;
        }
        std::string line;
        std::getline(fin, line);
        while (std::getline(fin, line)) {
            std::stringstream ss(line);
            std::string cell;
            CsvOdom rec{};
            std::getline(ss, cell, ',');
            rec.t = std::stod(cell);
            std::getline(ss, cell, ',');
            rec.vel = std::stod(cell);
            odoms->push_back(rec);
        }
    }
    return true;
}

// 按 IMU 数据回放；feed_odom=true 时在每条 odom 时间点前处理完更早的 IMU 再喂轮速；
// with_lidar=true 时在轮速时刻叠加零位姿观测（10 倍间隔取 1）
void ReplayStatic(const std::vector<CsvImu>& imus, const std::vector<CsvOdom>& odoms,
                  double bg, const Vec2d& ba, bool feed_odom, bool with_lidar,
                  double* out_px, double* out_py, double* out_theta, double* out_t) {
    ESKF::Options opt;
    ESKF eskf(opt);
    eskf.SetInitCondition(bg, ba);

    size_t oi = 0;
    long odom_count = 0;
    for (const auto& rec : imus) {
        while (oi < odoms.size() && odoms[oi].t <= rec.t) {
            if (feed_odom) {
                eskf.ObserveOdom(MakeOdom(odoms[oi].t, odoms[oi].vel));
                if (with_lidar && odom_count % 10 == 0) {
                    eskf.ObserveLidar(MakePose(0.0, 0.0, 0.0));
                }
            }
            ++odom_count;
            ++oi;
        }
        eskf.Predict(MakeImu(rec.t, rec.gz, rec.ax, rec.ay));
    }

    SE2 pose = eskf.GetNominalPose();
    *out_px = pose.translation().x();
    *out_py = pose.translation().y();
    *out_theta = pose.so2().log();
    *out_t = imus.back().t - imus.front().t;
}

void SuiteD_RealStatic(const std::string& data_path) {
    std::printf("--- Suite D: real static data (%s) ---\n", data_path.c_str());
    std::vector<CsvImu> imus;
    std::vector<CsvOdom> odoms;
    if (!LoadCsv(data_path, &imus, &odoms)) {
        CheckTrue("D: load CSV", false);
        return;
    }
    std::printf("       loaded imu=%zu odom=%zu\n", imus.size(), odoms.size());

    // 阶段 1 实测：陀螺 z 零偏与加计 xy 均值（含倾斜重力，2D 记入零偏账户）
    const double bg = 0.005403;
    const Vec2d ba(-0.186, 0.054);

    // D1：雷达+轮速观测下保持近零
    double px, py, theta, duration;
    ReplayStatic(imus, odoms, bg, ba, true, true, &px, &py, &theta, &duration);
    std::printf("       D1 (with obs):  pose=(%.6f, %.6f, %.6f)  T=%.1fs\n", px, py, theta, duration);
    CheckTrue("D1: |px| < 0.01", std::fabs(px) < 0.01);
    CheckTrue("D1: |py| < 0.01", std::fabs(py) < 0.01);
    CheckTrue("D1: |theta| < 0.01", std::fabs(theta) < 0.01);

    // D2：纯 IMU 航位推算（不喂轮速、无雷达），theta 漂移限随机游走理论界
    double px2, py2, theta2, duration2;
    ReplayStatic(imus, odoms, bg, ba, false, false, &px2, &py2, &theta2, &duration2);
    // 随机游走理论：theta 标准差 = gyro 噪声标准差(0.000507)·sqrt(T)，取 6 倍作容差
    double theory2 = 6.0 * 0.000507 * std::sqrt(duration2);
    std::printf("       D2 (imu only):  pose=(%.4f, %.4f, %.6f)  |theta|<%.4f\n",
                px2, py2, theta2, theory2);
    CheckTrue("D2: |theta drift| < 6·sigma_rw", std::fabs(theta2) < theory2);
    // p/v 不作硬断言：纯惯导 133s 位置漂移为米级，属预期物理行为，仅报告
    std::printf("       D2 note: dead-reckoning px/py reported only (expected meter-level drift)\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::printf("===== ESKF statistical & real-data test =====\n");
    SuiteB_MonteCarlo();
    SuiteC_NoiseScaling();

    std::string data_path = "/home/zodiac/zodiac_slam/slam_ws/test/data/imu_static";
    if (argc > 1) {
        data_path = argv[1];
    }
    SuiteD_RealStatic(data_path);

    std::printf("===== %s (fail=%d) =====\n", g_fail_count == 0 ? "ALL PASS" : "HAS FAIL", g_fail_count);
    return g_fail_count == 0 ? 0 : 1;
}
