#ifndef IESKF_2D_HPP_
#define IESKF_2D_HPP_

#include "scanlio/common/eigen_types.h"
#include "scanlio/common/imu.h"
#include "scanlio/common/odom.h"

#include <functional>

namespace sad {

// 迭代误差状态卡尔曼滤波（2D 紧耦合）。
//
// 与 ESKF 并列存在，由 Frontend::Options::filter_type_ 选择，不作替代。
// 对照原版 NdtLIO 的 dev_ws/src/ndtlio/include/ieskf.h 与 src/ieskf.cpp。
//
// 状态 8 维：p(2) v(2) θ bg ba(2)，与 ESKF 完全相同的维度划分。
//
// 两条路径的耦合层次不同：
//   ESKF   —— NDT 独立迭代到收敛，只把最终的 (x, y, θ) 交给滤波器；
//   IESKF2D —— NDT 只累加每个点的 8 维雅可比与信息矩阵，迭代在滤波器内部完成。
//
// 与原版的差异，逐条列在下面；实现处另有对应注释。
//   1. 接口命名沿用本项目风格（Predict / ObserveOdom / UpdateUsingCustomObserve
//      与大驼峰，参数用 const 引用而非 shared_ptr），不保留原版的小驼峰。
//   2. 噪声选项改名为 *_std_，语义显式声明为标准差，构造时平方成方差。
//      本项目 ESKF::Options 的同名字段直接作方差使用，两套语义相反，
//      所以 yaml 中本类对应的键名带 _std 后缀。
//   3. 首帧保护用 inited_ 标志（与本项目 ESKF 一致），替代原版
//      「dt > 5 * imu_dt_」的判别，因此本类不再需要 imu_dt 选项。
//   4. 误差状态雅可比 F 中 θ 对 θ 的偏导取 1（Identity 基底不变）。
//      原版 ieskf.cpp:90 写作 -dg，使角度方差每步被乘上 dg² 量级而趋近零，
//      与名义积分式 θ' = θ + (ω - bg)·dt 的 ∂θ'/∂θ = 1 不符，按推导改正。
//   5. dx_ 初值置零。原版 ieskf.h:90 以 Vec8d::Identity() 初始化，
//      使首次迭代的流形投影因子 J = 1 - 0.5 × 1 = 0.5，角度相关行列被无故减半。
//   6. 删除原版「accel.z < 2.0 时乘 9.82」的归一化分支。本项目 imu.csv 的
//      accel.z 实测区间 7.296 ~ 16.601（中位 9.774，含重力），该分支永不触发。
//   7. 流形投影只对与角度相关的交叉项乘 J²、对角项 (4,4) 乘一次 J²。
//      原版先乘第 4 行再乘第 4 列，(4,4) 被乘两次成为 J⁴，与 G·P·Gᵀ 不符。
class IESKF2D {
   public:
    struct Options {
        // 下列五项均为「标准差」，构造时平方成方差后填入 Q 与观测噪声。
        // 默认值取 ESKF 对应方差的算术平方根，便于两条路径在同等噪声设定下对照。
        double gyro_std_ = 0.001;          // 陀螺角速度标准差，(rad/s)
        double acce_std_ = 0.0316228;      // 加计比力标准差，(m/s^2)
        double bias_gyro_std_ = 0.00378209;  // 陀螺零偏随机游走标准差
        double bias_acce_std_ = 0.00961957;  // 加计零偏随机游走标准差
        double odom_std_ = 0.316228;       // 轮速观测标准差，(m/s)

        int num_iterations_ = 3;           // 测量更新内的最大迭代次数
        double eps_ = 1e-3;                // 迭代收敛判据：增量范数阈值
        double info_ratio_ = 0.01;         // 逐点信息矩阵的整体缩放因子

        bool update_bias_gyro_ = true;     // 是否注入陀螺零偏修正
        bool update_bias_acce_ = true;     // 是否注入加计零偏修正
    };

    // 自定义观测：给定线性化点，输出 Σ JᵀΩJ 与 Σ -JᵀΩr。
    // 与 NDT2D::compute_residual_and_jacobians 的签名对齐。
    using CustomObsFunc = std::function<void(const SE2& input_pose,
                                             Mat8d& HT_Vinv_H,
                                             Vec8d& HT_Vinv_r)>;

    IESKF2D() { ResetCovariance(); }
    explicit IESKF2D(const Options& options) : options_(options) { ResetCovariance(); }

    // 静态初始化：写入零偏，不改 inited_。首条 IMU 由 Predict 建立时间基准。
    void SetInitCondition(double bg, const Vec2d& ba);

    // 直接写入全部名义状态（对照原版 ieskf.h:51 的 setX）。
    // 用途：单元测试给偏移初值、调试时把状态拉到已知点。
    // 副作用：同时把 last_imu_time_ 设为 timestamp 并置 inited_，
    //        调用后第一条 IMU 就会参与积分，不再被首帧保护丢弃。
    void SetState(const Vec2d& p, const Vec2d& v, double theta, double bg, const Vec2d& ba,
                  double timestamp);

    // IMU 递推。首条 IMU 只建立时间基准并返回 false。
    bool Predict(const Imu& imu);

    // 轮速观测（2 维）：车体系速度 (v, 0) 旋转到世界系后与状态速度比较。
    // 第二分量恒 0 属非完整约束假设，不是测量值 —— 底盘消息的 y 字段实测恒 0。
    bool ObserveOdom(const Odom& odom);

    // 紧耦合观测更新：内部按 num_iterations_ 次反复线性化并注入名义状态。
    bool UpdateUsingCustomObserve(const CustomObsFunc& obs);

    SE2 GetNominalPose() const { return SE2(SO2(theta_), p_); }

    // 名义状态按 8 维向量返回：p(0~1) v(2~3) θ(4) bg(5) ba(6~7)
    Vec8d GetNominal() const;

    // 后验误差协方差（按值返回副本）
    Mat8d GetP() const { return P_; }

    // 当前名义零偏。只读，用于验证收敛。
    void GetBias(double* bg, Vec2d* ba) const;

   private:
    // 把 dx_ 注入名义状态并折回角度主值区间
    void Inject();

    // 流形投影：误差状态注入后，与角度相关的项乘 J²（见类注释第 7 条）
    void ProjectCovariance();

    // 协方差初值：位置与速度 1e-4，角度与三项零偏 0.01（与原版构造一致）
    void ResetCovariance();

    Options options_;

    // 名义状态
    Vec2d p_ = Vec2d::Zero();
    Vec2d v_ = Vec2d::Zero();
    double theta_ = 0.0;
    double bg_ = 0.0;
    Vec2d ba_ = Vec2d::Zero();

    // 误差状态（每轮更新结束后清零）
    Vec8d dx_ = Vec8d::Zero();

    // 误差协方差
    Mat8d P_ = Mat8d::Identity();

    double last_imu_time_ = 0.0;
    bool inited_ = false;
};

}  // namespace sad

#endif  // IESKF_2D_HPP_
