#include "sio_lio/src/ndt_2d.hpp"
#include "sio_lio/common/math_utils.h"
#include <set>
#include <limits>

namespace sad{
    std::vector<Vec2i> NDT2D::generate_nearby_grids(const Vec2i& grid_id){
        std::vector<Vec2i> nearby_grids;
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                nearby_grids.push_back(grid_id + Vec2i(i, j));
            }
        }
        return nearby_grids;
    }

    bool VoxelGrid::addPoint(const Eigen::Vector2d& point){
        points_set_.push_back(point);
        return true;
    }

    void NDT2D::update_voxel(VoxelGrid& voxel, bool first_scan) {
        (void)first_scan;   // 本参数在简化实现中无用途，保留以对照 NdtLIO 签名

        // 已生效且累计点数超上限
        if (voxel.ndt_estimated_ && voxel.point_num_ > options_.max_pts_num_) {
            return;
        }

        // 暂存点不足
        if (voxel.points_set_.size() < (size_t)options_.min_pts_num_) {
            return;
        }

        // 对暂存点计算均值与协方差（矩阵版，保留轴间耦合）
        Vec2d cur_mean = Vec2d::Zero();
        Mat2d cur_cov = Mat2d::Zero();
        math::ComputeMeanAndCov(voxel.points_set_, cur_mean, cur_cov,
                                [](const Vec2d& p) { return p; });

        if (!voxel.ndt_estimated_) {
            // 首次生效
            voxel.mean_ = cur_mean;
            voxel.sigma_ = cur_cov;
            voxel.point_num_ = (int)voxel.points_set_.size();
            voxel.ndt_estimated_ = true;
        } else {
            // 增量合并
            Vec2d new_mean = Vec2d::Zero();
            Mat2d new_cov = Mat2d::Zero();
            math::UpdateMeanAndCov(voxel.point_num_, (int)voxel.points_set_.size(),
                                   voxel.mean_, voxel.sigma_, cur_mean, cur_cov,
                                   new_mean, new_cov);
            voxel.mean_ = new_mean;
            voxel.sigma_ = new_cov;
            voxel.point_num_ += (int)voxel.points_set_.size();
        }
        voxel.points_set_.clear();

        if (options_.use_svd_inverse_) {
            // 信息矩阵 Ω = Σ⁻¹，SVD 稳健求逆：小奇异值钳到最大值的 1e-3，
            // 防止共线点（如走廊墙面）导致 Σ 病态、求逆发散（对照 NdtLIO ndt_inc.cpp:365-371）
            Eigen::JacobiSVD<Mat2d> svd(voxel.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec2d lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
                lambda[1] = lambda[0] * 1e-3;
            }
            Mat2d inv_lambda = Vec2d(1.0 / lambda[0], 1.0 / lambda[1]).asDiagonal();
            voxel.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
        } else {
            // 旧实现（仅对照实验用）：Σ + λI 正则化求逆
            voxel.info_ = (voxel.sigma_ + Mat2d::Identity() * 1e-3).inverse();
        }
    }

    void NDT2D::add_scan(const SE2& pose, const std::vector<Eigen::Vector2d>& scan) {
        // 本帧涉及的格子编号集合（set 自动去重：一帧多个点常落入同一格子）
        // less_vec<2>：Vec2i 缺少 < 运算符，用 eigen_types.h 提供的比较器补齐排序能力
        std::set<Vec2i, less_vec<2>> touched_keys;

        // 遍历本帧每个点：变换到世界系 → 计算格子编号 → 写入格子
        for (const auto& p : scan) {
            // SE2 作用于二维点：旋转 + 平移，得到世界系坐标
            Vec2d pw = pose * p;

            // 格子编号 = 点坐标除以格子边长后向下取整（floor 把 -0.3 归入格子 -1，
            // 保证边界两侧的点分属正确的相邻格子）
            int gx = (int)std::floor(pw.x() / options_.voxel_size_);
            int gy = (int)std::floor(pw.y() / options_.voxel_size_);
            Vec2i key(gx, gy);

            // 沿用原版 addScan 的 LRU 写入方式：
            // 键不存在 → 新体素插入链表头部并登记迭代器；
            // 键已存在 → 追加点，并把该节点 splice 到头部（表示最近使用）
            auto iter = grids_.find(key);
            if (iter == grids_.end()) {
                data_.push_front({key, VoxelGrid(pw)});
                grids_.insert({key, data_.begin()});
            } else {
                iter->second->second.addPoint(pw);
                data_.splice(data_.begin(), data_, iter->second);
                iter->second = data_.begin();   // splice 后更新迭代器指向新位置
            }
            touched_keys.insert(key);
        }

        // 对本帧涉及的每个格子执行生命周期更新（首次生效 / 增量合并 / 冻结跳过）
        for (const auto& key : touched_keys) {
            update_voxel(grids_[key]->second, first_scan_);
        }

        // 体素容量上限：超过 max_capacity_ 时淘汰最久未更新的体素（原版 capacity_ 机制）
        evict_if_full();

        // 第一帧结束：此后不再处于 first_scan 状态（对照 NdtLIO 的 addScan 行为）
        first_scan_ = false;
    }

    void NDT2D::evict_if_full() {
        while (data_.size() >= (size_t)options_.max_capacity_) {
            grids_.erase(data_.back().first);   // 删除最旧体素的索引（LRU 策略）
            data_.pop_back();                   // 删除链表尾部数据
        }
    }

    int NDT2D::match(SE2& pose, const std::vector<Eigen::Vector2d>& scan) {
        int effective_pts = 0;

        // 高斯-牛顿主循环（第三讲第十一节伪代码的逐行实现）
        for (int loop = 0; loop < options_.max_iteration_; ++loop) {
            Mat3d H = Mat3d::Zero();
            Vec3d g = Vec3d::Zero();
            effective_pts = 0;

            // 遍历当前帧每个点：变换 → 9 邻域归属 → 残差与雅可比 → 累加
            for (const auto& p : scan) {
                Vec2d p0 = pose * p;
                int gx = (int)std::floor(p0.x() / options_.voxel_size_);
                int gy = (int)std::floor(p0.y() / options_.voxel_size_);

                // 归属判定：在自身与周围 8 格中选马氏距离最小的已生效格子
                VoxelGrid* best = nullptr;
                double best_maha2 = std::numeric_limits<double>::max();
                for (const auto& key : generate_nearby_grids(Vec2i(gx, gy))) {
                    auto iter_grid = grids_.find(key);
                    if (iter_grid == grids_.end()) continue;             // 该方向无格子
                    VoxelGrid& vg = iter_grid->second->second;           // 迭代器 → 链表节点 → 体素
                    if (!vg.estimate()) continue;                        // 格子未生效
                    Vec2d r = p0 - vg.mean_;
                    double maha2 = r.dot(vg.info_ * r);
                    if (maha2 < best_maha2) {
                        best_maha2 = maha2;
                        best = &vg;
                    }
                }
                if (best == nullptr) continue;   // 周围无任何有效格子

                // 卡方检验离群剔除（对照原版 res_outlier_th_，2 自由度 95% 置信）：
                // 马氏距离为 nan 或超限时，该点不参与本次迭代——避免外点拉偏高斯-牛顿方向
                // （nan 检查同样取自原版 ndt_inc.cpp:156/279，防御协方差病态产生的非数值）
                if (std::isnan(best_maha2) || best_maha2 > options_.res_outlier_th_) {
                    continue;
                }

                // 残差与雅可比（第二讲第十节）
                Vec2d e = p0 - best->mean_;
                Eigen::Matrix<double, 2, 3> J;
                J << 1.0, 0.0, -p0.y(),
                     0.0, 1.0,  p0.x();

                // 累加 H += JᵀΩJ，g += JᵀΩe
                H += J.transpose() * best->info_ * J;
                g += J.transpose() * best->info_ * e;
                effective_pts++;
            }

            // 有效点不足：配准失败，返回当前计数
            if (effective_pts < options_.min_pts_num_) {
                return effective_pts;
            }

            // 解正规方程 H·Δξ = −g（H 对称正定，LDLT 分解）
            Vec3d dxi = H.ldlt().solve(-g);

            // 位姿更新：Δξ = (v_x, v_y, θ)，构造增量位姿后左乘（李代数 → 李群）
            pose = SE2(SO2(dxi[2]), Vec2d(dxi[0], dxi[1])) * pose;

            // 收敛判定：增量小于阈值即停止
            if (dxi.norm() < options_.eps_) {
                break;
            }
        }
        return effective_pts;
    }
}