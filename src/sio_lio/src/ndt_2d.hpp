#ifndef NDT_2D_HPP_
#define NDT_2D_HPP_

#include <cmath>
#include <list>
#include <unordered_map>
#include <vector>
#include "sio_lio/common/eigen_types.h"


// 2D NDT 算法

namespace sad{
    // 2D体素数据容器
    class VoxelGrid{
        public:
        VoxelGrid()=default;
        // 沿用原版 NdtInc2d::Voxel 的构造：直接用首点初始化容器
        explicit VoxelGrid(const Eigen::Vector2d& pt){
            points_set_.emplace_back(pt);
        }
        ~VoxelGrid()=default;

        std::vector<Eigen::Vector2d> points_set_;
        Eigen::Vector2d mean_ = Eigen::Vector2d::Zero();
        Eigen::Matrix2d sigma_ = Eigen::Matrix2d::Zero();
        Eigen::Matrix2d info_ = Eigen::Matrix2d::Zero();
        int point_num_ = 0;
        bool ndt_estimated_ = false;

        bool addPoint(const Eigen::Vector2d& point);
        bool estimate() const {return ndt_estimated_;}
        
    };

    // 体素管理与NDT配准
    class NDT2D{
        public:
        struct Options{
            double voxel_size_ = 0.65;
            int min_pts_num_ = 3;         // N10 雷达点密度核算后下调（原 10 导致 68 格仅 14 格生效）
            int max_pts_num_ = 50;
            int max_capacity_ = 10000;    // 体素容量上限（LRU：超限淘汰最久未更新的体素）
            int max_iteration_ = 50;
            double eps_ = 1e-3;
            double res_outlier_th_ = 5.991;  // 卡方检验阈值（2 自由度 95% 置信），马氏距离超限剔除
            // 求逆方式开关（2026-10-18 四配置对照实测后定为 false）：
            //   true  = 原版 NdtLIO 的 SVD 稳健求逆（与 ndt_inc.cpp:364-371 逐行一致）
            //   false = 旧正则化求逆 (Σ+λI)^-1
            // 实测结论（6 数据集 A/B/C/D 对照）：SVD 使转弯场景（circle/circle2/corridor）
            // 跳变从 0 增至 6~16、平均位移上升（circle 0.0123→0.0287），有效点数不变；
            // 叠加卡方剔除后跳变归零，但点数比「旧求逆+卡方」少 6~10%（D 严格优于 C）。
            // 故默认走旧正则化；SVD 分支保留供真机数据再评估。
            bool use_svd_inverse_ = false;
        };

        NDT2D()=default;
        explicit NDT2D(const Options& options):options_(options){}
        ~NDT2D()=default;

        /*
        写入已变换至世界系的激光数据至体素网格
        触发体素的高斯参数更新
        */
        void add_scan(const SE2& pose, const std::vector<Eigen::Vector2d>& scan);

        /*
        配准函数
        输入初始位姿，激光数据
        输出配准后位姿，有效匹配点数
        */
        int match(SE2& pose,const std::vector<Eigen::Vector2d>& scan);

        // 只读观测接口：当前体素数量（验证 LRU 淘汰与容量守卫用）
        size_t num_voxels() const { return data_.size(); }
        size_t num_indexed() const { return grids_.size(); }

        
        private:

        Options options_;
        bool first_scan_ = true;   // 系统是否处于第一帧（add_scan 结束后置 false）

        // LRU 存储（沿用原版 NdtInc2d 结构）：
        // data_ 链表按使用新旧排序，头部=最近更新，尾部=最久未更新
        // grids_ 只存「体素键 → 链表迭代器」的映射，不直接存体素数据
        using KeyAndVoxel = std::pair<Vec2i, VoxelGrid>;
        std::list<KeyAndVoxel> data_;
        std::unordered_map<Vec2i, std::list<KeyAndVoxel>::iterator, hash_vec<2>> grids_;

        /*
        LRU 淘汰：体素数量超过 max_capacity_ 时，从 data_ 尾部
        删除最久未更新的体素（对照原版 NdtInc2d addScan 的 capacity_ 机制）
        */
        void evict_if_full();

        /*
        更新单位体素的生命周期与高斯参数
        */
        void update_voxel(VoxelGrid& voxel, bool first_scan);

        /*
        生成体素网格id的邻域
        返回邻域网格id的集合
        */
        std::vector<Vec2i> generate_nearby_grids(const Vec2i& grid_id);
    };

}

#endif