#ifndef NDT_2D_HPP_
#define NDT_2D_HPP_

#include <cmath>
#include <unordered_map>
#include <vector>
#include "sio_lio/common/eigen_types.h"


// 2D NDT 算法

namespace sad{
    // 2D体素数据容器
    class VoxelGrid{
        public:
        VoxelGrid()=default;
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
            int max_capacity_ = 10000;
            int max_iteration_ = 50;
            double eps_ = 1e-3;
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

        
        private:

        Options options_;
        bool first_scan_ = true;   // 系统是否处于第一帧（add_scan 结束后置 false）
        std::unordered_map<Vec2i,VoxelGrid,hash_vec<2>> grids_;

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