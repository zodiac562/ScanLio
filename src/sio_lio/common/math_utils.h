#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <numeric>
#include <cassert>
#include "sio_lio/common/eigen_types.h"

namespace sad{
    namespace math{
        template<typename Container,typename Data,typename Getter>
        void ComputeMeanAndCovDiag(const Container& container,Data& mean,Data& cov_diag,Getter&& getter){
            size_t len = container.size();
            assert(len > 1);

            // 注意：返回类型标注 -> Data 不可省略
            // sum + getter(item) 的原生类型是 Eigen 表达式（持有对 sum 的引用），
            // 省略标注时 lambda 返回的是未求值表达式，循环中表达式内部引用的临时对象销毁，产生悬空引用，结果错乱（已实测验证）
            mean = std::accumulate(container.begin(),container.end(),Data::Zero().eval(),
            [&getter](const Data& sum,const auto& item) -> Data{
                return sum + getter(item);
            }) / len;

            cov_diag = std::accumulate(container.begin(),container.end(),Data::Zero().eval(),
            [&getter,&mean](const Data& sum,const auto& item) -> Data{
                return sum + (getter(item) - mean).cwiseAbs2().eval();
            }) / (len - 1);
        }

        // 矩阵版均值与协方差：保留轴间耦合项（对角版丢弃），NDT 体素统计需要
        template<typename Container,int dim,typename Getter>
        void ComputeMeanAndCov(const Container& container,
                               Eigen::Matrix<double,dim,1>& mean,
                               Eigen::Matrix<double,dim,dim>& cov,
                               Getter&& getter){
            size_t len = container.size();
            assert(len > 1);

            using Vec = Eigen::Matrix<double,dim,1>;
            using Mat = Eigen::Matrix<double,dim,dim>;

            mean = std::accumulate(container.begin(),container.end(),Vec::Zero().eval(),
            [&getter](const Vec& sum,const auto& item) -> Vec{
                return sum + getter(item);
            }) / len;

            cov = std::accumulate(container.begin(),container.end(),Mat::Zero().eval(),
            [&getter,&mean](const Mat& sum,const auto& item) -> Mat{
                Vec d = (getter(item) - mean).eval();
                return sum + d * d.transpose();
            }) / (len - 1);
        }

        // 增量合并两组统计：旧统计（hist_m 个点）与新批统计（curr_n 个点）→ 合并统计
        template<typename S,int dim>
        void UpdateMeanAndCov(int hist_m,int curr_n,
                              const Eigen::Matrix<S,dim,1>& hist_mean,
                              const Eigen::Matrix<S,dim,dim>& hist_var,
                              const Eigen::Matrix<S,dim,1>& curr_mean,
                              const Eigen::Matrix<S,dim,dim>& curr_var,
                              Eigen::Matrix<S,dim,1>& new_mean,
                              Eigen::Matrix<S,dim,dim>& new_var){
            int total = hist_m + curr_n;
            new_mean = (hist_m * hist_mean + curr_n * curr_mean) / total;
            Eigen::Matrix<S,dim,1> d_hist = hist_mean - new_mean;
            Eigen::Matrix<S,dim,1> d_curr = curr_mean - new_mean;
            new_var = (hist_m * (hist_var + d_hist * d_hist.transpose()) +
                       curr_n * (curr_var + d_curr * d_curr.transpose())) / total;
        }
    }
}
#endif