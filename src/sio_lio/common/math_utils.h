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
    }
}
#endif