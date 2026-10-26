#ifndef ODOM_H
#define ODOM_H  

namespace sad{
    struct Odom{
        Odom()=default;
        Odom(double timestamp,double vel)
        : timestamp_(timestamp),vel_(vel){}

        double timestamp_ = 0.0;
        double vel_ = 0.0;
    
    };

}

#endif