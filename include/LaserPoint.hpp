//
// Created by akira on 18-7-20.
//

#ifndef DSVSEGREGION_CPP_LASERPOINT_HPP
#define DSVSEGREGION_CPP_LASERPOINT_HPP

#include "cmath"
class LaserPoint{
public:
    LaserPoint(){x = y = z = intensity = 0.0;}
    LaserPoint(float x, float y, float z, unsigned int i){
        this->x = x;
        this->y = y;
        this->z = z;
        this->intensity = i;
            }
    float x;
    float y;
    float z;
    unsigned char intensity;
    float calXYZNorm()const{return sqrt(x * x + y * y + z * z);}
    float calXYNorm()const{return sqrt(x * x + y * y);}
    static inline double calDis(LaserPoint a, LaserPoint b){
        return sqrt(
                (a.x - b.x) * (a.x - b.x)+
                (a.y - b.y) * (a.y - b.y)+
                (a.z - b.z) * (a.z - b.z)
        );
    };
};

#endif //DSVSEGREGION_CPP_LASERPOINT_HPP
