//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_OTHER_OBJECT_H
#define REBUILD_AT_CLION_OTHER_OBJECT_H

#include "vec3d.h"
#include "mass.h"

struct sphere {
    Mass center;
    double radius;
    double k;
    double mu;
    sphere(Mass c, double r, double k_in, double u)
            : center(c), radius(r * 1.05), k(k_in), mu(u) {};
    bool is_in(Vec3d m_p) const { return (m_p - center.position).norm() < radius; }
};
#endif //REBUILD_AT_CLION_OTHER_OBJECT_H
