//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_MASS_H
#define REBUILD_AT_CLION_MASS_H

#include "vec3d.h"

struct Mass {
    Mass(int id, Vec3d position, float mass, bool pinned)
            : id(id), start_position(position), position(position), last_position(position),
              mass(mass), pinned(pinned), velocity(Vec3d(0, 0, 0)), forces(Vec3d(0, 0, 0)),
              v_predict(Vec3d(0, 0, 0))
    {}

    int id;
    float mass;

    //pinned == true means this vertex cannot be moved
    bool pinned;


    Vec3d position;
    Vec3d start_position;
    Vec3d last_position;
    //u+
    Vec3d velocity;

    //interal force
    Vec3d forces;
    //Lambda
    double Lmabda;

    //save the diag of Jacobian
    Vec3d P = Vec3d(0, 0, 0);
    //save every gradient
    Vec3d v_predict;

};
#endif //REBUILD_AT_CLION_MASS_H
