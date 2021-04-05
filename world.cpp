//
// Created by admin on 2021/4/5.
//

#include "world.h"

//container

// update vertex position 
void World::step(int type) {
    switch (type)
    {
        case 0:
            rope->simulateEuler(dt, gravity);
            break;
        case 1:
            rope->simulateVerlet(dt, gravity);
            break;
        case 2:
            rope->simulatePrime(dt, gravity, objs);
            break;
        case 3:
            rope->simulateDual(dt, gravity, objs);
            break;
        default:
            break;
    }
}

// set delta t for updating
void World::set_dt(double time_step) {
    dt = time_step;
}
const double World::get_dt() const {
    return dt;
}

// set gravity
const Vec3d& World::get_gravity() const {
    return gravity;
}

