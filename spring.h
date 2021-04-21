//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_SPRING_H
#define REBUILD_AT_CLION_SPRING_H

#include "mass.h"
#include "other_obj.h"

struct Spring {
    Spring(Mass* m1, Mass* m2, float k)
            :m1(m1), m2(m2), k(k), k_r(1.0 / k), rest_length((m1->position - m2->position).norm()) {};

    //k stand for stiffness
    float k;
    double k_r;
    double rest_length;

    Mass* m1;
    Mass* m2;

    double lambda = 0;
};

struct constrain {

    constrain(Mass* mass, sphere* obj_in)
            :m(mass), obj(obj_in), lambda(0) {};

    sphere* obj;

    double lambda;
    double lambda_f_1;
    double lambda_f_2;

    Mass* m;


};

#endif //REBUILD_AT_CLION_SPRING_H
