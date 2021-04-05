//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_ROPE_H
#define REBUILD_AT_CLION_ROPE_H

#pragma once
#ifndef ROPE_H
#define ROpe_H

#include <vector>

#include "vec3d.h"
#include "mass.h"
#include "spring.h"
#include <string>
#include "output.h"
#include "other_obj.h"

using namespace std;

const int p_exp = 2;


class Rope {
private:
    vector<Mass*> masses;
    vector<Spring*> springs;
    vector<Spring*> visual_spr;
    vector<constrain*> constrains;

    static Files_in_obj f;

    //for each spring, calculate the force by hook's law and save it in m.forces
    void cal_force();
    //for each spring, calculate the Jcobian in every degree of freedom  and save it in m.P
    void cal_prim_Jaco(double dt);
    void init_v_p(double dt, Vec3d gravity);
    void contact_penalty(double d_t);
    void contact_dual(double d_t, Vec3d gravity);
    void make_constrains(vector<sphere*>& objs);
    void clear_constrains();

public:
    Rope(vector<Mass*>& masses, vector<Spring*>& springs)
            :masses(masses), springs(springs) {};
    Rope(vector<Mass*>& masses, vector<Spring*>& springs, vector<Spring*>& v_spr)
            :masses(masses), springs(springs), visual_spr(v_spr) {};
    Rope(Vec3d start, Vec3d end, int node_num, float node_mass,
         float k, vector<int> pinned_nodes);
    ~Rope() {
        for (auto i:springs)
            if (NULL != i)
            {
                delete i;
                i = NULL;
            }
        springs.clear();
        for (auto i : masses)
            if (NULL != i)
            {
                delete i;
                i = NULL;
            }
        springs.clear();
        for (auto i : visual_spr)
            if (NULL != i)
            {
                delete i;
                i = NULL;
            }
        springs.clear();
    }
    //iterate methord
    void simulateVerlet(float delta_t, Vec3d gravity);
    void simulateEuler(float delta_t, Vec3d gravity);
    void simulatePrime(float delta_t, Vec3d gravity, vector<sphere*>& objs);
    void simulateDual(float delta_t, Vec3d gravity, vector<sphere*>& objs);

    vector<Mass*>& get_masses() { return masses; }
    vector<Spring*>& get_springs() { return springs; }
    void output_2_obj(int step_index);
    void output_begin(int step_index) { f.make_New_file(step_index); }
    void stream_out() {
        for (auto& m : masses) f.add_point(m->position);
        for (auto& s : visual_spr) f.add_edge(s->m1->id, s->m2->id);
    }
    void output_end() { f.close_f(); }
    const vector<Spring*> get_vs() const { return visual_spr; }
};
#endif // !ROPE_H


#endif //REBUILD_AT_CLION_ROPE_H
