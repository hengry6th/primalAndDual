//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_WORLD_H
#define REBUILD_AT_CLION_WORLD_H

#include "rope.h"
#include "other_obj.h"
#include "output.h"

enum step_types {Euler, Verlet, Prime, Dual};

class World {
private:
    // delta t, internal of two steps
    double dt;
    Vec3d gravity;
    Rope* rope;
    vector<Rope*> ropes;
    vector<int> types;
    vector<sphere*> objs;
    Files_in_obj f;

    double primal_condi_num = 0;
    double dual_condi_num = 0;

public:
    //defalt dt = 0.025 or 40fps;
    World():dt(0.025), gravity(Vec3d(0, 0, 0)){}
    World(Rope* rope) : rope(rope){}
    ~World() {
        delete rope;
        for (auto r : ropes) {
            if (r != NULL) delete r;
            r = NULL;
        }
        ropes.clear();
        for (auto r : objs) {
            if (r != NULL) {
                delete r;
                r = NULL;
            }
        }
        objs.clear();
    }
    void set_rope(Rope* rope_in) { rope = rope_in; }
    void step(int type);
    void set_dt(double time_step);
    void set_g(Vec3d g) { gravity = g; }

    void add_rope(Rope* rope, step_types type) {
        ropes.push_back(rope);
        types.push_back(type);
    }

    void step_all() {
        for (int i = 0; i < ropes.size(); i++) {
            switch (types[i])
            {
                case step_types::Euler:
                    ropes[i]->simulateEuler(dt, gravity);
                    break;
                case step_types::Verlet:
                    ropes[i]->simulateVerlet(dt, gravity);
                    break;
                case step_types::Prime:
                    ropes[i]->simulatePrime(dt, gravity, objs);
                    break;
                case step_types::Dual:
                    ropes[i]->simulateDual(dt, gravity, objs);
                    break;
                default:
                    break;
            }
        }
    }
    const double get_dt() const;
    const Vec3d& get_gravity() const;
    void output_2_obj(int index) {
        rope->output_2_obj(index);
    }

    void output_all(int index) {
        if (!ropes.empty()) {
            ropes[0]->output_begin(index);
            for (auto& r : ropes) {
                r->stream_out(objs);
            }
            ropes[0]->output_end();
        } else {
            rope->output_begin(index);
            rope->stream_out(objs);
            rope->output_end();
        }
    }

    vector<Rope*>& get_all_ropes() {
        return ropes;
    }

    void add_obj(sphere* obj) { objs.push_back(obj); }

    vector<sphere*>& get_objs() { return objs; };

    void start_print_rope_ratio(string type) {
        string f_name1 = "dual_" + type;
        string f_name2 = "primal_" + type;
        f.open_f(f_name1, f_name2);
    }

    void reset_condi_num() {
        primal_condi_num = 0;
        dual_condi_num = 0;
    }

    void count_rope_ratio() {
        primal_condi_num += ropes[1]->condiction_num;
        dual_condi_num += ropes[0]->condiction_num;
    }

    void print_rope_ratio(int step_count,int ratio_type) {
        if (ratio_type == 0) {
            f.print_ratio_and_condiction_num(ropes[0]->m_ratio, dual_condi_num/step_count, ropes[1]->m_ratio, primal_condi_num/step_count);
        } else {
            f.print_ratio_and_condiction_num(ropes[0]->k_ratio, dual_condi_num/step_count, ropes[1]->k_ratio, primal_condi_num/step_count);
        }
    }

    void end_print_rope_ratio() {
        f.close_f_all();
    }

    void clear_ropes() {
        for(auto r : ropes) {
            delete r;
        }
        ropes.clear();
        types.clear();
    }
};

#endif //REBUILD_AT_CLION_WORLD_H
