//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_WORLD_H
#define REBUILD_AT_CLION_WORLD_H

#include "rope.h"
#include "other_obj.h"

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
                r->stream_out();
            }
            ropes[0]->output_end();
        } else {
            rope->output_begin(index);
            rope->stream_out();
            rope->output_end();
        }

    }

    vector<Rope*>& get_all_ropes() {
        return ropes;
    }

    void add_obj(sphere* obj) { objs.push_back(obj); }

    vector<sphere*>& get_objs() { return objs; };
};

#endif //REBUILD_AT_CLION_WORLD_H
