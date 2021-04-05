//
// Created by admin on 2021/4/5.
//
#include <new>
#include <vector>
#include "mass.h"
#include "spring.h"
#include "rope.h"
#include "output.h"
#include "vec3d.h"

#define kd 0.00005
#define alpha 0.1
#define MAX_ITERATE 10

const double kf = 1;

using namespace std;
//using namespace Eigen;

Rope::Rope(Vec3d start, Vec3d end, int node_num, float node_mass,
           float k, vector<int> pinned_nodes) {
    //initialize 
    masses = vector<Mass*>(node_num);
    springs = vector<Spring*>(node_num - 1);
    //calculate each spring's rest_length
    Vec3d length = (end - start) / (node_num - 1);
    //initialize the inite point
    masses[0] = new Mass(0, start, node_mass, false);
    masses[0]->P = Vec3d(0, 0, 0);
    for (int i = 1; i < node_num; i++) {
        //update the new mass vertex position
        start += length;
        //create new mass vertex
        masses[i] = new Mass(i, start, node_mass, false);
        masses[i]->P = Vec3d(0, 0, 0);
        //create new spring by mass_i and mass_(i-1) 
        springs[i - 1] = new Spring(masses[i - 1], masses[i], k);
    }
    for (auto& i : pinned_nodes) {
        masses[i]->pinned = true;
    }
}

void Rope::simulateEuler(float delta_t, Vec3d gravity) {
    cal_force();
    for (auto& m : masses)
    {
        if (!m->pinned)
        {
            // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
            // save last position;
            m->last_position = m->position;
            // update position with damp;
            Vec3d a = m->forces / m->mass + gravity;
            m->velocity += a * delta_t;
            m->velocity *= (1 - kd);
            m->position += m->velocity * delta_t;//+ 0.5 * a * delta_t * delta_t;


        }
        // Reset all forces on each mass
        m->forces = Vec3d(0, 0, 0);
    }
}
void Rope::simulateVerlet(float delta_t, Vec3d gravity) {
    for (auto& s : springs)
    {
        // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        Mass* m1 = s->m1;
        Mass* m2 = s->m2;
        Vec3d l = m1->position - m2->position;
        double constraint = l.norm() - s->rest_length;
        Vec3d delt_s = constraint * l / l.norm();
        if (!m1->pinned) s->m1->position -= delt_s * m2->mass / (m1->mass + m2->mass);
        if (!m2->pinned) s->m2->position += delt_s * m1->mass / (m1->mass + m2->mass);
    }

    for (auto& m : masses)
    {
        if (!m->pinned)
        {
            Vec3d temp_position = m->position;
            // TODO (Part 3.1): Set the new position of the rope mass
            // update position with damp
            m->position = m->position + (1 - kd) * (m->position - m->last_position) + gravity * delta_t * delta_t;
            m->last_position = temp_position;
            // TODO (Part 4): Add global Verlet damping
        }
    }
}

void Rope::make_constrains(vector<sphere*>& objs) {
    for (Mass* m : masses) {
        for (auto obj : objs) {
            if (obj->is_in(m->position)) {
                constrains.push_back(new constrain(m, obj));
            }
        }
    }
}

void Rope::clear_constrains() {
    for (auto c : constrains) {
        if (c != NULL) {
            delete c;
            c = NULL;
        }
    }
    constrains.clear();
}



void Rope::contact_penalty(double d_t) {
    // perform collison detection and calculate forces

    for (int i = 0; i < MAX_ITERATE; i++) {
        // initialize m
        for (auto m : masses) {
            m->forces = Vec3d(0, 0, 0);
            m->P = Vec3d(0, 0, 0);
            m->Lmabda = 0;
        }

        // Evaluate forces and derivatives;
        for (auto c : constrains) {
            Mass* m = c->m;
            auto obj = c->obj;
            Vec3d n = m->position - obj->center;
            Vec3d tengent = -1 * n;
            if (n.z != 0) {
                tengent.z += n.squaredNorm() / n.z;
            } else{
                tengent = Vec3d(0, 0, 1.0);
            }
            Vec3d tengent2 =  n.cross(tengent);

            tengent /= tengent.norm();
            tengent2 /= tengent2.norm();

            const Vec3d* v = &(m->velocity);

            double u1 = tengent.dot(m->velocity);
            double u2 = tengent2.dot(m->velocity);

            double n_norm = n.norm();
            double ci = n_norm - obj->radius;
            m->forces -= n / n_norm * obj->k * min(0.0, ci);

            double squrt_u = sqrt(u1 * u1 + u2 * u2);
            double uf = obj->mu * m->forces.norm() ;
            m->forces -= min(kf, uf/squrt_u) * (u1 * tengent + u2 * tengent2);
            m->Lmabda = kf * squrt_u < uf ? kf : uf / squrt_u;

            double x2 = n.x * n.x / n_norm;
            double y2 = n.y * n.y / n_norm;
            double z2 = n.z * n.z / n_norm;
            m->P += d_t * obj->k * Vec3d(x2, y2, z2);
        }

        // the same as in simulate
        for (auto m : masses) {
            m->P.x = 1 / (m->mass + d_t * m->P.x);
            m->P.y = 1 / (m->mass + d_t * m->P.y);
            m->P.z = 1 / (m->mass + d_t * m->P.z);
            //calculate gradient
            Vec3d d = m->mass * (m->velocity - m->v_predict) - d_t * m->forces;
            //update m's state
            m->velocity -= alpha * Vec3d(d.x * m->P.x, d.y * m->P.y, d.z * m->P.z);
            m->velocity -= alpha * m->Lmabda * d;
            //not rigid body, G = E
            m->position = m->last_position + d_t * m->velocity;
        }
    }

    //release costraints
    clear_constrains();
}

void Rope::simulatePrime(float delta_t, Vec3d gravity, vector<sphere*>& objs) {
    // update
    //
    init_v_p(delta_t, gravity);
    for (int i = 0; i < MAX_ITERATE; i++) {
        for (auto& m : masses) {
            // clear m's forces and jacobian
            m->forces = Vec3d(0, 0, 0);
            m->P = Vec3d(0, 0, 0);
        }
        //Evaluate forces and derivatives;
        cal_force();
        cal_prim_Jaco(delta_t);
        for (auto& m : masses) {
            if (!m->pinned) {
                //build predictioner PD
                m->P.x = 1 / (m->mass + delta_t * m->P.x);
                m->P.y = 1 / (m->mass + delta_t * m->P.y);
                m->P.z = 1 / (m->mass + delta_t * m->P.z);
                //calculate gradient
                Vec3d d = m->mass * (m->velocity - m->v_predict) - delta_t * m->forces;
                //update m's state
                m->velocity -= alpha * Vec3d(d.x * m->P.x, d.y * m->P.y, d.z * m->P.z);
                //not rigid body, G = E
                m->position = m->last_position + delta_t * m->velocity;
            }
        }
    }
    //Perform collision detection;
    make_constrains(objs);
    if (!constrains.empty()) {
        contact_penalty(delta_t);
    }
}

void Rope::contact_dual(double d_t, Vec3d gravity) {
    // perform collison detection and calculate forces

    for (int i = 0; i < MAX_ITERATE; i++) {
        for (auto c : constrains) {
            auto m = c->m;
            auto obj = c->obj;
            Vec3d n = m->position - obj->center;
            double ci = n.norm() - obj->radius;
            //calculate hi
            double hi = -1 * ci - c->lambda / obj->k;
            //calculate P^D_ii
            double pi = 1 / (d_t * d_t / m->mass + 1 / obj->k);
            // update lambda
            auto d_lam = alpha * pi * hi;
            //calculate lambda
            c->lambda += d_lam;
            //calculate friction
            Vec3d tengent = -1 * n;
            if (n.z != 0) {
                tengent.z += n.squaredNorm() / n.z;
            }
            else {
                tengent = Vec3d(0, 0, 1.0);
            }
            Vec3d tengent2 = n.cross(tengent);

            Vec3d norm_n = n / n.norm();
            tengent /= tengent.norm();
            tengent2 /= tengent2.norm();

            double Rx = m->mass / (pow(norm_n.x, 2) + pow(tengent.x, 2) + pow(tengent2.x, 2));
            double Ry = m->mass / (pow(norm_n.y, 2) + pow(tengent.y, 2) + pow(tengent2.y, 2));
            double Rz = m->mass / (pow(norm_n.z, 2) + pow(tengent.z, 2) + pow(tengent2.z, 2));

            double Axy = -1  * (n.x * n.y + tengent.x * tengent.y + tengent2.x * tengent2.y) * c->lambda;
            double Axz = -1 * Ry * (n.x * n.z + tengent.x * tengent.z + tengent2.x * tengent2.z) * c->lambda;
            double Ayz = -1 * Rz * (n.y * n.z + tengent.y * tengent.z + tengent2.y * tengent2.z) * c->lambda;

            double Ax = (Axy * n.y + Axz * n.z) * Rx * 2;
            double Ay = (Axy * n.x + Ayz * n.z) * Ry * 2;
            double Az = (Axz * n.x + Ayz * n.y) * Rz * 2;

            Vec3d Jx = Vec3d(norm_n.x, tengent.x, tengent2.x);
            Vec3d Jy = Vec3d(norm_n.y, tengent.y, tengent2.y);
            Vec3d Jz = Vec3d(norm_n.z, tengent.z, tengent2.z);

            double Jux = 1.5 * Rx * Jx.dot(m->velocity);
            double Juy = 1.5 * Ry * Jy.dot(m->velocity);
            double Juz = 1.5 * Rz * Jy.dot(m->velocity);

            double tJMhx = Rx * d_t * Jx.dot(gravity) / m->mass;
            double tJMhy = Ry * d_t * Jy.dot(gravity) / m->mass;
            double tJMhz = Rz * d_t * Jz.dot(gravity) / m->mass;

            double zx = Ax - Jux - tJMhx;
            double zy = Ay - Juy - tJMhy;
            double zz = Az - Juz - tJMhz;

            Vec3d lambda_f;

            double norm2 = zx * zx + zy * zy + zz * zz;
            double mu_lambda = obj->mu * c->lambda * n.norm() * 2;
            lambda_f = Vec3d(zx, zy, zz);

            if (norm2 > mu_lambda * mu_lambda) {
                double norm = sqrt(norm2);
                lambda_f = mu_lambda * lambda_f / norm;
            }

            Vec3d delat_l_f = lambda_f - c->lambda_f;
            c->lambda_f = lambda_f;

            // evaluate jacobian
            m->velocity += d_t / m->mass * (d_lam * 2 * n + delat_l_f);
            m->position = m->last_position + d_t * m->velocity;
        }
    }
    // release costrains
    constrains.clear();
}

void Rope::simulateDual(float delta_t, Vec3d gravity, vector<sphere*>& objs) {

    init_v_p(delta_t, gravity);
    //initialize lambda
    for (auto& s : springs) {
        s->lambda = 0;
    }
    for (int i = 0; i < MAX_ITERATE; i++) {
        //Evaluate constraints and derivatives
        for (auto& s : springs) {
            auto& m1 = s->m1;
            auto& m2 = s->m2;
            //calculate hi
            double hi = s->rest_length - (m1->position - m2->position).norm() - s->lambda / s->k;
            //calculate P^D_ii
            double pi = 1 / (double(delta_t) * delta_t * (1.0 / m1->mass + 1.0 / m2->mass) + s->k_r);
            // update lambda
            auto d_lam = alpha * pi * hi;
            s->lambda += d_lam;
            // evaluate jacobian
            Vec3d j = m1->position - m2->position;
            j /= j.norm();
            if (!m1->pinned) {
                m1->velocity += delta_t / m1->mass * d_lam * j;
                m1->position = m1->last_position + delta_t * m1->velocity;
            }
            if (!m2->pinned) {
                m2->velocity -= delta_t / m2->mass * d_lam * j;
                m2->position = m2->last_position + delta_t * m2->velocity;
            }
        }
    }
    //Perform collision detection;
    make_constrains(objs);
    if (!constrains.empty()) {
        contact_dual(delta_t, gravity);
    }
}

inline void Rope::output_2_obj(int step_index) {
    f.make_New_file(step_index);
    for (auto& m : masses) {
        f.add_point(m->position);
    }
    for (auto& s : springs) {
        f.add_edge(s->m1->id, s->m2->id);
    }
    f.close_f();
}

inline void Rope::cal_force() {
    for (auto& s : springs)
    {
        // TODO (Part 2): Use Hooke's law to calculate the force on a node
        // get force by Hook's law
        Vec3d l = s->m2->position - s->m1->position;
        Vec3d f = -(s->k) * l / l.norm() * (l.norm() - s->rest_length);

        s->m1->forces -= f;
        s->m2->forces += f;
    }
}

//calculate P by constrain function ||p1-p2||-rest_length for Primal Descent
inline void Rope::cal_prim_Jaco(double dt) {
    for (auto& s : springs) {
        //p1-p2
        Vec3d l = s->m1->position - s->m2->position;
        //calculate Pdd in every degree of freedom
        double lsm = l.squaredNorm();
        double x2 = dt * s->k * l.x * l.x / lsm;
        double y2 = dt * s->k * l.y * l.y / lsm;
        double z2 = dt * s->k * l.z * l.z / lsm;
        //update m.P
        s->m1->P += Vec3d(x2, y2, z2);
        s->m2->P += Vec3d(x2, y2, z2);
    }
}

inline void Rope::init_v_p(double dt, Vec3d gravity) {
    for (auto& m : masses) {
        if (!m->pinned) {
            // u+ = u~
            m->velocity += dt * gravity;
            m->v_predict = m->velocity;
            // p+ = p- + dt * G * u
            m->last_position = m->position;
            m->position += dt * m->velocity;
        }

    }
}