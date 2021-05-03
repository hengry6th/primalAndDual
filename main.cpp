#include <string>
#include <chrono>
#include <ctime>

#include "mass.h"
#include "spring.h"
#include "rope.h"
#include "world.h"
#include "other_obj.h"
#include <fstream>

Files_in_obj Rope::f;

Rope* rope;
World* world;
static int globalstep = 1;

void build_cloth(Vec3d left_top, Vec3d left_bottom, Vec3d right_top, Vec3d right_bottom, int length, int width) {
    const double mass_w = 0.001;
    const double k = 10;
    vector<Mass*> masses(length * width);
    vector<Spring*> v_springs;
    vector<Spring*> springs;
    Vec3d start_point = left_top;
    Vec3d left_2_right = (right_top - left_top) / (width - 1);
    Vec3d top_2_bottom =  (left_bottom - left_top) / (length - 1);
    int count = 0;
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
            Mass* m = new Mass(i * length + j,
                               start_point + left_2_right * j + top_2_bottom * i,
                               mass_w, false);
            //if (j == 0) m->pinned = true;
            masses[count++] = m;
            if (i != 0) {
                Spring* s = new Spring(masses[(i - 1) * width + j], m, k);
                v_springs.push_back(s);
                springs.push_back(s);
                if (j != 0)
                    springs.push_back(
                            new Spring(masses[(i - 1) * width + j - 1], masses[i * width + j], k)
                    );
                if (j != width - 1)
                    springs.push_back(
                            new Spring(masses[(i - 1) * width + j + 1], masses[i * width + j], k)
                    );
            }
            if (j != 0) {
                Spring* s = new Spring(masses[i * width + j - 1], m, k);
                v_springs.push_back(s);
                springs.push_back(s);
            }
            if (i > 1)
                springs.push_back(
                        new Spring(masses[(i - 2) * width + j], masses[i * width + j], k)
                );

            if (j > 1)
                springs.push_back(
                        new Spring(masses[i * width + j - 2], masses[i * width + j], k)
                );
        }
    }
    rope = new Rope(masses, springs, v_springs);
    rope->is_cloth = true;
    rope->number_of_row = width;
    world->set_rope(rope);
}

void build_cloth2(Vec3d left_top, Vec3d left_bottom, Vec3d right_top, Vec3d right_bottom, int length, int width) {
    const double mass_w = 1.0E-3;
    const double k = 1.0E10;
    vector<Mass*> masses(length * width);
    vector<Spring*> v_springs;
    vector<Spring*> springs;
    Vec3d start_point = left_top;
    Vec3d left_2_right = (right_top - left_top) / (width - 1);
    Vec3d top_2_bottom =  (left_bottom - left_top) / (length - 1);
    int count = 0;
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
            Mass* m = new Mass(i * length + j,
                               start_point + left_2_right * j + top_2_bottom * i,
                               mass_w, false);
            if (j == 0 || j == width - 1 || i == 0 || i == length - 1) m->pinned = true;
            masses[count++] = m;
            if (i != 0) {
                Spring* s = new Spring(masses[(i - 1) * width + j], m, k);
                v_springs.push_back(s);
                springs.push_back(s);
                if (j != 0)
                    springs.push_back(
                            new Spring(masses[(i - 1) * width + j - 1], masses[i * width + j], k)
                    );
                if (j != width - 1)
                    springs.push_back(
                            new Spring(masses[(i - 1) * width + j + 1], masses[i * width + j], k)
                    );
            }
            if (j != 0) {
                Spring* s = new Spring(masses[i * width + j - 1], m, k);
                v_springs.push_back(s);
                springs.push_back(s);
            }
            if (i > 1)
                springs.push_back(
                        new Spring(masses[(i - 2) * width + j], masses[i * width + j], k)
                );

            if (j > 1)
                springs.push_back(
                        new Spring(masses[i * width + j - 2], masses[i * width + j], k)
                );
        }
    }
    rope = new Rope(masses, springs, v_springs);
    rope->is_cloth = true;
    rope->number_of_row = width;
    world->set_rope(rope);
}

std::string getCurrentSystemTime()
{
    auto tt = std::chrono::system_clock::to_time_t
            (std::chrono::system_clock::now());
    struct tm* ptm = localtime(&tt);
    char date[60] = { 0 };
    sprintf(date, "%d-%02d-%02d-%02d.%02d.%05d",
            (int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday,
            (int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
    return std::string(date);
}

void build_ropes_k() {
    vector<Mass*> ms1 = *new vector<Mass*>;
    ms1.push_back(new Mass(0, Vec3d(3.5, 0, 0), 0.1, true));
    ms1.push_back(new Mass(1, Vec3d(2, 0, 0), 0.1, false));
    ms1.push_back(new Mass(2, Vec3d(0.5, 0, 0), 0.1, false));
    vector<Spring*> sp1;
    sp1.push_back(new Spring(ms1[0], ms1[1], 10));
    sp1.push_back(new Spring(ms1[1], ms1[2], 10000));
    Rope* r1 = new Rope(ms1, sp1);
    world->add_rope(r1, step_types::Dual);

    vector<Mass*> ms2 = * new vector<Mass*>;
    ms2.push_back(new Mass(3, Vec3d(0, 0, 0), 0.1, true));
    ms2.push_back(new Mass(4, Vec3d(-1.5, 0, 0), 0.1, false));
    ms2.push_back(new Mass(5, Vec3d(-3, 0, 0), 0.1, false));
    vector<Spring*> sp2;
    sp2.push_back(new Spring(ms2[0], ms2[1], 10));
    sp2.push_back(new Spring(ms2[1], ms2[2], 10000));
    Rope* r2 = new Rope(ms2, sp2);
    world->add_rope(r2, step_types::Prime);
}

void build_ropes_m() {
    vector<Mass*> ms1;
    ms1.push_back(new Mass(0, Vec3d(2.5, 0, 0), 0.1, true));
    ms1.push_back(new Mass(1, Vec3d(1.5, 0, 0), 0.1, false));
    ms1.push_back(new Mass(2, Vec3d(0.5, 0, 0), 100.0, false));
    vector<Spring*> sp1;
    sp1.push_back(new Spring(ms1[0], ms1[1], 10000));
    sp1.push_back(new Spring(ms1[1], ms1[2], 10000));
    Rope* r1 = new Rope(ms1, sp1);
    world->add_rope(r1, step_types::Dual);

    vector<Mass*> ms2;
    ms2.push_back(new Mass(3, Vec3d(0, 0, 0), 0.1, true));
    ms2.push_back(new Mass(4, Vec3d(-1, 0, 0), 0.1, false));
    ms2.push_back(new Mass(5, Vec3d(-2, 0, 0), 100.0, false));
    vector<Spring*> sp2;
    sp2.push_back(new Spring(ms2[0], ms2[1], 10000));
    sp2.push_back(new Spring(ms2[1], ms2[2], 10000));
    Rope* r2 = new Rope(ms2, sp2);
    world->add_rope(r2, step_types::Prime);
}


void build_rope_by_m_and_k(double base_m, double m_ratio, double base_k, double k_ratio) {
    vector<Mass*> ms1;
    ms1.push_back(new Mass(0, Vec3d(4, 0, 0), base_m, true));
    ms1.push_back(new Mass(1, Vec3d(3, 0, 0), base_m, false));
    ms1.push_back(new Mass(2, Vec3d(2, 0, 0), base_m  * m_ratio, false));
    vector<Spring*> sp1;
    sp1.push_back(new Spring(ms1[0], ms1[1], base_k));
    sp1.push_back(new Spring(ms1[1], ms1[2], base_k * k_ratio));

    Rope* r1 = new Rope(ms1, sp1);
    r1->m_ratio = m_ratio;
    r1->k_ratio = k_ratio;
    world->add_rope(r1, step_types::Dual);

    vector<Mass*> ms2;
    ms2.push_back(new Mass(4, Vec3d(-5, 0, 0), base_m, true));
    ms2.push_back(new Mass(5, Vec3d(-6, 0, 0), base_m, false));
    ms2.push_back(new Mass(6, Vec3d(-7, 0, 0), base_m * m_ratio, false));
    vector<Spring*> sp2;
    sp2.push_back(new Spring(ms2[0], ms2[1], base_k));
    sp2.push_back(new Spring(ms2[1], ms2[2], base_k * k_ratio));
    Rope* r2 = new Rope(ms2, sp2);
    r2->m_ratio = m_ratio;
    r2->k_ratio = k_ratio;
    world->add_rope(r2, step_types::Prime);
}


void build_spheres(double x, double y, double z, double r, double k, double u) {
    Mass m = Mass(-1, Vec3d(x, y, z), 1000, true);
    sphere* s = new sphere(m, r, k, u);
    world->add_obj(s);
}

void build_spheres_with_mass(Mass m, double r, double k, double u) {
    sphere* s = new sphere(m, r, k, u);
    world->add_obj(s);
}

void test_mass_ratio_and_stiffness_ratio(){
    world->start_print_rope_ratio("m_ratio.txt");
    for(double i = 1; i < 101; i += 0.5) {
        build_rope_by_m_and_k(100,  i, 0.01, 1);
        world->reset_condi_num();
        for (int j = 0; j < 180; j++) {
            world->step_all();
            world->count_rope_ratio();
            world->output_all(j);
        }
        world->print_rope_ratio(180, 0);
        world->clear_ropes();
        printf("m_ratio:%lf\n", i);
    }
    world->end_print_rope_ratio();

    world->start_print_rope_ratio("k_ratio.txt");
    for(double i = 1; i < 100; i += 0.5) {
        build_rope_by_m_and_k(0.01, 1, 1, i);
        world->reset_condi_num();
        for (int j = 0; j < 180; j++) {
            world->step_all();
            world->count_rope_ratio();
        }
        world->print_rope_ratio(180, 1);
        world->clear_ropes();
        printf("k_ratio:%lf\n", i);
    }
    world->end_print_rope_ratio();
}

step_types step_type = step_types::Dual;

void exp_mass_ratio_contrast_in_ropes() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -5, 0));

    build_ropes_m();

    for (int i = 1; i <= 300; i++) {
        world->output_all(i);
        world->step_all();
        std::cout << "Time step " << i << endl;
    }
}

void exp_stiffness_ratio_contrast_in_ropes() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -5, 0));

    build_ropes_k();

    for (int i = 1; i <= 300; i++) {
        world->output_all(i);
        world->step_all();
        std::cout << "Time step " << i << endl;
    }
}

void exp_cloth_simulate_primal() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth(Vec3d(-15, 15, 15), Vec3d(-15, 0, 0), Vec3d(15, 15, 15), Vec3d(15, 0, 0),30, 30);

    for (int i = 1; i <= 600; i++) {
        world->output_all(i);
        world->step(step_types::Prime);
        std::cout << "Time step " << i << endl;
    }
}

void exp_cloth_simulate_dual() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth(Vec3d(-15, 15, 15), Vec3d(-15, 0, 0), Vec3d(15, 15, 15), Vec3d(15, 0, 0),30, 30);

    for (int i = 1; i <= 600; i++) {
        world->output_all(i);
        world->step(step_types::Dual);
        std::cout << "Time step " << i << endl;
    }
}

void exp_cloth_simulate_primal_contact() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth(Vec3d(-10, 10, 10), Vec3d(-10, 10, -10),
                Vec3d(10, 10, 10),Vec3d(10, 10, -10),
                30, 30);

    build_spheres(-5, -15, 0, 20, 1000, 0);

    for (int i = 1; i <= 600; i++) {
        world->output_all(i);
        world->step(step_types::Prime);
        std::cout << "Time step " << i << endl;
    }
}

void exp_cloth_simulate_dual_contact() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth(Vec3d(-15, 10, 15), Vec3d(-15, 10, -15), Vec3d(15, 10, 15), Vec3d(15, 10, -15),30, 30);

    build_spheres(-5, -10, 0, 15, 1000, 0);

    for (int i = 1; i <= 1200; i++) {
        world->output_all(i);
        world->step(step_types::Dual);
        std::cout << "Time step " << i << endl;
    }
}

void exp_cloth_simulate_primal_friction() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth(Vec3d(-10, 10, 10), Vec3d(-10, 10, -10),
                Vec3d(10, 10, 10),Vec3d(10, 10, -10),
                30, 30);

    build_spheres(-5, -15, 0, 20, 1000, 1);
    world->add_objs_to_rope();

    for (int i = 1; i <= 600; i++) {
        world->output_all(i);
        world->step(step_types::Prime);
        std::cout << "Time step " << i << endl;
    }
}

void exp_cloth_simulate_dual_friction() {
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth(Vec3d(-15, 10, 15), Vec3d(-15, 10, -15), Vec3d(15, 10, 15), Vec3d(15, 10, -15),30, 30);

    build_spheres(-5, -10, 0, 15, 1000, 1);

    world->add_objs_to_rope();

    for (int i = 1; i <= 1200; i++) {
        world->output_all(i);
        world->step(step_types::Dual);
        std::cout << "Time step " << i << endl;
    }
}

void exp_mass_stiffness_condiction_num() {
    world = new World();
    world->set_dt(2.0);
    world->set_g(Vec3d(0, -9.8, 0));

    world->start_print_rope_ratio("mass_ratio.txt");
    for (double i = 1; i < 100; i += 0.5) {
        build_rope_by_m_and_k(0.1, i, 10000, 1);
        world->reset_condi_num();
        for (int j = 0; j < 180; j++) {
            world->step_all();
            world->count_rope_ratio();
        }
        printf("mass ratio: %lf\n", i);
        world->print_rope_ratio(180, 0);
        world->reset_condi_num();
        world->clear_ropes();
    }
    world->end_print_rope_ratio();

    world->start_print_rope_ratio("stiffness_ratio.txt");
    for (double i = 1; i < 100; i += 0.5) {
        build_rope_by_m_and_k(0.1, 1, 10.0, i);
        world->reset_condi_num();
        for (int j = 0; j < 180; j++) {
            world->step_all();
            world->count_rope_ratio();
        }
        printf("stiffness ratio: %lf\n", i);
        world->print_rope_ratio(180, 1);
        world->reset_condi_num();
        world->clear_ropes();
    }
    world->end_print_rope_ratio();

};

void exp_contact_force_with_penetration() {
    FILE *primal_normal_f = fopen("primal_normal_force.txt", "w+");
    FILE *primal_friction_f = fopen("primal_friction_force.txt", "w+");
    const double kf = 1;
    for (double i = 1.99; i > 0; i -= 0.01) {
        Vec3d velocity = Vec3d(-1, -1, -1);
        velocity /= velocity.norm() * (2 - i);

        Vec3d n = Vec3d(0, 0, 1);
        Vec3d tengent = Vec3d(0, 1, 0);
        Vec3d tengent2 =  Vec3d(1, 0, 0);

        double u1 = tengent.dot(velocity);
        double u2 = tengent2.dot(velocity);

        double n_norm = n.norm();

        double ci = i - n_norm;
        Vec3d forces = n / n_norm * 2 * min(0.0, ci);

        double squrt_u = sqrt(u1 * u1 + u2 * u2);
        double uf = forces.norm() ;
        fprintf(primal_normal_f, "%lf,%lf\n", 1 - i, forces.norm());
        printf("penetrate: %lf\n", 1 - i);
    }
    fclose(primal_normal_f);
    for (double i = 1.99; i > 0; i -= 0.01) {
        Vec3d velocity = Vec3d(-1, -1, 0);
        velocity /= velocity.norm();
        velocity *= (1 - i);

        Vec3d n = Vec3d(0, 0, 1);
        Vec3d tengent = Vec3d(0, 1, 0);
        Vec3d tengent2 =  Vec3d(1, 0, 0);

        double u1 = tengent.dot(velocity);
        double u2 = tengent2.dot(velocity);

        double n_norm = n.norm();

        double ci = 0.7 - n_norm;
        Vec3d forces = n / n_norm * 2 * min(0.0, ci);

        double squrt_u = sqrt(u1 * u1 + u2 * u2);
        double uf = forces.norm() ;
        Vec3d forces_f = min(kf, uf/squrt_u) * (u1 * tengent + u2 * tengent2);
        double f_norm = forces_f.norm() * ((1 - i > 0) ? -1.0 : 1.0);
        fprintf(primal_friction_f, "%lf,%lf\n", 1 - i, f_norm / forces.norm());
        printf("penetrate: %lf\n", 1 - i);
    }
    fclose(primal_friction_f);
};

void exp_sphere_on_primal_cloth()
{
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth2(Vec3d(-15, 0, 5), Vec3d(-15, 0, -5),
                Vec3d(15, 0, 5),Vec3d(15, 0, -5),
                30, 30);
    Mass m = Mass(-1, Vec3d(0, 10, 0), 1000, false);
    build_spheres_with_mass(m, 9, 1, 1);
    world->add_objs_to_rope();

    for (int i = 1; i <= 600; i++) {
        world->output_all(i);
        world->step(step_types::Prime);
        std::cout << "Time step " << i << endl;
    }
}

void exp_sphere_on_dual_cloth()
{
    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth2(Vec3d(-15, 0, 5), Vec3d(-15, 0, -5),
                 Vec3d(15, 0, 5),Vec3d(15, 0, -5),
                 30, 30);
    Mass m = Mass(-1, Vec3d(0, 10, 0), 1000, false);
    build_spheres_with_mass(m, 9, 1000, 1);
    world->add_objs_to_rope();

    for (int i = 1; i <= 600; i++) {
        world->output_all(i);
        world->step(step_types::Dual);
        std::cout << "Time step " << i << endl;
    }
}

int main() {

    //exp_mass_ratio_contrast_in_ropes();
    //exp_stiffness_ratio_contrast_in_ropes();
    //exp_cloth_simulate_primal();
    //exp_cloth_simulate_dual();
    //exp_cloth_simulate_primal_contact();
    //exp_cloth_simulate_dual_contact();
    //exp_cloth_simulate_primal_friction();
    //exp_cloth_simulate_dual_friction();
    exp_mass_stiffness_condiction_num();
    //exp_contact_force_with_penetration();
    //exp_sphere_on_primal_cloth();
    //exp_sphere_on_dual_cloth();
    return 0;


}
