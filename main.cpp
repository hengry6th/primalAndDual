#include <string>
#include <chrono>
#include <ctime>

#include "mass.h"
#include "spring.h"
#include "rope.h"
#include "world.h"
#include "other_obj.h"


Files_in_obj Rope::f;

Rope* rope;
World* world;
static int globalstep = 1;

void build_cloth(int length, int width, double spring_l) {
    const double mass_w = 0.001;
    const double k = 10.0;
    vector<Mass*> masses(length * width);
    vector<Spring*> v_springs;
    vector<Spring*> springs;
    Vec3d start_point(-width*spring_l/2, 20, 0.0);
    int count = 0;
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
            Mass* m = new Mass(i * length + j,
                               start_point + Vec3d(j * spring_l,0 , -i * spring_l),
                               mass_w, false);
            //if (i == 0 || j == 0) m->pinned = true;
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
    ms1.push_back(new Mass(0, Vec3d(2.5, 0, 0), base_m, true));
    ms1.push_back(new Mass(1, Vec3d(1.5, 0, 0), base_m, false));
    ms1.push_back(new Mass(2, Vec3d(0.5, 0, 0), base_m * m_ratio, false));
    ms1.push_back(new Mass(3, Vec3d(-0.5, 0, 0), base_m * m_ratio, false));
    ms1.push_back(new Mass(3, Vec3d(-1.5, 0, 0), base_m * m_ratio, false));
    vector<Spring*> sp1;
    sp1.push_back(new Spring(ms1[0], ms1[1], base_k));
    sp1.push_back(new Spring(ms1[1], ms1[2], base_k * k_ratio));
    sp1.push_back(new Spring(ms1[2], ms1[3], base_k * k_ratio));
    sp1.push_back(new Spring(ms1[3], ms1[4], base_k * k_ratio));
    Rope* r1 = new Rope(ms1, sp1);
    r1->m_ratio = m_ratio;
    r1->k_ratio = k_ratio;
    world->add_rope(r1, step_types::Dual);

    vector<Mass*> ms2;
    ms2.push_back(new Mass(4, Vec3d(0, 0, 0), base_m, true));
    ms2.push_back(new Mass(5, Vec3d(-1, 0, 0), base_m, false));
    ms2.push_back(new Mass(6, Vec3d(-2, 0, 0), base_m * m_ratio, false));
    ms2.push_back(new Mass(7, Vec3d(-3, 0, 0), base_m * m_ratio, false));
    ms2.push_back(new Mass(7, Vec3d(-4, 0, 0), base_m * m_ratio, false));
    vector<Spring*> sp2;
    sp2.push_back(new Spring(ms2[0], ms2[1], base_k));
    sp2.push_back(new Spring(ms2[1], ms2[2], base_k * k_ratio));
    sp2.push_back(new Spring(ms2[2], ms2[3], base_k * k_ratio));
    sp2.push_back(new Spring(ms2[3], ms2[4], base_k * k_ratio));
    Rope* r2 = new Rope(ms2, sp2);
    r2->m_ratio = m_ratio;
    r2->k_ratio = k_ratio;
    world->add_rope(r2, step_types::Prime);
}


void build_spheres(double x, double y, double z, double r, double k, double u) {
    sphere* s = new sphere(Vec3d(x, y, z), r, k, u);
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

int main() {

    world = new World();
    world->set_dt(1/60.0);
    world->set_g(Vec3d(0, -9.8, 0));

    build_cloth(10, 10, 1);
    build_spheres(0, 0, -0, 10, 100, 0.9);
    //build_ropes_m();

    world->set_rope(rope);


    for (int i = 1; i <= 300; i++) {
        world->output_all(i);
        world->step(step_type);
        std::cout << "Time step " << i << endl;
    }

    return 0;


}
