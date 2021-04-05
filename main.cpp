#include <string>
#include <chrono>
#include <ctime>

#include "mass.h"
#include "spring.h"
#include "rope.h"
#include "world.h"
#include "other_obj.h"

#define sim_type 3
#define step_type step_types::Dual

Files_in_obj Rope::f;

Rope* rope;
World* world;
static int globalstep = 1;

void build_cloth(int length, int width, double spring_l) {
    const double mass_w = 0.001;
    const double k = 50.0;
    vector<Mass*> masses(length * width);
    vector<Spring*> v_springs;
    vector<Spring*> springs;
    Vec3d start_point(-width*spring_l/2, length*spring_l/2, 0.0);
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

void build_spheres(double x, double y, double z, double r, double k, double u) {
    sphere* s = new sphere(Vec3d(x, y, z), r, k, u);
    world->add_obj(s);
}

int main() {

    world = new World();
    world->set_dt(1.0/60);
    world->set_g(Vec3d(0, -9.8, 0));
    build_cloth(10, 10, 0.5);
    build_spheres(0, -3, -2, 3, 1, 1);
    //build_ropes_m();

    world->set_rope(rope);
    
    for (int i = 1; i <= 180; i++) {
        world->output_all(i);
        world->step(step_type);
        std::cout << "Time step " << i << endl;
    }

    return 0;


}
