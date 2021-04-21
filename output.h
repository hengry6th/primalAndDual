//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_OUTPUT_H
#define REBUILD_AT_CLION_OUTPUT_H

#include <string>
#include <fstream>
#include <iostream>

#include "vec3d.h"

using namespace std;

// file class, include file ptr and put Eigen::Vertex2d to  
class Files_in_obj {
private:
    FILE* obj_f;
    FILE* obj_f_2;
public:
    Files_in_obj() : obj_f(NULL) {};
    void make_New_file(int index);
    void add_point(Vec3d v_position) {
        fprintf(obj_f, "v %lf %lf %lf\n", v_position.x, v_position.y, v_position.z);
    };
    //.obj v index start from 1
    void add_edge(int v, int u) {
        fprintf(obj_f, "l %d %d\n", v + 1, u + 1);
    }
    void close_f() {
        fclose(obj_f);
    }

    void close_f_all() {
        fclose(obj_f);
        fclose(obj_f_2);
    }

    void open_f(string f_name1, string f_name2) {
        obj_f = fopen(f_name1.c_str(), "w+");
        obj_f_2 = fopen(f_name2.c_str(), "w+");
    }

    void print_ratio_and_condiction_num(double x1, double y1, double x2, double y2){
        fprintf(obj_f, "%lf,%lf\n", x1, y1);
        fprintf(obj_f_2, "%lf,%lf\n", x2, y2);
    }
};
#endif //REBUILD_AT_CLION_OUTPUT_H
