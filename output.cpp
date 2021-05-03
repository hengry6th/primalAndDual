//
// Created by admin on 2021/4/5.
//

#include "output.h"
//#include <direct.h>
#include <io.h>
/*
inline void delete_file(char* path) {
    _finddata_t dir_info;
    _finddata_t file_info;
    intptr_t f_handle;
    char tmp_path[_MAX_PATH];
    remove(path);
}
*/
void Files_in_obj::make_New_file(int index) {
    //string ts = ".";// "./output";
    //string del_path = ts + "/*";
   /* if (0 !=  access(ts.data(), 0))
    {
        // if this folder not exist, create a new one.
        mkdir(ts.data());   // 返回 0 表示创建成功，-1 表示失败
    }
    */
    //delete_file(del_path.data());

    string ts = "output/rope_obj_" + to_string(index) + ".obj";
    if (!(obj_f = fopen(ts.data(), "w+"))) {
        cout << "openfail" << obj_f << endl;
    };
}
