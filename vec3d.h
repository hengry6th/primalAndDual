//
// Created by admin on 2021/4/5.
//

#ifndef REBUILD_AT_CLION_VEC3D_H
#define REBUILD_AT_CLION_VEC3D_H

#include <math.h>

class Vec3d {
public:
    double x;
    double y;
    double z;

    Vec3d() {};

    Vec3d(double x, double y, double  z)
        :x(x), y(y), z(z) {};

    Vec3d operator+(const Vec3d& B) {
        Vec3d vec;
        vec.x = x + B.x;
        vec.y = y + B.y;
        vec.z = z + B.z;
        return vec;
    }

    Vec3d operator-(const Vec3d& B) {
        Vec3d vec;
        vec.x = x - B.x;
        vec.y = y - B.y;
        vec.z = z - B.z;
        return vec;
    }

    Vec3d operator*(const float& B) {
        return Vec3d(x * B, y * B, z * B);
    }

    friend Vec3d operator*(const double &A, const Vec3d &B) {
        return Vec3d(A*B.x, A * B.y, A * B.z);
    }

    Vec3d operator/(const double& B) {
        Vec3d vec;
        vec.x = x / B;
        vec.y = y / B;
        vec.z = z / B;
        return vec;
    }

    Vec3d operator=(const Vec3d& B) {
        x = B.x;
        y = B.y;
        z = B.z;
        return B;
    }

    Vec3d operator+=(const Vec3d& B) {
        x += B.x;
        y += B.y;
        z += B.z;
        return *this;
    }

    Vec3d operator-=(const Vec3d& B) {
        x -= B.x;
        y -= B.y;
        z -= B.z;
        return *this;
    }

    Vec3d operator*=(const double& B) {
        x *= B;
        y *= B;
        z *= B;
        return *this;
    }

    Vec3d operator/=(const double& B) {
        x /= B;
        y /= B;
        z /= B;
        return *this;
    }

    double squaredNorm() {
        return x * x + y * y + z * z;
    }

    double norm() {
        return sqrt(this->squaredNorm());
    }

    double dot(const Vec3d& B) {
        return x * B.x + y * B.y + z * B.z;
    }

    Vec3d cross(const Vec3d& B) {
        Vec3d vec(y*B.z - B.y * z,B.x * z - x * B.z, x * B.y - B.x * y);
        return vec;
    }
};
#endif //REBUILD_AT_CLION_VEC3D_H
