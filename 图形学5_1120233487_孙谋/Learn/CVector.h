#pragma once
#include"math.h"
class CEuler;
class CVector {
public:
    float x, y, z, w; // 四个分量

    CVector() : x(0), y(0), z(0), w(0) {}
    CVector(float x, float y, float z, float w = 1.0f) : x(x), y(y), z(z), w(w) {}
    void Set(float x, float y, float z, float w ) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }
    ~CVector() {};

    // 重载加法操作符
    CVector operator+(const CVector& p) const {
        return CVector(x + p.x, y + p.y, z + p.z, w + p.w);
    }

    // 重载减法操作符
    CVector operator-(const CVector& p) const {
        return CVector(x - p.x, y - p.y, z - p.z, w - p.w);
    }

    // 重载赋值操作符
    CVector& operator=(const CVector& p) {
        if (this != &p) {
            x = p.x;
            y = p.y;
            z = p.z;
            w = p.w;
        }
        return *this;
    }

    // 重载相等操作符
    bool operator==(const CVector& p) const {
        return (x == p.x) && (y == p.y) && (z == p.z) && (w == p.w);
    }

    // 重载不等操作符
    bool operator!=(const CVector& p) const {
        return !(*this == p);
    }

    // 重载数乘操作符
    CVector operator*(float scalar) const {
        return CVector(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    // 重载友元数乘操作符，允许标量在左侧
    friend CVector operator*(float scalar, const CVector& p) {
        return CVector(p.x * scalar, p.y * scalar, p.z * scalar, p.w * scalar);
    }

    // 点乘操作
    float dotMul(const CVector& n) const {
        return x * n.x + y * n.y + z * n.z + w * n.w;
    }

    // 叉乘操作（不适用于四维向量）
    CVector crossMul(const CVector& n) const {
        return CVector(y * n.z - z * n.y, z * n.x - x * n.z, x * n.y - y * n.x);
    }

    // 单位化操作
    CVector Normalize() const {
        float length = len();
        if (length != 0) {
            return *this * (1.0f / length);
        }
        return *this;
    }

    // 求模操作
    float len() const {
        return sqrt(x * x + y * y + z * z + w * w);
    }

    // 投影操作
    CVector project(const CVector& n) const {
        float dotProduct = dotMul(n);
        float lengthSquared = n.len() * n.len();
        if (lengthSquared != 0) {
            return n * (dotProduct / lengthSquared);
        }
        return CVector(0, 0, 0, 0);
    }
    CEuler ToEuler() const ;

    operator float* () { return &x; } // 自动类型转换
};

bool Calculate();