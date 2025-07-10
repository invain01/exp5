#pragma once
#include "CMatrix.h"
#include "math.h"
#define PI 3.1415926
class CQuaternion;
class CEuler {
public:
    float h, p, b; // 偏航角（Yaw）、俯仰角（Pitch）、翻滚角（Roll）

    // 构造函数
    CEuler(float fh = 0.0f, float fp = 0.0f, float fb = 0.0f) : h(fh), p(fp), b(fb) {}
    ~CEuler() {};
    // 设置欧拉角
    void Set(float fh, float fp, float fb) {
        h = fh;
        p = fp;
        b = fb;
    }

    // 类型转换为 float*，返回 h 的地址
    operator float* () {
        return &h;
    }

    // 欧拉角转换为向量
    CVector ToVector3() const {
        
        // 将角度转换为弧度
        float yawRad = h * (3.1415926f / 180.0f);   // 偏航角（Yaw）
        float pitchRad = p * (3.1415926f / 180.0f); // 俯仰角（Pitch）

        // 计算方向向量
        float z = -cos(yawRad) * cos(pitchRad);
        float y = sin(pitchRad);
        float x = -sin(yawRad) * cos(pitchRad);

        return CVector(x, y, z, 1.0f); // 返回归一化的方向向量
        
    }
    
    // 欧拉角转换为矩阵
    CMatrix ToMatrix() const {
        float ch = cos(h * (3.1415926f / 180.0f));
        float sh = sin(h * (3.1415926f / 180.0f));
        float cp = cos(p * (3.1415926f / 180.0f));
        float sp = sin(p * (3.1415926f / 180.0f));
        float cr = cos(b * (3.1415926f / 180.0f));
        float sr = sin(b * (3.1415926f / 180.0f));

        CMatrix mat;
        mat.m22 = ch * cp;
        mat.m02 = (ch * sp * sr) - (sh * cr);
        mat.m12 = (ch * sp * cr) + (sh * sr);
        

        mat.m20 = sh * cp;
        mat.m00 = (sh * sp * sr) + (ch * cr);
        mat.m10 = (sh * sp * cr) - (ch * sr);
        

        mat.m21 = -sp;
        mat.m01 = cp * sr;
        mat.m11 = cp * cr;
        

        mat.m31 = 0.0f;
        mat.m32 = 0.0f;
        mat.m30 = 0.0f;
        mat.m03 = 0.0f;
        mat.m13 = 0.0f;
        mat.m23 = 0.0f;
        mat.m33 = 1.0f;
        
        return mat;
    }

    // 欧拉角转换为四元数
    CQuaternion ToQuaternion() const ;
    
    // 欧拉角规范化
    void Normalize() {
        while (1)
        {
            if (h < -180.0f)
            {
                h += 360.0f;
                continue;
            }
            if (h > 180.0f)
            {
                h -= 360.0f;
                continue;
            }
            if (b < -180.0f)
            {
                b += 360.0f;
                continue;
            }
            if (b > 180.0f)
            {
                b -= 360.0f;
                continue;
            }
            if (p < -90.0f)
            {
                p += 180.0f;
                continue;
            }
            if (p > 90.0f)
            {
                p -= 180.0f;
                continue;
            }
            break;
        }
       
    }
};
class CQuaternion {
public:
    float w, x, y, z;

    // 构造函数
    CQuaternion(float fw = 1.0f, float fx = 0.0f, float fy = 0.0f, float fz = 0.0f) : w(fw), x(fx), y(fy), z(fz) {}

    // 设置四元数
    void Set(float fw, float fx, float fy, float fz) {
        w = fw;
        x = fx;
        y = fy;
        z = fz;
    }

    // 类型转换为 float*，返回 w 的地址
    operator float* () {
        return &w;
    }

    // 设置旋转角度和轴
    void SetAngle(float angle, const CVector& axis) {
        float halfAngle = angle * 0.5f * (3.1415926f / 180.0f); // 角度转弧度
        float sinHalfAngle = sin(halfAngle);
        w = cos(halfAngle);
        x = axis.x * sinHalfAngle;
        y = axis.y * sinHalfAngle;
        z = axis.z * sinHalfAngle;
    }

    // 四元数转换为欧拉角
    CEuler ToEuler() const {
        CEuler euler;
        float test = x * y + z * w;

        if (test > 0.499f) {
            euler.h = 2.0f * atan2(x, w);
            euler.p = 3.1415926f / 2.0f;
            euler.b = 0.0f;
            return euler;
        }
        if (test < -0.499f) {
            euler.h = -2.0f * atan2(x, w);
            euler.p = -3.1415926f / 2.0f;
            euler.b = 0.0f;
            return euler;
        }

        euler.h = atan2(2.0f * y * w - 2.0f * x * z, 1.0f - 2.0f * y * y - 2.0f * z * z);
        euler.p = asin(2.0f * test);
        euler.b = atan2(2.0f * x * w - 2.0f * y * z, 1.0f - 2.0f * x * x - 2.0f * z * z);
        return euler;
    }

    // 四元数转换为矩阵
    CMatrix ToMatrix() const {
        CMatrix mat;
        float xx = x * x;
        float yy = y * y;
        float zz = z * z;
        float xy = x * y;
        float xz = x * z;
        float yz = y * z;
        float wx = w * x;
        float wy = w * y;
        float wz = w * z;

        mat.m00 = 1.0f - 2.0f * (yy + zz);
        mat.m10 = 2.0f * (xy - wz);
        mat.m20 = 2.0f * (xz + wy);
        mat.m30 = 0.0f;

        mat.m01 = 2.0f * (xy + wz);
        mat.m11 = 1.0f - 2.0f * (xx + zz);
        mat.m21 = 2.0f * (yz - wx);
        mat.m31 = 0.0f;

        mat.m02 = 2.0f * (xz - wy);
        mat.m12 = 2.0f * (yz + wx);
        mat.m22 = 1.0f - 2.0f * (xx + yy);
        mat.m32 = 0.0f;

        mat.m03 = 0.0f;
        mat.m13 = 0.0f;
        mat.m23 = 0.0f;
        mat.m33 = 1.0f;

        return mat;
    }

    // 重载赋值操作符
    CQuaternion& operator=(const CQuaternion& p) {
        if (this != &p) {
            w = p.w;
            x = p.x;
            y = p.y;
            z = p.z;
        }
        return *this;
    }

    // 重载加法操作符
    CQuaternion operator+(const CQuaternion& p) const {
        return CQuaternion(w + p.w, x + p.x, y + p.y, z + p.z);
    }
    

    // 重载数乘操作符
    CQuaternion operator*(float data) const {
        return CQuaternion(w * data, x * data, y * data, z * data);
    }

    // 四元数乘法
    CQuaternion operator*(const CQuaternion& p) const {
        return CQuaternion(
            w * p.w - x * p.x - y * p.y - z * p.z,
            w * p.x + x * p.w + y * p.z - z * p.y,
            w * p.y - x * p.z + y * p.w + z * p.x,
            w * p.z + x * p.y - y * p.x + z * p.w
        );
    }
    // 四元数幂运算实现 (优化版本)
    CQuaternion operator^(float t) const {
        // 提取旋转角度和轴
        float angle = 2.0f * acos(w);
        float sinHalfAngle = sqrt(1.0f - w * w);

        if (fabs(sinHalfAngle) < 1e-6f) // 无旋转的情况
            return CQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

        CVector axis(x / sinHalfAngle, y / sinHalfAngle, z / sinHalfAngle, 0.0f);

        // 计算缩放后的新角度
        float newAngle = angle * t;
        float newW = cos(newAngle / 2.0f);
        float newSin = sin(newAngle / 2.0f);

        return CQuaternion(newW,
            axis.x * newSin,
            axis.y * newSin,
            axis.z * newSin);
    }
    // 点乘
    float dotMul(const CQuaternion& p) const {
        return w * p.w + x * p.x + y * p.y + z * p.z;
    }

    // 求模
    float len() const {
        return sqrt(w * w + x * x + y * y + z * z);
    }

    // 求标准化
    bool Normalize() {
        float length = len();
        if (length == 0) return false;
        w /= length;
        x /= length;
        y /= length;
        z /= length;
        return true;
    }

    // 求逆四元数，会改变自身
    CQuaternion& Inverse() {
        float lengthSquared = w * w + x * x + y * y + z * z;
        if (lengthSquared == 0) return *this; // 如果四元数为零向量，直接返回
        float invLengthSquared = 1.0f / lengthSquared;
        w *= invLengthSquared;
        x *= -invLengthSquared;
        y *= -invLengthSquared;
        z *= -invLengthSquared;
        return *this;
    }

    // 求逆四元数，不改变自身，生成新的四元数
    CQuaternion GetInverse() const {
        float lengthSquared = w * w + x * x + y * y + z * z;
        if (lengthSquared == 0) return CQuaternion(); // 如果四元数为零向量，返回零四元数
        float invLengthSquared = 1.0f / lengthSquared;
        return CQuaternion(w * invLengthSquared, -x * invLengthSquared, -y * invLengthSquared, -z * invLengthSquared);
    }

    // 求差 当前为a,求c=a-b
    CQuaternion Div(const CQuaternion& b) const {
        return *this * (b.GetInverse());
    }

    // 求旋转轴和角度
    void GetAngle(float& angle, CVector& axis) const {
        float length = sqrt(x * x + y * y + z * z);
        if (length == 0) {
            angle = 0;
            axis = CVector(1, 0, 0, 0);
        }
        else {
            angle = 2.0f * acos(w) * (180.0f / 3.1415926f); // 弧度转角度
            axis = CVector(x / length, y / length, z / length, 0);
        }
    }

    // 球面线性插值
    CQuaternion Slerp(const CQuaternion& Vend, float t) const {
        CQuaternion q0 = *this;
        CQuaternion q1 = Vend;

        // 确保单位四元数
        q0.Normalize();
        q1.Normalize();

        // 计算点积判断方向
        float cosTheta = q0.dotMul(q1);
        if (cosTheta < 0.0f) { // 保证最短路径
            q1 = q1 * -1.0f;
            cosTheta = -cosTheta;
        }

        // 处理数值接近的情况
        const float THRESHOLD = 0.9995f;
        if (cosTheta > THRESHOLD) { // 线性插值
            CQuaternion result = q0 + (q1.Div(q0)) * t;
            result.Normalize();
            return result;
        }

        CQuaternion delta = q1*(q0.GetInverse()) ; // 计算角位移差
        CQuaternion deltaPower = delta ^ t;       // 四元数幂运算
        return deltaPower*q0;                   // 应用插值
    }

    // 球面线性插值，一次插值出 n 个数据
    void Slerp(const CQuaternion& Vend, int n, float* t, CQuaternion* Result) const {
        for (int i = 0; i < n; ++i) {
            Result[i] = Slerp(Vend, t[i]);
        }
    }
};

bool Calculate2();