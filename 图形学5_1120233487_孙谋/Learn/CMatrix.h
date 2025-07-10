#pragma once
#include "CVector.h"
class CEuler;
class CQuaternion;
class CMatrix {
public:
    float m00, m01, m02, m03;
    float m10, m11, m12, m13;
    float m20, m21, m22, m23;
    float m30, m31, m32, m33;


    // 构造函数，初始化为单位矩阵
    CMatrix() {
        SetIdentity();
    }
    ~CMatrix() {};
    // 设置为单位矩阵
    void SetIdentity() {
        m00 = 1.0f; m10 = 0.0f; m20 = 0.0f; m30 = 0.0f;
        m01 = 0.0f; m11 = 1.0f; m21 = 0.0f; m31 = 0.0f;
        m02 = 0.0f; m12 = 0.0f; m22 = 1.0f; m32 = 0.0f;
        m03 = 0.0f; m13 = 0.0f; m23 = 0.0f; m33 = 1.0f;
    }

    // 设置矩阵的值
    void Set(float* val) {
        m00 = val[0]; m01 = val[4]; m02 = val[8]; m03 = val[12];
        m10 = val[1]; m11 = val[5]; m12 = val[9]; m13 = val[13];
        m20 = val[2]; m21 = val[6]; m22 = val[10]; m23 = val[14];
        m30 = val[3]; m31 = val[7]; m32 = val[11]; m33 = val[15];
    }

    // 类型转换重载，将 CMatrix 转换为 float* 类型
    operator float* () {
        return &m00;
    }

    // 矩阵赋值
    CMatrix& operator=(const CMatrix& p) {
        m00 = p.m00; m01 = p.m01; m02 = p.m02; m03 = p.m03;
        m10 = p.m10; m11 = p.m11; m12 = p.m12; m13 = p.m13;
        m20 = p.m20; m21 = p.m21; m22 = p.m22; m23 = p.m23;
        m30 = p.m30; m31 = p.m31; m32 = p.m32; m33 = p.m33;
        return *this;
    }
    void Orthogonalize() {
        // 提取列向量
        CVector col0(m00, m10, m20, m30);
        CVector col1(m01, m11, m21, m31);
        CVector col2(m02, m12, m22, m32);

        // 使用 Gram-Schmidt 过程正交化列向量
        // 先单位化第一个向量
        CVector q0 = col0.Normalize();

        // 第二个向量减去在第一个向量上的投影，然后单位化
        CVector q1 = col1 - q0 * (q0.dotMul(col1));
        q1 = q1.Normalize();

        // 第三个向量减去在前两个向量上的投影，然后单位化
        CVector q2 = col2 - q0 * (q0.dotMul(col2)) - q1 * (q1.dotMul(col2));
        q2 = q2.Normalize();

        // 更新矩阵的列向量
        m00 = q0.x;
        m10 = q1.x;
        m20 = q2.x;
        m30 = q0.w;

        m01 = q0.y;
        m11 = q1.y;
        m21 = q2.y;
        m31 = q1.w;

        m02 = q0.z;
        m12 = q1.z;
        m22 = q2.z;
        m32 = q2.w;
    }
    // 矩阵数乘
    CMatrix operator*(float d) {
        CMatrix result;
        result.m00 = m00 * d; result.m01 = m01 * d; result.m02 = m02 * d; result.m03 = m03 * d;
        result.m10 = m10 * d; result.m11 = m11 * d; result.m12 = m12 * d; result.m13 = m13 * d;
        result.m20 = m20 * d; result.m21 = m21 * d; result.m22 = m22 * d; result.m23 = m23 * d;
        result.m30 = m30 * d; result.m31 = m31 * d; result.m32 = m32 * d; result.m33 = m33 * d;
        return result;
    }

    // 矩阵相乘
    CMatrix operator*(const CMatrix& p) {
        CMatrix result;
        result.m00 = m00 * p.m00 + m01 * p.m10 + m02 * p.m20 + m03 * p.m30;
        result.m01 = m00 * p.m01 + m01 * p.m11 + m02 * p.m21 + m03 * p.m31;
        result.m02 = m00 * p.m02 + m01 * p.m12 + m02 * p.m22 + m03 * p.m32;
        result.m03 = m00 * p.m03 + m01 * p.m13 + m02 * p.m23 + m03 * p.m33;

        result.m10 = m10 * p.m00 + m11 * p.m10 + m12 * p.m20 + m13 * p.m30;
        result.m11 = m10 * p.m01 + m11 * p.m11 + m12 * p.m21 + m13 * p.m31;
        result.m12 = m10 * p.m02 + m11 * p.m12 + m12 * p.m22 + m13 * p.m32;
        result.m13 = m10 * p.m03 + m11 * p.m13 + m12 * p.m23 + m13 * p.m33;

        result.m20 = m20 * p.m00 + m21 * p.m10 + m22 * p.m20 + m23 * p.m30;
        result.m21 = m20 * p.m01 + m21 * p.m11 + m22 * p.m21 + m23 * p.m31;
        result.m22 = m20 * p.m02 + m21 * p.m12 + m22 * p.m22 + m23 * p.m32;
        result.m23 = m20 * p.m03 + m21 * p.m13 + m22 * p.m23 + m23 * p.m33;

        result.m30 = m30 * p.m00 + m31 * p.m10 + m32 * p.m20 + m33 * p.m30;
        result.m31 = m30 * p.m01 + m31 * p.m11 + m32 * p.m21 + m33 * p.m31;
        result.m32 = m30 * p.m02 + m31 * p.m12 + m32 * p.m22 + m33 * p.m32;
        result.m33 = m30 * p.m03 + m31 * p.m13 + m32 * p.m23 + m33 * p.m33;
        return result;
    }

    // 矩阵与向量相乘（向量作为列向量）
    CVector vecMul(const CVector& p) {
        return CVector(
            m00 * p.x + m01 * p.y + m02 * p.z + m03 * p.w,
            m10 * p.x + m11 * p.y + m12 * p.z + m13 * p.w,
            m20 * p.x + m21 * p.y + m22 * p.z + m23 * p.w,
            m30 * p.x + m31 * p.y + m32 * p.z + m33 * p.w
        );
    }

    // 矩阵与位置相乘（向量作为列向量，考虑平移）
    CVector posMul(const CVector& p) {
        return CVector(
            m00 * p.x + m01 * p.y + m02 * p.z + m03,
            m10 * p.x + m11 * p.y + m12 * p.z + m13,
            m20 * p.x + m21 * p.y + m22 * p.z + m23,
            m30 * p.x + m31 * p.y + m32 * p.z + m33
        );
    }

    // 设置为旋转矩阵
    void SetRotate(float angle, const CVector& axis) {
        float rad = angle * 3.1415926f / 180.0f; // 角度转弧度
        CVector n = axis.Normalize();
        float c = cos(rad);
        float s = sin(rad);
        float t = 1.0f - c;

        SetIdentity();
        m00 = t * n.x * n.x + c;
        m10 = t * n.x * n.y - s * n.z;
        m20 = t * n.x * n.z + s * n.y;
        m01 = t * n.x * n.y + s * n.z;
        m11 = t * n.y * n.y + c;
        m21 = t * n.y * n.z - s * n.x;
        m02 = t * n.x * n.z - s * n.y;
        m12 = t * n.y * n.z + s * n.x;
        m22 = t * n.z * n.z + c;
    }

    // 设置为平移矩阵
    void SetTrans(const CVector& trans) {
        SetIdentity();
        m30 = trans.x;
        m31 = trans.y;
        m32 = trans.z;
        m33 = 1.0f;
    }

    // 设置为缩放矩阵
    void SetScale(const CVector& scale) {
        SetIdentity();
        m00 = scale.x;
        m11 = scale.y;
        m22 = scale.z;
        m33 = 1.0f;
    }

    // 矩阵求逆，成功返回行列式的值，否则返回 0
    float Inverse() {
        float det = Determinant();
        if (det == 0.0f) {
            return 0.0f; // 矩阵不可逆
        }
        CMatrix adj = Adjugate();
        *this = adj * (1.0f / det);
        return det;
    }

    // 返回逆矩阵
    CMatrix GetInverse() {
        float det = Determinant();
        if (det == 0.0f) {
            return CMatrix(); // 返回单位矩阵
        }
        CMatrix adj = Adjugate();
        return adj * (1.0f / det);
    }
    CEuler ToEuler() const;
    CQuaternion ToQuaternion() const;
private:
    // 计算行列式
    float Determinant() const {
        return
            m00 * (m11 * (m22 * m33 - m32 * m23) - m21 * (m12 * m33 - m32 * m13) + m31 * (m12 * m23 - m22 * m13)) -
            m10 * (m01 * (m22 * m33 - m32 * m23) - m21 * (m02 * m33 - m32 * m03) + m31 * (m02 * m23 - m22 * m03)) +
            m20 * (m01 * (m12 * m33 - m32 * m13) - m11 * (m02 * m33 - m32 * m03) + m31 * (m02 * m13 - m12 * m03)) -
            m30 * (m01 * (m12 * m23 - m22 * m13) - m11 * (m02 * m23 - m22 * m03) + m21 * (m02 * m13 - m12 * m03));
    }

    // 计算伴随矩阵
    CMatrix Adjugate() const {
        CMatrix adj;
        adj.m00 = Cofactor(0, 0);
        adj.m10 = Cofactor(1, 0);
        adj.m20 = Cofactor(2, 0);
        adj.m30 = Cofactor(3, 0);
        adj.m01 = Cofactor(0, 1);
        adj.m11 = Cofactor(1, 1);
        adj.m21 = Cofactor(2, 1);
        adj.m31 = Cofactor(3, 1);
        adj.m02 = Cofactor(0, 2);
        adj.m12 = Cofactor(1, 2);
        adj.m22 = Cofactor(2, 2);
        adj.m32 = Cofactor(3, 2);
        adj.m03 = Cofactor(0, 3);
        adj.m13 = Cofactor(1, 3);
        adj.m23 = Cofactor(2, 3);
        adj.m33 = Cofactor(3, 3);
        return adj;
    }

    // 计算代数余子式
    float Cofactor(int row, int col) const {
        return pow(-1, row + col) * Minor(row, col);
    }

    // 计算余子式
    float Minor(int row, int col) const {
        float minor[12];
        int index = 0;
        for (int i = 0; i < 4; ++i) {
            if (i == row) continue;
            for (int j = 0; j < 4; ++j) {
                if (j == col) continue;
                minor[index++] = ((float*)this)[i * 4 + j];
            }
        }
        return
            minor[0] * (minor[4] * minor[8] - minor[7] * minor[5]) -
            minor[1] * (minor[3] * minor[8] - minor[6] * minor[5]) +
            minor[2] * (minor[3] * minor[7] - minor[6] * minor[4]);
    }
};

bool Calculate1();