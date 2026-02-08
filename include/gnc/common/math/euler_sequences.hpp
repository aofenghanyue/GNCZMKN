/**
 * @file euler_sequences.hpp
 * @brief 欧拉角序列公式库
 * 
 * 实现12种欧拉角序列（6种Tait-Bryan + 6种Proper Euler）
 * 
 * 约定：
 * - 内旋序列 (Intrinsic Rotations)
 * - 序列 123 表示：先绕 X 轴，再绕新 Y 轴，最后绕新 Z 轴
 * - 矩阵: M_total = M3 * M2 * M1
 */
#pragma once

#include "quaternion.hpp"
#include <cmath>

namespace gnc::math {

/**
 * @brief 欧拉角序列枚举
 */
enum class EulerSeq {
    // Tait-Bryan 序列
    XYZ, // 123
    XZY, // 132
    YXZ, // 213
    YZX, // 231
    ZXY, // 312
    ZYX, // 321
    
    // Proper Euler 序列
    XYX, // 121
    XZX, // 131
    YXY, // 212
    YZY, // 232
    ZXZ, // 313
    ZYZ  // 323
};

// ==================== 基元旋转矩阵 ====================

/**
 * @brief 绕 X 轴旋转矩阵
 */
inline Matrix3 rotX(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Matrix3 R;
    R << 1,  0,  0,
         0,  c,  s,
         0, -s,  c;
    return R;
}

/**
 * @brief 绕 Y 轴旋转矩阵
 */
inline Matrix3 rotY(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Matrix3 R;
    R << c,  0, -s,
         0,  1,  0,
         s,  0,  c;
    return R;
}

/**
 * @brief 绕 Z 轴旋转矩阵
 */
inline Matrix3 rotZ(double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);
    Matrix3 R;
    R <<  c, s, 0,
         -s, c, 0,
          0, 0, 1;
    return R;
}

// ==================== Tait-Bryan 序列 (6种) ====================

/**
 * @brief 序列 123 (X->Y->Z) 欧拉角转四元数
 */
inline Quaternion euler123ToQuat(double t1, double t2, double t3) {
    double c1 = std::cos(t1 * 0.5), s1 = std::sin(t1 * 0.5);
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c3 = std::cos(t3 * 0.5), s3 = std::sin(t3 * 0.5);
    
    return Quaternion(
        c1*c2*c3 - s1*s2*s3,  // w
        c2*c3*s1 + c1*s2*s3,  // x
        c1*c3*s2 - c2*s1*s3,  // y
        c3*s1*s2 + c1*c2*s3   // z
    );
}

/**
 * @brief 序列 123 (X->Y->Z) 欧拉角转旋转矩阵
 */
inline Matrix3 euler123ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c2*c3,               s1*s2*c3 + c1*s3,    s1*s3 - c1*s2*c3,
        -c2*s3,               c1*c3 - s1*s2*s3,    c1*s2*s3 + s1*c3,
         s2,                 -s1*c2,               c1*c2;
    return R;
}

/**
 * @brief 序列 132 (X->Z->Y) 欧拉角转四元数
 */
inline Quaternion euler132ToQuat(double t1, double t2, double t3) {
    double c1 = std::cos(t1 * 0.5), s1 = std::sin(t1 * 0.5);
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c3 = std::cos(t3 * 0.5), s3 = std::sin(t3 * 0.5);
    
    return Quaternion(
        c1*c2*c3 + s1*s2*s3,  // w
        c2*c3*s1 - c1*s2*s3,  // x
        c1*c2*s3 - c3*s1*s2,  // y
        c1*c3*s2 + c2*s1*s3   // z
    );
}

/**
 * @brief 序列 132 (X->Z->Y) 欧拉角转旋转矩阵
 */
inline Matrix3 euler132ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c2*c3,    c1*s2*c3 + s1*s3,    s1*s2*c3 - c1*s3,
        -s2,       c1*c2,               s1*c2,
         c2*s3,    c1*s2*s3 - s1*c3,    s1*s2*s3 + c1*c3;
    return R;
}

/**
 * @brief 序列 213 (Y->X->Z) 欧拉角转四元数
 */
inline Quaternion euler213ToQuat(double t1, double t2, double t3) {
    double c1 = std::cos(t1 * 0.5), s1 = std::sin(t1 * 0.5);
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c3 = std::cos(t3 * 0.5), s3 = std::sin(t3 * 0.5);
    
    return Quaternion(
        c1*c2*c3 + s1*s2*s3,  // w
        c1*c3*s2 + c2*s1*s3,  // x
        c2*c3*s1 - c1*s2*s3,  // y
        c1*c2*s3 - c3*s1*s2   // z
    );
}

/**
 * @brief 序列 213 (Y->X->Z) 欧拉角转旋转矩阵
 */
inline Matrix3 euler213ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << s1*s2*s3 + c1*c3,   c2*s3,   c1*s2*s3 - s1*c3,
         s1*s2*c3 - c1*s3,   c2*c3,   c1*s2*c3 + s1*s3,
         s1*c2,             -s2,      c1*c2;
    return R;
}

/**
 * @brief 序列 231 (Y->Z->X) 欧拉角转四元数
 */
inline Quaternion euler231ToQuat(double t1, double t2, double t3) {
    double c1 = std::cos(t1 * 0.5), s1 = std::sin(t1 * 0.5);
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c3 = std::cos(t3 * 0.5), s3 = std::sin(t3 * 0.5);
    
    return Quaternion(
        c1*c2*c3 - s1*s2*s3,  // w
        c3*s1*s2 + c1*c2*s3,  // x
        c2*c3*s1 + c1*s2*s3,  // y
        c1*c3*s2 - c2*s1*s3   // z
    );
}

/**
 * @brief 序列 231 (Y->Z->X) 欧拉角转旋转矩阵
 */
inline Matrix3 euler231ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c1*c2,                        s2,         -c2*s1,
         s1*s3 - c1*s2*c3,             c2*c3,       s1*s2*c3 + c1*s3,
         c1*s2*s3 + s1*c3,            -c2*s3,       c1*c3 - s1*s2*s3;
    return R;
}

/**
 * @brief 序列 312 (Z->X->Y) 欧拉角转四元数
 */
inline Quaternion euler312ToQuat(double t1, double t2, double t3) {
    double c1 = std::cos(t1 * 0.5), s1 = std::sin(t1 * 0.5);
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c3 = std::cos(t3 * 0.5), s3 = std::sin(t3 * 0.5);
    
    return Quaternion(
        c1*c2*c3 - s1*s2*s3,  // w
        c1*c3*s2 - c2*s1*s3,  // x
        c3*s1*s2 + c1*c2*s3,  // y
        c2*c3*s1 + c1*s2*s3   // z
    );
}

/**
 * @brief 序列 312 (Z->X->Y) 欧拉角转旋转矩阵
 */
inline Matrix3 euler312ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c1*c3 - s1*s2*s3,   c1*s2*s3 + s1*c3,  -c2*s3,
        -s1*c2,              c1*c2,              s2,
         s1*s2*c3 + c1*s3,   s1*s3 - c1*s2*c3,   c2*c3;
    return R;
}

/**
 * @brief 序列 321 (Z->Y->X) 欧拉角转四元数
 * 
 * 航空常用序列 (yaw-pitch-roll)
 */
inline Quaternion euler321ToQuat(double t1, double t2, double t3) {
    double c1 = std::cos(t1 * 0.5), s1 = std::sin(t1 * 0.5);
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c3 = std::cos(t3 * 0.5), s3 = std::sin(t3 * 0.5);
    
    return Quaternion(
        c1*c2*c3 + s1*s2*s3,  // w
        c1*c2*s3 - c3*s1*s2,  // x
        c1*c3*s2 + c2*s1*s3,  // y
        c2*c3*s1 - c1*s2*s3   // z
    );
}

/**
 * @brief 序列 321 (Z->Y->X) 欧拉角转旋转矩阵
 */
inline Matrix3 euler321ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c1*c2,                        s1*c2,                       -s2,
         c1*s2*s3 - s1*c3,             s1*s2*s3 + c1*c3,             c2*s3,
         c1*s2*c3 + s1*s3,             s1*s2*c3 - c1*s3,             c2*c3;
    return R;
}

// ==================== Proper Euler 序列 (6种) ====================

/**
 * @brief 序列 121 (X->Y->X) 欧拉角转四元数
 */
inline Quaternion euler121ToQuat(double t1, double t2, double t3) {
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c_sum = std::cos((t1 + t3) * 0.5), s_sum = std::sin((t1 + t3) * 0.5);
    double c_diff = std::cos((t1 - t3) * 0.5), s_diff = std::sin((t1 - t3) * 0.5);
    
    return Quaternion(c2*c_sum, c2*s_sum, c_diff*s2, s2*s_diff);
}

/**
 * @brief 序列 121 (X->Y->X) 欧拉角转旋转矩阵
 */
inline Matrix3 euler121ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c2,           s1*s2,                       -c1*s2,
         s2*s3,        c1*c3 - s1*c2*s3,             c1*c2*s3 + s1*c3,
         s2*c3,       -s1*c2*c3 - c1*s3,             c1*c2*c3 - s1*s3;
    return R;
}

/**
 * @brief 序列 131 (X->Z->X) 欧拉角转四元数
 */
inline Quaternion euler131ToQuat(double t1, double t2, double t3) {
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c_sum = std::cos((t1 + t3) * 0.5), s_sum = std::sin((t1 + t3) * 0.5);
    double c_diff = std::cos((t1 - t3) * 0.5), s_diff = std::sin((t1 - t3) * 0.5);
    
    return Quaternion(c2*c_sum, c2*s_sum, -s2*s_diff, c_diff*s2);
}

/**
 * @brief 序列 131 (X->Z->X) 欧拉角转旋转矩阵
 */
inline Matrix3 euler131ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c2,           c1*s2,                        s1*s2,
        -s2*c3,        c1*c2*c3 - s1*s3,             s1*c2*c3 + c1*s3,
         s2*s3,       -s1*c3 - c1*c2*s3,             c1*c3 - s1*c2*s3;
    return R;
}

/**
 * @brief 序列 212 (Y->X->Y) 欧拉角转四元数
 */
inline Quaternion euler212ToQuat(double t1, double t2, double t3) {
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c_sum = std::cos((t1 + t3) * 0.5), s_sum = std::sin((t1 + t3) * 0.5);
    double c_diff = std::cos((t1 - t3) * 0.5), s_diff = std::sin((t1 - t3) * 0.5);
    
    return Quaternion(c2*c_sum, c_diff*s2, c2*s_sum, -s2*s_diff);
}

/**
 * @brief 序列 212 (Y->X->Y) 欧拉角转旋转矩阵
 */
inline Matrix3 euler212ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c1*c3 - s1*c2*s3,   s2*s3,   -s1*c3 - c1*c2*s3,
         s1*s2,              c2,       c1*s2,
         s1*c2*c3 + c1*s3,  -s2*c3,    c1*c2*c3 - s1*s3;
    return R;
}

/**
 * @brief 序列 232 (Y->Z->Y) 欧拉角转四元数
 */
inline Quaternion euler232ToQuat(double t1, double t2, double t3) {
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c_sum = std::cos((t1 + t3) * 0.5), s_sum = std::sin((t1 + t3) * 0.5);
    double c_diff = std::cos((t1 - t3) * 0.5), s_diff = std::sin((t1 - t3) * 0.5);
    
    return Quaternion(c2*c_sum, s2*s_diff, c2*s_sum, c_diff*s2);
}

/**
 * @brief 序列 232 (Y->Z->Y) 欧拉角转旋转矩阵
 */
inline Matrix3 euler232ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c1*c2*c3 - s1*s3,   s2*c3,    s1*c2*c3 + c1*s3,
        -c1*s2,              c2,       s1*s2,
         c1*c2*s3 + s1*c3,   s2*s3,    c1*c3 - s1*c2*s3;
    return R;
}

/**
 * @brief 序列 313 (Z->X->Z) 欧拉角转四元数
 */
inline Quaternion euler313ToQuat(double t1, double t2, double t3) {
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c_sum = std::cos((t1 + t3) * 0.5), s_sum = std::sin((t1 + t3) * 0.5);
    double c_diff = std::cos((t1 - t3) * 0.5), s_diff = std::sin((t1 - t3) * 0.5);
    
    return Quaternion(c2*c_sum, c_diff*s2, s2*s_diff, c2*s_sum);
}

/**
 * @brief 序列 313 (Z->X->Z) 欧拉角转旋转矩阵
 */
inline Matrix3 euler313ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c1*c3 - s1*c2*s3,   c1*c2*s3 + s1*c3,   s2*s3,
        -s1*c2*c3 - c1*s3,   c1*c2*c3 - s1*s3,   s2*c3,
         s1*s2,             -c1*s2,              c2;
    return R;
}

/**
 * @brief 序列 323 (Z->Y->Z) 欧拉角转四元数
 */
inline Quaternion euler323ToQuat(double t1, double t2, double t3) {
    double c2 = std::cos(t2 * 0.5), s2 = std::sin(t2 * 0.5);
    double c_sum = std::cos((t1 + t3) * 0.5), s_sum = std::sin((t1 + t3) * 0.5);
    double c_diff = std::cos((t1 - t3) * 0.5), s_diff = std::sin((t1 - t3) * 0.5);
    
    return Quaternion(c2*c_sum, -s2*s_diff, c_diff*s2, c2*s_sum);
}

/**
 * @brief 序列 323 (Z->Y->Z) 欧拉角转旋转矩阵
 */
inline Matrix3 euler323ToMatrix(double t1, double t2, double t3) {
    double c1 = std::cos(t1), s1 = std::sin(t1);
    double c2 = std::cos(t2), s2 = std::sin(t2);
    double c3 = std::cos(t3), s3 = std::sin(t3);
    
    Matrix3 R;
    R << c1*c2*c3 - s1*s3,   s1*c2*c3 + c1*s3,  -s2*c3,
        -s1*c3 - c1*c2*s3,   c1*c3 - s1*c2*s3,   s2*s3,
         c1*s2,              s1*s2,              c2;
    return R;
}

// ==================== 统一接口 ====================

/**
 * @brief 欧拉角转四元数（通用接口）
 */
inline Quaternion eulerToQuat(double t1, double t2, double t3, EulerSeq seq) {
    switch (seq) {
        case EulerSeq::XYZ: return euler123ToQuat(t1, t2, t3);
        case EulerSeq::XZY: return euler132ToQuat(t1, t2, t3);
        case EulerSeq::YXZ: return euler213ToQuat(t1, t2, t3);
        case EulerSeq::YZX: return euler231ToQuat(t1, t2, t3);
        case EulerSeq::ZXY: return euler312ToQuat(t1, t2, t3);
        case EulerSeq::ZYX: return euler321ToQuat(t1, t2, t3);
        case EulerSeq::XYX: return euler121ToQuat(t1, t2, t3);
        case EulerSeq::XZX: return euler131ToQuat(t1, t2, t3);
        case EulerSeq::YXY: return euler212ToQuat(t1, t2, t3);
        case EulerSeq::YZY: return euler232ToQuat(t1, t2, t3);
        case EulerSeq::ZXZ: return euler313ToQuat(t1, t2, t3);
        case EulerSeq::ZYZ: return euler323ToQuat(t1, t2, t3);
        default: return Quaternion::identity();
    }
}

/**
 * @brief 欧拉角转旋转矩阵（通用接口）
 */
inline Matrix3 eulerToMatrix(double t1, double t2, double t3, EulerSeq seq) {
    switch (seq) {
        case EulerSeq::XYZ: return euler123ToMatrix(t1, t2, t3);
        case EulerSeq::XZY: return euler132ToMatrix(t1, t2, t3);
        case EulerSeq::YXZ: return euler213ToMatrix(t1, t2, t3);
        case EulerSeq::YZX: return euler231ToMatrix(t1, t2, t3);
        case EulerSeq::ZXY: return euler312ToMatrix(t1, t2, t3);
        case EulerSeq::ZYX: return euler321ToMatrix(t1, t2, t3);
        case EulerSeq::XYX: return euler121ToMatrix(t1, t2, t3);
        case EulerSeq::XZX: return euler131ToMatrix(t1, t2, t3);
        case EulerSeq::YXY: return euler212ToMatrix(t1, t2, t3);
        case EulerSeq::YZY: return euler232ToMatrix(t1, t2, t3);
        case EulerSeq::ZXZ: return euler313ToMatrix(t1, t2, t3);
        case EulerSeq::ZYZ: return euler323ToMatrix(t1, t2, t3);
        default: return Matrix3::Identity();
    }
}

} // namespace gnc::math
