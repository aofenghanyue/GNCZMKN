/**
 * @file test_transform.cpp
 * @brief 坐标转换库 MATLAB 验证测试
 * 
 * 对比 MATLAB checkScripts.m 的结果验证正确性
 */

#include "gnc/common/math/math.hpp"
#include "gnc/libraries/coord/coord.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace gnc::math;
using namespace gnc::coord;

// 容差
constexpr double TOL = 1e-6;

// 比较函数
bool compare(double a, double b, const char* name) {
    double diff = std::abs(a - b);
    bool ok = diff < TOL;
    if (!ok) {
        std::cout << "  [FAIL] " << name << ": C++=" << a << ", MATLAB=" << b 
                  << ", diff=" << diff << std::endl;
    }
    return ok;
}

bool compareVec3(const Vector3& v, double m0, double m1, double m2, const char* name) {
    bool ok = true;
    if (std::abs(v(0) - m0) > TOL) { ok = false; }
    if (std::abs(v(1) - m1) > TOL) { ok = false; }
    if (std::abs(v(2) - m2) > TOL) { ok = false; }
    if (!ok) {
        std::cout << "  [FAIL] " << name << std::endl;
        std::cout << "    C++:    [" << v(0) << ", " << v(1) << ", " << v(2) << "]" << std::endl;
        std::cout << "    MATLAB: [" << m0 << ", " << m1 << ", " << m2 << "]" << std::endl;
    }
    return ok;
}

bool compareQuat(const Quaternion& q, double mw, double mx, double my, double mz, const char* name) {
    bool ok = true;
    if (std::abs(q.w() - mw) > TOL) { ok = false; }
    if (std::abs(q.x() - mx) > TOL) { ok = false; }
    if (std::abs(q.y() - my) > TOL) { ok = false; }
    if (std::abs(q.z() - mz) > TOL) { ok = false; }
    if (!ok) {
        std::cout << "  [FAIL] " << name << std::endl;
        std::cout << "    C++:    [" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]" << std::endl;
        std::cout << "    MATLAB: [" << mw << ", " << mx << ", " << my << ", " << mz << "]" << std::endl;
    }
    return ok;
}

bool compareMat3(const Matrix3& M, 
                 double m00, double m01, double m02,
                 double m10, double m11, double m12,
                 double m20, double m21, double m22,
                 const char* name) {
    bool ok = true;
    if (std::abs(M(0,0) - m00) > TOL) { ok = false; }
    if (std::abs(M(0,1) - m01) > TOL) { ok = false; }
    if (std::abs(M(0,2) - m02) > TOL) { ok = false; }
    if (std::abs(M(1,0) - m10) > TOL) { ok = false; }
    if (std::abs(M(1,1) - m11) > TOL) { ok = false; }
    if (std::abs(M(1,2) - m12) > TOL) { ok = false; }
    if (std::abs(M(2,0) - m20) > TOL) { ok = false; }
    if (std::abs(M(2,1) - m21) > TOL) { ok = false; }
    if (std::abs(M(2,2) - m22) > TOL) { ok = false; }
    if (!ok) {
        std::cout << "  [FAIL] " << name << std::endl;
        std::cout << "    C++:" << std::endl;
        std::cout << "      " << M(0,0) << " " << M(0,1) << " " << M(0,2) << std::endl;
        std::cout << "      " << M(1,0) << " " << M(1,1) << " " << M(1,2) << std::endl;
        std::cout << "      " << M(2,0) << " " << M(2,1) << " " << M(2,2) << std::endl;
        std::cout << "    MATLAB:" << std::endl;
        std::cout << "      " << m00 << " " << m01 << " " << m02 << std::endl;
        std::cout << "      " << m10 << " " << m11 << " " << m12 << std::endl;
        std::cout << "      " << m20 << " " << m21 << " " << m22 << std::endl;
    }
    return ok;
}

int main() {
    std::cout << std::scientific << std::setprecision(8);
    std::cout << "========================================" << std::endl;
    std::cout << "Coordinate Transform MATLAB Verification" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 输入角度 (deg): 30, 45, 60
    double angle1 = deg2rad(30.0);
    double angle2 = deg2rad(45.0);
    double angle3 = deg2rad(60.0);
    
    int total_tests = 0;
    int passed_tests = 0;
    
    // ---------------------------------------------------------
    // 1. 四元数 from 欧拉角
    // ---------------------------------------------------------
    std::cout << "\n1. Quaternion from Angles" << std::endl;
    
    // MATLAB: q321 (ZYX): angle2quat(angle1, angle2, angle3, 'ZYX')
    // MATLAB角度顺序: Z by angle1, Y by angle2, X by angle3
    Quaternion q321 = euler321ToQuat(angle1, angle2, angle3);
    
    // MATLAB结果: [8.22363172e-01, 3.60423406e-01, 4.39679740e-01, 2.22600267e-02]
    total_tests++;
    if (compareQuat(q321, 8.22363172e-01, 3.60423406e-01, 4.39679740e-01, 2.22600267e-02, "q321 (ZYX)")) {
        passed_tests++;
        std::cout << "  [PASS] q321 (ZYX)" << std::endl;
    }
    
    // MATLAB: q231 (YZX): angle2quat(angle1, angle2, angle3, 'YZX')
    Quaternion q231 = euler231ToQuat(angle1, angle2, angle3);
    
    // MATLAB结果: [7.23317411e-01, 5.31975695e-01, 3.91903837e-01, 2.00562121e-01]
    total_tests++;
    if (compareQuat(q231, 7.23317411e-01, 5.31975695e-01, 3.91903837e-01, 2.00562121e-01, "q231 (YZX)")) {
        passed_tests++;
        std::cout << "  [PASS] q231 (YZX)" << std::endl;
    }
    
    // ---------------------------------------------------------
    // 2. DCM from 欧拉角
    // ---------------------------------------------------------
    std::cout << "\n2. DCM from Angles" << std::endl;
    
    Matrix3 T321 = euler321ToMatrix(angle1, angle2, angle3);
    // MATLAB T321 (ZYX):
    // 6.12372436e-01 3.53553391e-01 -7.07106781e-01
    // 2.80330086e-01 7.39198920e-01  6.12372436e-01
    // 7.39198920e-01 -5.73223305e-01 3.53553391e-01
    total_tests++;
    if (compareMat3(T321,
        6.12372436e-01,  3.53553391e-01, -7.07106781e-01,
        2.80330086e-01,  7.39198920e-01,  6.12372436e-01,
        7.39198920e-01, -5.73223305e-01,  3.53553391e-01,
        "T321 (ZYX)")) {
        passed_tests++;
        std::cout << "  [PASS] T321 (ZYX)" << std::endl;
    }
    
    Matrix3 T231 = euler231ToMatrix(angle1, angle2, angle3);
    // MATLAB T231 (YZX):
    // 6.12372436e-01 7.07106781e-01 -3.53553391e-01
    // 1.26826484e-01 3.53553391e-01  9.26776695e-01
    // 7.80330086e-01 -6.12372436e-01 1.26826484e-01
    total_tests++;
    if (compareMat3(T231,
        6.12372436e-01,  7.07106781e-01, -3.53553391e-01,
        1.26826484e-01,  3.53553391e-01,  9.26776695e-01,
        7.80330086e-01, -6.12372436e-01,  1.26826484e-01,
        "T231 (YZX)")) {
        passed_tests++;
        std::cout << "  [PASS] T231 (YZX)" << std::endl;
    }
    
    // ---------------------------------------------------------
    // 3. 向量变换
    // ---------------------------------------------------------
    std::cout << "\n3. Vector Transformation v=[1; 2; 3]" << std::endl;
    
    Vector3 v1(1, 2, 3);
    
    // MATLAB: v' = q* ⊗ v ⊗ q (共轭三明治)
    Vector3 v321_q = q321.rotate(v1);
    // MATLAB: v321_q: [-8.01841127e-01; 3.59584523e+00; 6.53412482e-01]
    total_tests++;
    if (compareVec3(v321_q, -8.01841127e-01, 3.59584523e+00, 6.53412482e-01, "v321_q (quatrotate)")) {
        passed_tests++;
        std::cout << "  [PASS] v321_q (quatrotate)" << std::endl;
    }
    
    // 矩阵乘法
    Vector3 v321_T = T321 * v1;
    // MATLAB: v321_T: [-8.01841127e-01; 3.59584523e+00; 6.53412482e-01]
    total_tests++;
    if (compareVec3(v321_T, -8.01841127e-01, 3.59584523e+00, 6.53412482e-01, "v321_T (T*v)")) {
        passed_tests++;
        std::cout << "  [PASS] v321_T (T*v)" << std::endl;
    }
    
    Vector3 v231_q = q231.rotate(v1);
    // MATLAB: v231_q: [9.65925826e-01; 3.61426335e+00; -6.39353334e-02]
    total_tests++;
    if (compareVec3(v231_q, 9.65925826e-01, 3.61426335e+00, -6.39353334e-02, "v231_q (quatrotate)")) {
        passed_tests++;
        std::cout << "  [PASS] v231_q (quatrotate)" << std::endl;
    }
    
    Vector3 v231_T = T231 * v1;
    // MATLAB: v231_T: [9.65925826e-01; 3.61426335e+00; -6.39353334e-02]
    total_tests++;
    if (compareVec3(v231_T, 9.65925826e-01, 3.61426335e+00, -6.39353334e-02, "v231_T (T*v)")) {
        passed_tests++;
        std::cout << "  [PASS] v231_T (T*v)" << std::endl;
    }
    
    // ---------------------------------------------------------
    // 4. 往返检查 (四元数 -> DCM 一致性)
    // ---------------------------------------------------------
    std::cout << "\n4. Round Trip Check (Quat -> DCM)" << std::endl;
    
    Matrix3 T321_from_q = q321.toRotationMatrix();
    // 应该与 T321 一致
    total_tests++;
    if (compareMat3(T321_from_q,
        6.12372436e-01,  3.53553391e-01, -7.07106781e-01,
        2.80330086e-01,  7.39198920e-01,  6.12372436e-01,
        7.39198920e-01, -5.73223305e-01,  3.53553391e-01,
        "T321 from q321")) {
        passed_tests++;
        std::cout << "  [PASS] T321 from q321" << std::endl;
    }
    
    // ---------------------------------------------------------
    // 5. NSE vs ECEF
    // ---------------------------------------------------------
    std::cout << "\n5. NSE vs ECEF" << std::endl;
    
    double lambda = deg2rad(100.0);  // 经度
    double phi = deg2rad(40.0);      // 纬度
    
    std::cout << "  Params: lambda=100deg, phi=40deg" << std::endl;
    std::cout << "  Rotation Sequence: Z(lam-90) -> X(phi) -> Y(-90)" << std::endl;
    
    // MATLAB使用 angle2dcm(lambda-pi/2, phi, -pi/2, 'ZXY')
    // 序列 ZXY: 先Z后X后Y
    Matrix3 T_nse_ecef = euler312ToMatrix(lambda - constants::HALF_PI, phi, -constants::HALF_PI);
    
    // MATLAB T_NSE_ECEF:
    // 1.11618897e-01 -6.33022222e-01 7.66044443e-01
    // -1.33022222e-01 7.54406507e-01 6.42787610e-01
    // -9.84807753e-01 -1.73648178e-01 4.69066938e-17
    total_tests++;
    if (compareMat3(T_nse_ecef,
        1.11618897e-01, -6.33022222e-01,  7.66044443e-01,
       -1.33022222e-01,  7.54406507e-01,  6.42787610e-01,
       -9.84807753e-01, -1.73648178e-01,  4.69066938e-17,
        "T_NSE_ECEF")) {
        passed_tests++;
        std::cout << "  [PASS] T_NSE_ECEF" << std::endl;
    }
    
    // ---------------------------------------------------------
    // 结果汇总
    // ---------------------------------------------------------
    std::cout << "\n========================================" << std::endl;
    std::cout << "Results: " << passed_tests << "/" << total_tests << " tests passed" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return (passed_tests == total_tests) ? 0 : 1;
}
