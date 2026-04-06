#pragma once

#include "gnc/core/component_base.hpp"
#include "gnc/core/component_factory.hpp"
#include "gnc/interfaces/environment/i_atmosphere_model.hpp"

#include <algorithm>
#include <cmath>

namespace gnc::components {

/**
 * @brief 1976 美国标准大气模型
 *
 * 支持 0 ~ 86 km 高度范围（7 层）。
 * 分层温度线性模型 + 理想气体方程。
 *
 * 各层参数：
 * | 层   | 底高 (m) | 底温 (K) | 梯度 (K/m)  |
 * |------|---------|---------|------------|
 * | 0    | 0       | 288.150 | -0.0065    |  对流层
 * | 1    | 11000   | 216.650 |  0.0       |  平流层下层(等温)
 * | 2    | 20000   | 216.650 |  0.001     |  平流层上层
 * | 3    | 32000   | 228.650 |  0.0028    |  平流层顶
 * | 4    | 47000   | 270.650 |  0.0       |  中间层下层(等温)
 * | 5    | 51000   | 270.650 | -0.0028    |  中间层
 * | 6    | 71000   | 214.650 | -0.002     |  中间层上层
 */
class StandardAtmosphere : public core::ComponentBase,
                           public interfaces::IAtmosphereModel {
public:
    StandardAtmosphere() : ComponentBase("StandardAtmosphere") {
        initLayers();
    }

    void update(double) override {}

    double getDensity(double altitude) const override {
        return getPressure(altitude) / (R_AIR * getTemperature(altitude));
    }

    double getPressure(double altitude) const override {
        const double h = std::clamp(altitude, 0.0, 86000.0);
        const int layer = findLayer(h);
        const auto& L = layers_[layer];
        const double dh = h - L.h_base;

        if (std::abs(L.lapse_rate) < 1e-10) {
            // 等温层：P = P_base * exp(-g0 * dh / (R * T_base))
            return L.p_base * std::exp(-G0 * dh / (R_AIR * L.t_base));
        }
        // 温度梯度层：P = P_base * (T / T_base)^(-g0 / (lapse * R))
        const double T = L.t_base + L.lapse_rate * dh;
        const double exponent = -G0 / (L.lapse_rate * R_AIR);
        return L.p_base * std::pow(T / L.t_base, exponent);
    }

    double getTemperature(double altitude) const override {
        const double h = std::clamp(altitude, 0.0, 86000.0);
        const int layer = findLayer(h);
        return layers_[layer].t_base + layers_[layer].lapse_rate * (h - layers_[layer].h_base);
    }

    double getSpeedOfSound(double altitude) const override {
        return std::sqrt(GAMMA_AIR * R_AIR * getTemperature(altitude));
    }

private:
    // 物理常数
    static constexpr double G0        = 9.80665;      // m/s²
    static constexpr double R_AIR     = 287.05287;     // J/(kg·K)
    static constexpr double GAMMA_AIR = 1.4;           // 比热比

    static constexpr int NUM_LAYERS = 7;

    struct Layer {
        double h_base;      // 层底高度 (m)
        double t_base;      // 层底温度 (K)
        double lapse_rate;   // 温度梯度 (K/m)
        double p_base;      // 层底压力 (Pa)
    };

    Layer layers_[NUM_LAYERS]{};

    /// 运行时计算各层底压力（从第 0 层向上递推）
    void initLayers() {
        // 层底高度、温度、梯度（常量数据）
        constexpr double h[] = {0, 11000, 20000, 32000, 47000, 51000, 71000};
        constexpr double t[] = {288.150, 216.650, 216.650, 228.650, 270.650, 270.650, 214.650};
        constexpr double lr[] = {-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002};

        layers_[0] = {h[0], t[0], lr[0], 101325.0};

        for (int i = 1; i < NUM_LAYERS; ++i) {
            const double dh = h[i] - h[i - 1];
            double p;
            if (std::abs(lr[i - 1]) < 1e-10) {
                p = layers_[i - 1].p_base * std::exp(-G0 * dh / (R_AIR * t[i - 1]));
            } else {
                const double T_top = t[i - 1] + lr[i - 1] * dh;
                const double exponent = -G0 / (lr[i - 1] * R_AIR);
                p = layers_[i - 1].p_base * std::pow(T_top / t[i - 1], exponent);
            }
            layers_[i] = {h[i], t[i], lr[i], p};
        }
    }

    int findLayer(double h) const {
        for (int i = NUM_LAYERS - 1; i > 0; --i) {
            if (h >= layers_[i].h_base) return i;
        }
        return 0;
    }
};

GNC_REGISTER_COMPONENT(StandardAtmosphere, interfaces::IAtmosphereModel)

} // namespace gnc::components
