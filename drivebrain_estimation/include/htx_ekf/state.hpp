#pragma once

#include <Eigen/Dense>

namespace htx_ekf
{

struct StateIndex
{
    static constexpr int PN = 0;
    static constexpr int PE = 1;
    static constexpr int YAW = 2;
    static constexpr int VX = 3;
    static constexpr int VY = 4;
    static constexpr int ALPHA = 5;
    static constexpr int BG = 6;
    static constexpr int BAX = 7;
    static constexpr int BAY = 8;

    static constexpr int SIZE = 9;
};

struct State
{
    Eigen::Matrix<double, StateIndex::SIZE, 1> x_;

    double px() const { return x_(StateIndex::PN); }
    double py() const { return x_(StateIndex::PE); }
    double yaw() const { return x_(StateIndex::YAW); }
    double vx() const { return x_(StateIndex::VX); }
    double vy() const { return x_(StateIndex::VY); }
    double alpha() const { return x_(StateIndex::ALPHA); }
    double bg() const { return x_(StateIndex::BG); }
    double bax() const { return x_(StateIndex::BAX); }
    double bay() const { return x_(StateIndex::BAY); }

    double& px() { return x_(StateIndex::PN); }
    double& py() { return x_(StateIndex::PE); }
    double& yaw() { return x_(StateIndex::YAW); }
    double& vx() { return x_(StateIndex::VX); }
    double& vy() { return x_(StateIndex::VY); }
    double& alpha() { return x_(StateIndex::ALPHA); }
    double& bg() { return x_(StateIndex::BG); }
    double& bax() { return x_(StateIndex::BAX); }
    double& bay() { return x_(StateIndex::BAY); }
};

}  // namespace htx_ekf