#pragma once
#include <Eigen/Dense>
namespace htx_ekf
{

struct QuatIndex
{
    static constexpr int W = 0;
    static constexpr int X = 1;
    static constexpr int Y = 2;
    static constexpr int Z = 3;
};

// WGS-84 Geodetic Constants
const double a = 6378137.0;
const double f = 1.0 / 298.257223563;
const double e2 = 2.0 * f - f * f;

double wrap_angle(double angle_rad);

Eigen::Matrix<double, 2, 2> rot_mat_by_angle(double angle_rad);

Eigen::Matrix<double, 4, 1> ang_2d_to_quat(double angle_rad);

Eigen::Vector3d lla_to_ecef(const Eigen::Vector3d& point_lla);

Eigen::Vector2d lla_to_ned_2d(const Eigen::Vector3d& point_lla,
                              const Eigen::Vector3d& origin_lla);

}  // namespace htx_ekf