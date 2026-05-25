#include "htx_ekf/math_utils.hpp"

#include <cmath>

namespace htx_ekf
{

double wrap_angle(double angle_rad)
{
    while (angle_rad > M_PI)
    {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < -M_PI)
    {
        angle_rad += 2.0 * M_PI;
    }

    return angle_rad;
}

Eigen::Matrix<double, 2, 2> rot_mat_by_angle(double angle_rad)
{
    Eigen::Matrix<double, 2, 2> R;
    R.setZero();

    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);

    R << c, -s, s, c;
    return R;
}

Eigen::Matrix<double, 4, 1> ang_2d_to_quat(double angle_rad)
{
    Eigen::Matrix<double, 4, 1> quat;
    quat.setZero();

    quat(QuatIndex::W) = std::cos(angle_rad / 2.0);
    quat(QuatIndex::Z) = std::sin(angle_rad / 2.0);

    return quat;
}

Eigen::Vector3d lla_to_ecef(const Eigen::Vector3d& point_lla)
{
    double lat_deg = point_lla(0);
    double lon_deg = point_lla(1);
    double alt_m = point_lla(2);

    double lat_rad = lat_deg * M_PI / 180.0;
    double lon_rad = lon_deg * M_PI / 180.0;

    double N = a / std::sqrt(1.0 - e2 * std::sin(lat_rad) * std::sin(lat_rad));

    double x = (N + alt_m) * std::cos(lat_rad) * std::cos(lon_rad);
    double y = (N + alt_m) * std::cos(lat_rad) * std::sin(lon_rad);
    double z = (N * (1.0 - e2) + alt_m) * std::sin(lat_rad);

    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector2d lla_to_ned_2d(const Eigen::Vector3d& point_lla,
                              const Eigen::Vector3d& origin_lla)
{
    Eigen::Vector3d point_ecef = lla_to_ecef(point_lla);
    Eigen::Vector3d origin_ecef = lla_to_ecef(origin_lla);

    Eigen::Vector3d d_ecef = point_ecef - origin_ecef;

    double lat0_rad = origin_lla(0) * M_PI / 180.0;
    double lon0_rad = origin_lla(1) * M_PI / 180.0;

    double sin_lat = std::sin(lat0_rad);
    double cos_lat = std::cos(lat0_rad);
    double sin_lon = std::sin(lon0_rad);
    double cos_lon = std::cos(lon0_rad);

    Eigen::Matrix3d R;
    R << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, -sin_lon, cos_lon,
        0.0, -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;

    Eigen::Vector3d ned_3d = R * d_ecef;

    return ned_3d.head<2>();
}

}  // namespace htx_ekf