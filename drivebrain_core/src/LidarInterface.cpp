#include "LidarInterface.hpp"
#include <memory>

#ifdef HOOTL_ENABLED
#include "LidarSim.hpp"
#else
#include "Lidar.hpp"
#endif

std::unique_ptr<LidarInterface> LidarInterface::create() {
  
#ifdef HOOTL_ENABLED
  return std::make_unique<LidarSim>();
#else
  return std::make_unique<Lidar>();
#endif
}
