#ifdef HOOTL_ENABLED
#include "SteeringSim.hpp"
#endif

#include "SteeringInterface.hpp"

std::unique_ptr<SteeringInterface> SteeringInterface::create() {
#ifdef HOOTL_ENABLED
  return std::make_unique<SteeringSim>();
#else
  return nullptr;
#endif
}
