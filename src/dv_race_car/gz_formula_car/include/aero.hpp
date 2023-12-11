#ifndef DRIVERLESS_GZ_AERO_HPP
#define DRIVERLESS_GZ_AERO_HPP

#include "utils/params.hpp"

namespace gazebo {
namespace driverless {

class Aero {
 public:
    explicit Aero(ModelParams::Aero &params) : param_(params) {}

    double getDownForce(const State &x) { return param_.SCL * x.vx * x.vx; }
    double getDragForce(const State &x) { return param_.SCD * x.vx * x.vx; }

 private:
    ModelParams::Aero param_; // Parameters
};

} // namespace driverless
} // namespace gazebo

#endif
