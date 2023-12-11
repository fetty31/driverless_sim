#ifndef DRIVERLESS_GZ_STEERING_WHEEL_HPP
#define DRIVERLESS_GZ_STEERING_WHEEL_HPP

#include "wheel.hpp"

namespace gazebo {
namespace driverless {

class WheelSteering : public Wheel {
 public:

    WheelSteering(physics::ModelPtr &_model,
                  sdf::ElementPtr &_sdf,
                  const std::string _name,
                  boost::shared_ptr<ros::NodeHandle> &nh,
                  ModelParams::Tire params) :
        Wheel(_model, _sdf, _name, nh, params) {
            
        max_steer_  = params.max_steer;

        std::string full_name = _model->GetName() + "::" + _sdf->Get<std::string>(_name + "_steering");
    }

    bool isSteering() override { return true; }

 private:

    double max_steer_;  // Max steering angle [rad]
};
}
}

#endif
