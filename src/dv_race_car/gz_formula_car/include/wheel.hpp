#ifndef DRIVERLESS_GZ_WHEEL_HPP
#define DRIVERLESS_GZ_WHEEL_HPP

#include "ros/ros.h"

// Gazebo Include
#include <gazebo/physics/physics.hh>
#include <gazebo/common/PID.hh>
#include <ignition/math/Pose3.hh>

// STD Include
#include <string>

// Race Car Include
#include "utils/params.hpp"

namespace gazebo {
namespace driverless {

class Wheel {
 public:

    Wheel(physics::ModelPtr &_model,
          sdf::ElementPtr &_sdf,
          std::string _name,
          boost::shared_ptr<ros::NodeHandle> &nh,
          ModelParams::Tire params);

    double getFy(double alpha, double Fz);

    virtual bool isSteering() { return false; }

    virtual void setAngle(const double delta) {}

 private:

    physics::ModelPtr &model_;          // Reference to the model

    std::string full_name_;             // Name of the element

    ModelParams::Tire param_;           // Pacejka Tire parameters

};

}  // namespace driverless
}  // namespace gazebo

#endif
