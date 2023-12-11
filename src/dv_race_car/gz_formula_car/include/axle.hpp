#ifndef DRIVERLESS_GZ_AXLE_HPP
#define DRIVERLESS_GZ_AXLE_HPP

#include "steering_wheel.hpp"
#include "utils/structures.hpp"

namespace gazebo {
namespace driverless {

template<class WheelType>
class Axle {
 public:

    Axle(physics::ModelPtr &_model,
         sdf::ElementPtr &_sdf,
         const std::string name,
         boost::shared_ptr<ros::NodeHandle> &nh,
         boost::shared_ptr<ModelParams> params);

    void getSlipAngles(const State &x, const Input &u, double &alphaL, double &alphaR);

    void getFy(const State &x, const Input &u, const double Fz, AxleTires &Fy, AxleTires *alpha = nullptr);

    double getDownForce(const double Fz);

    const WheelType &getWheelLeft() const { return wheel_l_; }
    const WheelType &getWheelRight() const { return wheel_r_; }

 private:
    // Name of the model
    std::string name_;

    // Left and Right wheel
    WheelType wheel_l_;
    WheelType wheel_r_;

    // Parameters
    double axle_width_;        // Width [m]
    double cog_dist_;          // CoG to axle dist [m]
    double weight_factor_;     // Weight distribution [m]

    int axle_factor_;      // Decides whether its front or rear [no units]

};

typedef Axle<Wheel>         RearAxle;
typedef Axle<WheelSteering> FrontAxle;

} // namespace driverlesss
} // namespace gazebo

#endif
