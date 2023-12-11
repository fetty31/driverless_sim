#include "axle.hpp"

namespace gazebo {
namespace driverless{

template<class WheelType>
Axle<WheelType>::Axle(physics::ModelPtr &_model,
                      sdf::ElementPtr &_sdf,
                      const std::string name,
                      boost::shared_ptr<ros::NodeHandle> &nh,
                      boost::shared_ptr<ModelParams> params)
            : wheel_l_(_model, _sdf, name + "_left_wheel", nh, params->tire),
            wheel_r_(_model, _sdf, name + "_right_wheel", nh, params->tire),
            name_(name) {
    
    if(name == "front"){
        this->axle_factor_ = 1;
        this->cog_dist_ = params->kinematic.l_F;
        this->axle_width_ = params->kinematic.w_F;
        this->weight_factor_ = params->inertia.weight_fact;
    }else{
        this->axle_factor_ = -1;
        this->cog_dist_ = params->kinematic.l_R;
        this->axle_width_ = params->kinematic.w_R;
        this->weight_factor_ = 1.0 - params->inertia.weight_fact;
    }

}

template<class WheelType>
void Axle<WheelType>::getSlipAngles(const State &x, const Input &u, double &alphaL, double &alphaR) {
    // double v_x = std::max(1.0, x->v_x);
    double v_x = x.vx;
    alphaL = std::atan2((x.vy + axle_factor_ * cog_dist_ * x.r), (v_x - 0.5 * axle_width_ * x.r))
             - u.delta * wheel_l_.isSteering();
    alphaR = std::atan2((x.vy + axle_factor_ * cog_dist_ * x.r), (v_x + 0.5 * axle_width_ * x.r))
             - u.delta * wheel_r_.isSteering();
}

template<class WheelType>
void Axle<WheelType>::getFy(const State &x, const Input &u, const double Fz, AxleTires &Fy, AxleTires *alpha) {
    double alphaL, alphaR;
    getSlipAngles(x, u, alphaL, alphaR);

    const double Fz_wheel = getDownForce(Fz);

    Fy.left  = wheel_l_.getFy(alphaL, Fz_wheel);
    Fy.right = wheel_r_.getFy(alphaR, Fz_wheel);

    if (alpha != nullptr) {
        alpha->left  = alphaL;
        alpha->right = alphaR;
    }
}

template<class WheelType>
double Axle<WheelType>::getDownForce(const double Fz) {
    double Fz_wheel = 0.5 * weight_factor_ * Fz;
    return Fz_wheel;
}

template
class Axle<WheelSteering>;
template
class Axle<Wheel>;

} // driverless
} // gazebo