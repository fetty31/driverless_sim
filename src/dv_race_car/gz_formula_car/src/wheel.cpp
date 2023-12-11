#include "wheel.hpp"

namespace gazebo{
namespace driverless{

Wheel::Wheel(gazebo::physics::ModelPtr &_model,
             sdf::ElementPtr &_sdf,
             const std::string _name,
             boost::shared_ptr<ros::NodeHandle> &nh,
             ModelParams::Tire params) : model_(_model), param_(params) {

    full_name_ = _model->GetName() + "::" + _sdf->Get<std::string>(_name);

}

// Compute pacjka model
double Wheel::getFy(const double alpha, const double Fz) {
    const double B    = param_.B;
    const double C    = param_.C;
    const double D    = param_.D;
    const double E    = param_.E;
    const double mu_y = D * std::sin(C * std::atan(B * (1.0 - E) * alpha + E * std::atan(B * alpha)));
    const double Fy   = Fz * mu_y;
    return Fy;
}

}
}