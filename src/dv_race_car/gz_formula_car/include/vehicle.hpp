#ifndef DRIVERLESS_GZ_VEHICLE_HPP
#define DRIVERLESS_GZ_VEHICLE_HPP

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "utils/params.hpp"
#include "utils/structures.hpp"
#include "utils/noise.hpp"

#include "aero.hpp"
#include "axle.hpp"

#include "dv_common/CarCommands.h"
#include "dv_common/CarState.h"
#include "dv_common/CarDynState.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {
namespace driverless {

class Vehicle {
  public:
    Vehicle(physics::ModelPtr &_parent,
            sdf::ElementPtr &_sdf,
            boost::shared_ptr<ros::NodeHandle> &nh,
            boost::shared_ptr<ModelParams> params);
    
    void update(double dt);

    void reset(boost::shared_ptr<ModelParams> params);

  private:

    // ROS NodeHandle
    boost::shared_ptr<ros::NodeHandle> nh_;

    // ROS publishers
    ros::Publisher pub_state_;
    ros::Publisher pub_dyn_state_;
    ros::Publisher pub_ground_truth_;

    // ROS subscribers
    ros::Subscriber sub_cmd_;

    // ROS TF obj
    tf::TransformBroadcaster tf_br_;

    // Ptr to parent gazebo model
    physics::ModelPtr model_;

    // Car Parameters
    boost::shared_ptr<ModelParams> param_;

    // States
    State state_;
    Input input_;
    double time_last_command_;

    // Front and Rear Axle
    FrontAxle front_axle_;
    RearAxle rear_axle_;

    // Consider Aerodynamics
    Aero aero_;

    // Name of the System
    std::string robot_name_;

    void cmd_callback(const dv_common::CarCommands::ConstPtr &msg);

    void publishTf(const State &x);

    dv_common::CarDynState getDynState(const State &x, const double &Fx, const double &Fz, 
                                          AxleTires *Fy_F, AxleTires *Fy_R, AxleTires *alphaF, AxleTires *alphaR);

    State f(const State &x,
            const Input &u,
            double Fx,
            double M_TV,
            const AxleTires &FyF,
            const AxleTires &FyR);

    State f_kin_correction(const State &x_in,
                           const State &x_state,
                           const Input &u,
                           const double Fx,
                           const double M_TV,
                           const AxleTires &FyF,
                           const AxleTires &FyR,
                           const double dt);

    void setModelState(const State &x);

    double getFx(const State &x, const Input &u);

    double getNormalForce(const State &x);

    State &getState() { return state_; }

    Input &getInput() { return input_; }

};

typedef std::unique_ptr<Vehicle> VehiclePtr;

}
}



#endif