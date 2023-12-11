#include "vehicle.hpp"

namespace gazebo{
namespace driverless{

Vehicle::Vehicle(physics::ModelPtr &_parent,
            sdf::ElementPtr &_sdf,
            boost::shared_ptr<ros::NodeHandle> &nh,
            boost::shared_ptr<ModelParams> params)
        : model_(_parent),
        nh_(nh),
        param_(params),
        front_axle_(_parent, _sdf, "front", nh, params),
        rear_axle_(_parent, _sdf, "rear", nh, params),
        aero_(params->aero) {
    
    // ROS subscribers
    this->sub_cmd_ = nh->subscribe(param_->topics.car_commands, 1, &Vehicle::cmd_callback, this);

    // ROS publishers
    this->pub_state_        = nh->advertise<dv_common::CarState>(param_->topics.kin_state, 10);
    this->pub_dyn_state_    = nh->advertise<dv_common::CarDynState>(param_->topics.dyn_state, 10);
    this->pub_ground_truth_ = nh->advertise<dv_common::CarState>(param_->topics.ground_truth, 10);

}

void Vehicle::reset(boost::shared_ptr<ModelParams> params){ // workaround to reset simu

    this->param_ = params;

    this->state_ = State();
    this->input_ = Input();

    setModelState(state_);
    publishTf(state_); /*there will be a jump back in time, 
                        ROS TF will probably clear the TF buffer for us*/ 
}

void Vehicle::update(double dt){

    // Current Load
    double Fz = getNormalForce(state_);

    // Lateral Tire Forces
    AxleTires FyF{}, FyR{}, alphaF{}, alphaR{};
    front_axle_.getFy(state_, input_, Fz, FyF, &alphaF);
    rear_axle_.getFy(state_, input_, Fz, FyR, &alphaR);

    // Drivetrain Model
    const double Fx = getFx(state_, input_);

    // Get k+1 state following dynamic bicycle model ODE
    State x_dot_1  = f(state_,                      input_, Fx, 0.0, FyF, FyR);
    State x_dot_2  = f(state_ + x_dot_1 * 0.5 * dt, input_, Fx, 0.0, FyF, FyR);
    State x_dot_3  = f(state_ + x_dot_2 * 0.5 * dt, input_, Fx, 0.0, FyF, FyR);
    State x_dot_4  = f(state_ + x_dot_3 * dt,       input_, Fx, 0.0, FyF, FyR);

    // Integrate (Runge Kutta method)
    const auto x_next_dyn = state_ + (x_dot_1 + x_dot_2 * 2.0 + x_dot_3 * 2.0 + x_dot_4) * (1.0/6.0) * dt;

    // Integrate (forward Euler method)
    // const auto x_next_dyn = state_ + x_dot_1 * dt;

    // Compensate with kinematic correction (only for small velocities)
    state_ = f_kin_correction(x_next_dyn, state_, input_, Fx, 0.0, FyF, FyR, dt);
    state_.validate(); // avoid vx negative values

    // Set next state on gazebo
    setModelState(state_);

    // Update transform
    publishTf(state_);

    // Publish next state
    auto state_msg = state_.toROS(ros::Time::now());
    state_msg.delta = input_.delta;
    this->pub_ground_truth_.publish(state_msg); // without error

    // Add white noise
    state_msg.vx += noise::getGaussianNoise(0.0, param_->sensors.noise_vx_sigma); // no bias
    state_msg.vy += noise::getGaussianNoise(0.0, param_->sensors.noise_vy_sigma);
    state_msg.r  += noise::getGaussianNoise(0.0, param_->sensors.noise_r_sigma);
    this->pub_state_.publish(state_msg); // with error

    // Publish dynamic info
    auto dyn_state_msg = getDynState(state_, Fx, Fz, &FyF, &FyR, &alphaF, &alphaR);
    dyn_state_msg.delta = input_.delta;
    this->pub_dyn_state_.publish(dyn_state_msg);

}

void Vehicle::cmd_callback(const dv_common::CarCommands::ConstPtr &msg){
    this->input_.delta = std::min(param_->tire.max_steer, std::max(msg->delta, -param_->tire.max_steer)) ;
    this->input_.throttle = msg->throttle;

    // ROS_WARN("New command received: delta = %f, throttle = %f", input_.delta, input_.throttle);
    ROS_WARN_ONCE("Receiving commands");

    this->time_last_command_ = ros::Time::now().toSec();
}

// Dynamic bicycle model ODE
State Vehicle::f(const State &x,
                 const Input &u,
                 const double Fx,
                 const double M_TV,
                 const AxleTires &FyF,
                 const AxleTires &FyR) {
    
    // Total front/rear load
    const double FyF_tot = FyF.sum();
    const double FyR_tot = FyR.sum();

    // Add static long. load
    const double m_lon = param_->inertia.m + param_->drivetrain.m_lon_add;

    // State evolution (derivative)
    State x_dot{};
    x_dot.x   = std::cos(x.yaw) * x.vx - std::sin(x.yaw) * x.vy;
    x_dot.y   = std::sin(x.yaw) * x.vx + std::cos(x.yaw) * x.vy;
    x_dot.yaw = x.r;
    x_dot.vx = (x.r * x.vy) + (Fx - std::sin(u.delta) * (FyF_tot)) / m_lon;
    x_dot.vy = ((std::cos(u.delta) * FyF_tot) + FyR_tot) / param_->inertia.m - (x.r * x.vx);
    x_dot.r   = ((std::cos(u.delta) * FyF_tot * param_->kinematic.l_F
                  + std::sin(u.delta) * (FyF.left - FyF.right) * 0.5 * param_->kinematic.w_F)
                 - ((FyR_tot) * param_->kinematic.l_R)
                 + M_TV) / param_->inertia.I_z;
    x_dot.ax = 0;
    x_dot.ay = 0;

    return x_dot;
}

// Kinematic correction (until 3.5 m/s are reached, a kin compensation will be computed)
State Vehicle::f_kin_correction(const State &x_in,
                                const State &x_state,
                                const Input &u,
                                const double Fx,
                                const double M_TV,
                                const AxleTires &FyF,
                                const AxleTires &FyR,
                                const double dt) {
    // Next dyn state (k+1)
    State x = x_in;

    // Long. acceleration
    const double v_x_dot = Fx / (param_->inertia.m + param_->drivetrain.m_lon_add);

    // Velocity norm
    const double v = std::hypot(x_state.vx, x_state.vy);

    // Heuristic kin-dyn blend parameter
    const double v_blend = 0.5 * (v - 1.5);
    const double blend   = std::fmax(std::fmin(1.0, v_blend), 0.0);

    // Final odom state
    x.vx = blend * x.vx + (1.0 - blend) * (x_state.vx + dt * v_x_dot);

    const double v_y = std::tan(u.delta) * x.vx * param_->kinematic.l_R / param_->kinematic.length;
    const double r   = std::tan(u.delta) * x.vx / param_->kinematic.length;

    x.vy = blend * x.vy + (1.0 - blend) * v_y;
    x.r = blend * x.r + (1.0 - blend) * r;

    return x;
}

// Set robot pose/odom into gazebo world (not using any physics engine)
void Vehicle::setModelState(const State &x){
    const ignition::math::Pose3d pose(x.x, x.y, 0.0, 0.0, 0.0, x.yaw);
    const ignition::math::Vector3d vel(x.vx, x.vy, 0.0);
    const ignition::math::Vector3d angular(0.0, 0.0, x.r);
    this->model_->SetWorldPose(pose);
    this->model_->SetAngularVel(angular);
    this->model_->SetLinearVel(vel);
}

// Publish ROS transform
void Vehicle::publishTf(const State &x) {

    // Position
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x.x, x.y, 0.0));

    // Orientation
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, x.yaw);
    transform.setRotation(q);

    // Send TF
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));
}

// Fill dynamic info
dv_common::CarDynState Vehicle::getDynState(const State &x, const double &Fx, const double &Fz, 
                                                AxleTires *Fy_F, AxleTires *Fy_R, AxleTires *alphaF, AxleTires *alphaR){

    dv_common::CarDynState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.vx = x.vx;
    msg.vy = x.vy;
    msg.r = x.r;

    msg.alpha_f   = alphaF->avg();
    msg.alpha_f_l = alphaF->left;
    msg.alpha_f_r = alphaF->right;
    msg.alpha_r   = alphaR->avg();
    msg.alpha_r_l = alphaR->left;
    msg.alpha_r_r = alphaR->right;

    msg.Fy_f   = Fy_F->sum();
    msg.Fy_f_l = Fy_F->left;
    msg.Fy_f_r = Fy_F->right;
    msg.Fy_r   = Fy_R->sum();
    msg.Fy_r_l = Fy_R->left; 
    msg.Fy_r_r = Fy_R->right;

    msg.Fz = Fz;
    msg.Fx = Fx;

    return msg;
}

// Get load
double Vehicle::getNormalForce(const State& x){
    return param_->inertia.g * param_->inertia.m + aero_.getDownForce(x);
}

// Get long. force
double Vehicle::getFx(const State &x, const Input &u) {
    double throttle = (x.vx <= 0.0 && u.throttle < 0.0) ? 0.0 : u.throttle;
    const double dc = std::max(std::min(throttle, 1.0), -1.0);
    const double Fx = dc * param_->drivetrain.cm1 - aero_.getDragForce(x) - param_->drivetrain.cr0;
    return Fx;
}

}
}
