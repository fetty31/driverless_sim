#include "formula_car_plugin.hpp"

namespace gazebo{
namespace driverless{

FormulaCarPlugin::FormulaCarPlugin(){

    // Init ROS node (should be done?)
    int argc = 0;
    char *argv = nullptr;
    ros::init(argc, &argv, "FormulaCarPlugin");

    // Declare NodeHandle instance
    this->nh_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

}

FormulaCarPlugin::~FormulaCarPlugin(){
    // we use smart ptr, no need to delete
}

void FormulaCarPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    // Set parent model
    this->model_ = _parent;

    // Get world ptr
    this->world_ = model_->GetWorld();

    // Init update callback
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&FormulaCarPlugin::OnUpdate, this));

    // Reset time
    this->last_sim_tic = world_->SimTime();

    // Set vehicle obj
    params_ = boost::shared_ptr<ModelParams>(new ModelParams(nh_));
    this->vhcl_ptr_ = std::unique_ptr<driverless::Vehicle>(new driverless::Vehicle(_parent, _sdf, nh_, params_));

}

void FormulaCarPlugin::OnUpdate(){
    common::Time now = this->world_->SimTime();
    double dt = 0.0;

    if(!isSameLoop(now, dt)) return;

    this->vhcl_ptr_->update(dt); // update vehicle state

    this->last_sim_tic = now;
}

bool FormulaCarPlugin::isSameLoop(const common::Time &time, double &dt){
    dt = (time - this->last_sim_tic).Double();
    if(dt < 0.0){
        this->Reboot();
        return false;
    }else return true;
}

void FormulaCarPlugin::Reboot(){
    // this->updateConnection->Reset(); // deprecated

    this->params_.reset(new ModelParams(nh_)); // read parameters again (maybe they have changed)

    // this->vhcl_ptr_.reset(new driverless::Vehicle(model_, sdf_, nh_, params_)); // set up new vehicle model

    this->vhcl_ptr_->reset(params_); // reset vehicle model

    this->last_sim_tic = this->world_->SimTime();
    ROS_ERROR("RACE CAR: SIMULATION REBOOT SIGNAL APPLIED");
}

} // driverless

GZ_REGISTER_MODEL_PLUGIN(driverless::FormulaCarPlugin)
} // gazebo