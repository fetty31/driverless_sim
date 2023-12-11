#include "tracker_plugin.hpp"

namespace gazebo{
namespace driverless{

TrackerPlugin::TrackerPlugin(){

    // Init ROS node (should be done?)
    int argc = 0;
    char *argv = nullptr;
    ros::init(argc, &argv, "TrackerPlugin");

    // Declare NodeHandle instance
    this->nh_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

}

TrackerPlugin::~TrackerPlugin(){
    // we use smart ptr, no need to delete
}

void TrackerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    // Set parent model
    this->model_ = _parent;

    // Get world ptr
    this->world_ = model_->GetWorld();

    // Init update callback
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&TrackerPlugin::OnUpdate, this));

    // Reset time
    this->last_sim_tic = world_->SimTime();

    // Set up Tracker obj
    this->tracker_ptr_ = std::unique_ptr<driverless::Tracker>(new driverless::Tracker(_parent, _sdf, nh_));

}

void TrackerPlugin::OnUpdate(){
    common::Time now = this->world_->SimTime();
    double dt = 0.0;

    if(!isSameLoop(now, dt)) return;

    this->last_sim_tic = now;
}

bool TrackerPlugin::isSameLoop(const common::Time &time, double &dt){
    dt = (time - this->last_sim_tic).Double();
    if(dt < 0.0){
        this->Reboot();
        return false;
    }else return true;
}

void TrackerPlugin::Reboot(){
    // this->updateConnection->Reset(); // deprecated

    this->tracker_ptr_->Reboot();

    this->last_sim_tic = this->world_->SimTime();
    ROS_ERROR("TRACKER: SIMULATION REBOOT SIGNAL APPLIED");
}

} // driverless

GZ_REGISTER_MODEL_PLUGIN(driverless::TrackerPlugin)
} // gazebo