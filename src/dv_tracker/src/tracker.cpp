#include "tracker_plugin.hpp"

namespace gazebo{
namespace driverless{

Tracker::Tracker(physics::ModelPtr &_parent,
            sdf::ElementPtr &_sdf,
            boost::shared_ptr<ros::NodeHandle> &nh) 
        : model_(_parent),
        nh_(nh) {

    // Set up params
    std::string state_topic, track_topic, cones_topic, down_topic, laptime_topic, moved_topic, reset_topic;
    nh_->param<std::string>("/Tracker/Topics/Input/State",     state_topic,    "/driverless/state");
    nh_->param<std::string>("/Tracker/Topics/Input/Track",     track_topic,    "/driverless/track");
    nh_->param<std::string>("/Tracker/Topics/Output/Cones",    cones_topic,    "/tracker/cones");
    nh_->param<std::string>("/Tracker/Topics/Output/ConesDown",down_topic,     "/tracker/conesdown");
    nh_->param<std::string>("/Tracker/Topics/Output/Laptime",  laptime_topic,  "/tracker/lapinfo");
    nh_->param<std::string>("/Tracker/Topics/Output/Moved",    moved_topic,    "/tracker/moved");
    nh_->param<std::string>("/Tracker/Service/Reset",   reset_topic,    "/tracker/reset");

    float freq;
    nh_->param<float>("/Tracker/Frequency", freq, 50.0);
    max_count = int(1000/freq);

    // Subscribers & Publishers
    state_sub = nh_->subscribe(state_topic, 10, &Tracker::state_callback, this);
    track_sub = nh_->subscribe(track_topic, 5, &Tracker::track_callback, this);

    cones_pub   = nh_->advertise<dv_common::ConeArray>(cones_topic, 1);
    down_pub    = nh_->advertise<std_msgs::Int32>(down_topic, 1);
    lapinfo_pub = nh_->advertise<dv_common::LapInfoArray>(laptime_topic, 1);
    moved_pub   = nh_->advertise<std_msgs::Float32MultiArray>(moved_topic, 1);

    // Bring up reset service
    reset_srv = nh_->advertiseService(reset_topic, &Tracker::Reset , this);

    // Set Accumulator & Listener obj
    accum_ptr_    = std::unique_ptr<driverless::tracker::Accumulator>(new driverless::tracker::Accumulator(_parent, _sdf, nh_));
    listener_ptr_ = std::unique_ptr<driverless::tracker::Listener>(new driverless::tracker::Listener("map", "lidar"));

}

Tracker::~Tracker(){
    // we use smart ptr, no need to delete
}

void Tracker::state_callback(const dv_common::CarState::ConstPtr& msg){

    if(counter >= max_count){

        // Global pose
        state.pose(0) = msg->x;
        state.pose(1) = msg->y;
        state.pose(2) = msg->yaw;

        // Global velocity
        state.velocity(0) = msg->vx;
        state.velocity(1) = msg->vy;
        state.velocity(2) = msg->r;

        // Heading
        state.heading = msg->yaw;

        // Call TF transform
        listener_ptr_->lookup_tf();
        state.lidar_pose(0) = listener_ptr_->position.x;
        state.lidar_pose(1) = listener_ptr_->position.y;
        state.lidar_pose(2) = listener_ptr_->yaw;

        // Publish Seen Cones
        if(accum_ptr_->isTreeBuild()){
            dv_common::ConeArray coneArrayMsg = dv_common::ConeArray();
            accum_ptr_->getSeenCones(&state, &coneArrayMsg);
            cones_pub.publish(coneArrayMsg);

            std_msgs::Int32 conesdown_msg;
            conesdown_msg.data = accum_ptr_->getNumConesDown();
            down_pub.publish(conesdown_msg);
        }

        // Publish Lap Info
        if(accum_ptr_->getLaps() > this->laps){
            lapInfoArray.info.push_back(accum_ptr_->getLapInfo());
            this->laps++;
        }
        lapinfo_pub.publish(lapInfoArray);

        counter = 0;

    }else counter++;

}

void Tracker::track_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){

    if(!accum_ptr_->isTreeBuild()) accum_ptr_->fillTracker(msg);

    // Published moved cones new positions
    accum_ptr_->fillConesDown(&movedArray);
    moved_pub.publish(movedArray);

}

bool Tracker::Reset(dv_tracker::Init::Request &req, dv_tracker::Init::Response &res){

    ROS_WARN("TRACKER: Reseting accumulator...");

    res.success = accum_ptr_->reset(); // Reset accumulator obj
    this->lapInfoArray = dv_common::LapInfoArray(); // Reset lap info array
    this->laps = 0;
    this->counter = 0;
    
    return true;
}

void Tracker::Reboot(){

    // TO DO
    //      - read parameters again (not strictly necessary)

    this->lapInfoArray = dv_common::LapInfoArray();
    this->laps = 0;
    this->counter = 0;

    listener_ptr_.reset(new driverless::tracker::Listener("map", "lidar"));
    if(accum_ptr_->reset()) ROS_WARN("TRACKER: Reboot");
    else ROS_ERROR("TRACKER: An error ocurred when rebooting the Accumulator");

}

} // driverless
} // gazebo