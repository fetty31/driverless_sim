#ifndef DRIVERLESS_GZ_TRACKER_HPP
#define DRIVERLESS_GZ_TRACKER_HPP 

#include <ros/ros.h>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "accumulator.hh"
#include "listener.hh"

#include <dv_tracker/Init.h>

#include "dv_common/CarState.h"
#include "dv_common/LapInfoArray.h"

#include "std_msgs/Int32.h"

namespace gazebo{
namespace driverless{

class Tracker {
  public:

    Tracker(physics::ModelPtr &_parent,
            sdf::ElementPtr &_sdf,
            boost::shared_ptr<ros::NodeHandle> &nh);
    ~Tracker();

    void track_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    void state_callback(const dv_common::CarState::ConstPtr& msg);

    bool Reset(dv_tracker::Init::Request &req, dv_tracker::Init::Response &res); 

    void Reboot();

  private:
    physics::ModelPtr model_;

    boost::shared_ptr<ros::NodeHandle> nh_;

    ros::Subscriber state_sub; // not indispensable
    ros::Subscriber track_sub;

    ros::Publisher cones_pub, down_pub, moved_pub, lapinfo_pub;

    ros::ServiceServer reset_srv;

    tracker::AccumPtr accum_ptr_;
    tracker::ListenerPtr listener_ptr_;

    dv_common::LapInfoArray lapInfoArray;
    std_msgs::Float32MultiArray movedArray;

    StateCar state;

    int laps = 0;

    int counter = 0;
    int max_count;

};

typedef std::unique_ptr<Tracker> TrackerPtr;

}
}

#endif