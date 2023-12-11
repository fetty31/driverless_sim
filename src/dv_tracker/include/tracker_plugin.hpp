#ifndef DRIVERLESS_GZ_TRACKER_PLUGIN_HPP
#define DRIVERLESS_GZ_TRACKER_PLUGIN_HPP 

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "accumulator.hh"
#include "listener.hh"
#include "tracker.hh"

namespace gazebo{
namespace driverless{

class TrackerPlugin : public ModelPlugin{
  public:

    TrackerPlugin();
    ~TrackerPlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate();

  private:
    physics::ModelPtr model_;

    physics::WorldPtr world_;

    sdf::ElementPtr sdf_;

    event::ConnectionPtr updateConnection;

    boost::shared_ptr<ros::NodeHandle> nh_;

    common::Time last_sim_tic;

    TrackerPtr tracker_ptr_;

    bool isSameLoop(const common::Time &time, double &dt);

    void Reboot();

};


}
}

#endif