#ifndef DRIVERLESS_GZ_PLUGIN_HPP
#define DRIVERLESS_GZ_PLUGIN_HPP 

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "vehicle.hpp"
#include "utils/params.hpp"

namespace gazebo{
namespace driverless{

class FormulaCarPlugin : public ModelPlugin{
  public:

    FormulaCarPlugin();
    ~FormulaCarPlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void OnUpdate();

  private:
    physics::ModelPtr model_;

    physics::WorldPtr world_;

    sdf::ElementPtr sdf_;

    event::ConnectionPtr updateConnection;

    boost::shared_ptr<ros::NodeHandle> nh_;

    VehiclePtr vhcl_ptr_;

    boost::shared_ptr<ModelParams> params_;

    common::Time last_sim_tic;

    bool isSameLoop(const common::Time &time, double &dt);

    void Reboot();

};


}
}

#endif