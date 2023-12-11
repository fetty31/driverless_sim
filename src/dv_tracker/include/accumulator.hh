#ifndef TRACKER_ACCUM_HH
#define TRACKER_ACCUM_HH

#include <ros/ros.h>

#include <eigen3/Eigen/Dense>

#include <std_msgs/Float32MultiArray.h>

#include <dv_common/Cone.h>
#include <dv_common/ConeArray.h>
#include <dv_common/LapInfo.h>

#include <visualization_msgs/MarkerArray.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <vector>
#include <cmath>
#include <set>
// #include <unordered_map>

#include <utils/kdtree.h>
#include <utils/lapcount.hh>
#include <utils/noise.hh>
#include <utils/structs.hh>

namespace gazebo{
namespace driverless{
namespace tracker{

class Accumulator{
  public:
    kdt::KDTree<Point> Tree;
    // std::unordered_map<int, Cone> conemap;
    std::vector<Cone> track;
    std::vector<int> seen_idx;
    std::set<int> moved_idx;

    // Ptr to parent gazebo model
    physics::ModelPtr model_;

    // ROS NodeHandle
    boost::shared_ptr<ros::NodeHandle> nh_;

    // ROS Subscribers
    ros::Subscriber state_sub;
    ros::Subscriber track_sub;

    Lapcount lapcount;
    noise::Gaussian* noise;

    double radius, vision_angle;
    double threshold = 0.1;
    double likelihood;
    double Lf, Lr, T; // length and track of the car
    bool treeFlag = false, stateFlag = false, resetFlag = false, moveConesFlag = false;
    int cones_down = 0; 

    Accumulator(double &mean, double &std);
    Accumulator(physics::ModelPtr &_parent,
            sdf::ElementPtr &_sdf,
            boost::shared_ptr<ros::NodeHandle> &nh);
    ~Accumulator();

    void create_KDTree();

    void fillTracker(const visualization_msgs::MarkerArray::ConstPtr& msg);

    template<typename NUM> NUM mod(NUM x, NUM y);// Norm of (x,y) vector

    bool sameCone(Point p); // Check if the given is the same cone as the one in the accumulator

    void fillConeMsg(int id, dv_common::Cone* msg);

    Point local2global(const Point p, const Eigen::Vector3d& pose);
    Point global2local(const Point p, const Eigen::Vector3d& pose);
    
    bool reset(); // Reset all values (a new Test Run is started)
    
    bool coneDown(int &id, StateCar* state); // Check whether the cone is hit
    
    void moveCone(int &id, StateCar* state); // Move hit cones

    void fillConesDown(std_msgs::Float32MultiArray* msg);

    int getNumConesDown();
    int getLaps();
    
    bool isTreeBuild();
   
    dv_common::LapInfo getLapInfo(); // Get Laptime and Laps according to FSG rules

    bool isInLidarField(int id, StateCar* state); // Check if the cone is in the lidar field of view
    
    void getSeenCones(StateCar* state, dv_common::ConeArray* coneArray); // Get all seen cones

};

typedef std::unique_ptr<Accumulator> AccumPtr;

}
}
}

#endif