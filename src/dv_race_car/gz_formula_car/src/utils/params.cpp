#include "utils/params.hpp"

ModelParams::ModelParams(boost::shared_ptr<ros::NodeHandle> &nh){

    // Topics
    nh->param<std::string>("/RaceCar/Topics/Input/Commands",      topics.car_commands,    "/driverless/commands");
    nh->param<std::string>("/RaceCar/Topics/Output/DynState",     topics.dyn_state,       "/driverless/dynamic/state");
    nh->param<std::string>("/RaceCar/Topics/Output/KinState",     topics.kin_state,       "/driverless/kinematic/state");
    nh->param<std::string>("/RaceCar/Topics/Output/GroundTruth",  topics.ground_truth,    "/driverless/ground_truth");

    // Kinematic
    nh->param<double>("/RaceCar/Kinematic/Width/F", kinematic.w_F, 0.7);
    nh->param<double>("/RaceCar/Kinematic/Width/R", kinematic.w_R, 0.7);
    nh->param<double>("/RaceCar/Kinematic/Length/F", kinematic.l_F, 0.8);
    nh->param<double>("/RaceCar/Kinematic/Length/R", kinematic.l_R, 1.3);
    nh->param<double>("/RaceCar/Kinematic/CoG_height", kinematic.h_cg, 0.25);
    kinematic.length = kinematic.l_F + kinematic.l_R;
    kinematic.width = kinematic.w_F + kinematic.w_R;

    // Inertia
    nh->param<double>("/RaceCar/Inertia/Mass", inertia.m, 1000.0);
    nh->param<double>("/RaceCar/Inertia/Gravity", inertia.g, 9.81);
    nh->param<double>("/RaceCar/Inertia/Iz", inertia.I_z, 90.0);
    nh->param<double>("/RaceCar/Inertia/WeightFactor", inertia.weight_fact, 0.55);

    // Tire
    nh->param<double>("/RaceCar/Tire/Pacejka/B", tire.B, 12.56);
    nh->param<double>("/RaceCar/Tire/Pacejka/C", tire.C, -1.38);
    nh->param<double>("/RaceCar/Tire/Pacejka/D", tire.D, 1.6);
    nh->param<double>("/RaceCar/Tire/Pacejka/E", tire.E, -0.58);
    nh->param<double>("/RaceCar/Tire/MaxSteering", tire.max_steer, 0.4);
    nh->param<double>("/RaceCar/Tire/Radius", tire.radius, 0.232);

    // Aero
    double area, density, cd, cl;
    nh->param<double>("/RaceCar/Aero/Area", area, 1.0);
    nh->param<double>("/RaceCar/Aero/AirDensity", density, 1.2);
    nh->param<double>("/RaceCar/Aero/Cd", cd, 0.7);
    nh->param<double>("/RaceCar/Aero/Cl", cl, 1.22);
    aero.SCD = 0.5*area*density*cd;
    aero.SCL = 0.5*area*density*cl;

    // DriveTrain
    nh->param<int>("/RaceCar/DriveTrain/N_wheels", drivetrain.nm_wheels, 4);
    nh->param<double>("/RaceCar/DriveTrain/Inertia", drivetrain.inertia, 0.4);
    nh->param<double>("/RaceCar/DriveTrain/DynRadius", drivetrain.r_dyn, 0.231);
    nh->param<double>("/RaceCar/DriveTrain/MotorFactor", drivetrain.cm1, 2000.0);
    nh->param<double>("/RaceCar/DriveTrain/RollingResistance", drivetrain.cr0, 180.0);
    nh->param<double>("/RaceCar/DriveTrain/AddedMass", drivetrain.m_lon_add, 0.0);

    // Sensors Noise
    nh->param<double>("/RaceCar/Noise/VxSigma", sensors.noise_vx_sigma, 0.0);
    nh->param<double>("/RaceCar/Noise/VySigma", sensors.noise_vy_sigma, 0.0);
    nh->param<double>("/RaceCar/Noise/YawRateSigma", sensors.noise_r_sigma, 0.0);


}