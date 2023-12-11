#ifndef DRIVERLESS_GZ_PARAMS_HPP
#define DRIVERLESS_GZ_PARAMS_HPP

#include <ros/ros.h>
#include <string>

struct ModelParams {

    // Constructor
    ModelParams(boost::shared_ptr<ros::NodeHandle> &nh);

    struct Inertia {
        double m;
        double g;
        double I_z;
        double weight_fact;
        void print() {
            ROS_INFO("Inertia: \n "
                      "\tm: %f\n"
                      "\tg: %f\n"
                      "\tweight_fact: %f\n"
                      "\tI_z: %f", m, g, weight_fact, I_z);
        }
    } inertia;

    struct Kinematic {
        double w_F;
        double w_R;
        double width;
        double length;
        double l_F;
        double l_R;
        double h_cg;
        void print() {
            ROS_INFO("Kinematic: \n "
                      "\tw_F: %f\n"
                      "\tw_R: %f\n"
                      "\twidth: %f\n"
                      "\tlength: %f\n"
                      "\tl_F: %f\n"
                      "\tl_R: %f\n"
                      "\th_cg: %f", w_F, w_R, width, length, l_F, l_R, h_cg);
        }
    } kinematic;

    struct Tire {
        double B;
        double C;
        double D;
        double E;
        double max_steer;
        double radius;
        void print() {
            ROS_INFO("Tire: \n "
                      "\tmax_steer: %f\n"
                      "\tradius: %f\n"
                      "\tB: %f\n"
                      "\tC: %f\n"
                      "\tD: %f\n"
                      "\tE: %f",max_steer, radius, B, C, D, E);
        }
    } tire;

    struct Aero {
        double SCL;
        double SCD;
        void print() {
            ROS_INFO("Aero: \n "
                      "\tSCL: %f\n"
                      "\tSCD: %f", SCL, SCD);
        }
    } aero;

    struct DriveTrain {
        int    nm_wheels;
        double inertia;
        double r_dyn;
        double m_lon_add;
        double cm1;
        double cr0;
        void print() {
            ROS_INFO("DriveTrain: \n "
                      "\tnm_wheels: %i\n"
                      "\tinertia: %f\n"
                      "\tr_dyn: %f\n"
                      "\tCm1: %f\n"
                      "\tCr0: %f", nm_wheels, inertia, r_dyn, cm1, cr0);
        }
    } drivetrain;

    struct Sensors {
        double noise_vx_sigma;
        double noise_vy_sigma;
        double noise_r_sigma;

        void print() {
            ROS_INFO("Sensors: \n "
                      "\tnoise_vx_sigma: %f\n"
                      "\tnoise_vy_sigma: %f\n"
                      "\tnoise_r_sigma: %f", noise_vx_sigma, noise_vy_sigma, noise_r_sigma);
        }
    } sensors;

    struct Topics {
        std::string kin_state;
        std::string dyn_state;
        std::string car_commands;
        std::string ground_truth;

        void print() {
            ROS_INFO("Topics: \n "
                        "\tkin_state: %s\n"
                        "\tdyn_state: %s\n"
                        "\tground_truth: %s\n"
                        "\tcar_commands: %s\n", kin_state.c_str(), dyn_state.c_str(), ground_truth.c_str(), car_commands.c_str());
        }
    } topics;

};

#endif