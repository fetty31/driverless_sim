#ifndef DRIVERLESS_GZ_STRUCTURES_HPP
#define DRIVERLESS_GZ_STRUCTURES_HPP

#include <string>
#include "dv_common/CarState.h"

namespace gazebo {
namespace driverless {

struct State {

    double x, y, yaw;
    double vx, vy, r;
    double ax, ay;

    State() : x(0.0), y(0.0), yaw(0.0), vx(0.0), vy(0.0), r(0.0), ax(0.0), ay(0.0) {}
    State(double x, double y, double yaw, double vx, 
            double vy, double r, double ax, double ay) : x(x), y(y), yaw(yaw),  
                                                        vx(vx), vy(vy), r(r),
                                                        ax(ax), ay(ay) {}

    State operator*(const double &dt) const {
        return {dt * x, dt * y, dt * yaw, dt * vx, dt * vy, dt * r, dt * ax, dt * ay};
    }

    State operator*(double &dt) const {
        return {dt * x, dt * y, dt * yaw, dt * vx, dt * vy, dt * r, dt * ax, dt * ay};
    }

    State operator+(const State &x2) const {
        return {x + x2.x, y + x2.y, yaw + x2.yaw, vx + x2.vx, vy + x2.vy, r + x2.r, ax + x2.ax, ay + x2.ay};
    }

    std::stringstream &operator<<(std::stringstream &os) {
        os << print();
    }

    inline std::string print() const {
        std::string str = "x:" + std::to_string(x)
                          + "| y:" + std::to_string(y)
                          + "| yaw:" + std::to_string(yaw)
                          + "| vx:" + std::to_string(vx)
                          + "| vy:" + std::to_string(vy)
                          + "| r:" + std::to_string(r)
                          + "| ax:" + std::to_string(ax)
                          + "| ay:" + std::to_string(ay);
        return str;
    }

    dv_common::CarState toROS(const ros::Time &time) const {
        dv_common::CarState msg;
        msg.header.stamp = time;
        msg.header.frame_id = "map";
        msg.x = x;
        msg.y = y;
        msg.yaw = yaw;
        msg.vx = vx;
        msg.vy = vy;
        msg.r = r;
        msg.ax = ax;
        msg.ay = ay;
        return msg;
    }

    void validate() { vx = std::max(0.0, vx); }
};

struct Input {
    double throttle, delta;
    Input() : throttle(0.0), delta(0.0) {}
};

struct AxleTires {
    double left;
    double right;

    double avg() const { return (left + right) / 2.0; }
    double sum() const { return left + right; }
};

} // namespace driverless
} // namespace gazebo

#endif