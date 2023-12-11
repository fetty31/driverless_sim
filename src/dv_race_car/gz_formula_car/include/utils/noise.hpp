#ifndef DRIVERLESS_GZ_NOISE_HPP
#define DRIVERLESS_GZ_NOISE_HPP

// STD Includes
#include <random>
#include <ctime>
#include <chrono>

namespace gazebo {
namespace noise {

inline double getGaussianNoise(double mean, double var) {
    std::normal_distribution<double> distribution(mean, var);

    // construct a trivial random generator engine from a time-based seed:
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    
    return distribution(generator);
}

}  // namespace noise
}  // namespace gazebo

#endif 