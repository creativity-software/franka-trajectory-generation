#include "trajectory_gen/velocity_profile.hpp"

namespace velocity_profile
{

Profile::Profile(double p_start, double p_end, double q_dot_max, double q_double_dot_max):
        q_double_dot_max(q_double_dot_max), q_dot_max(q_dot_max), p_start(p_start), p_end(p_end) 
{     
    // d1 refers to the path of acceleration, and d2 the path of constant velocity period. Only when both of
    // them are positive is the velocity profile trapezoidal

    double d1 = 0.5 * q_double_dot_max * tc() * tc();
    double d2 = p_end - p_start - 2 * d1;
    std::cout << "Difference is d1 " << d1 << ", d2" << d2 << "\n";
    if (d2 <= 0) {
        throw std::invalid_argument("Input for the acceleration and speed give non-trapezoidal speed");
    }
    tf = 2 * tc() + d2 / q_dot_max;

    // std::cout << "d1: " << d1 << ", d2: " << d2 << std::endl;
};

double Profile::q_double_dot_current() {
    if (time <= tc()) {
        // acceleration is constant positive
        return q_double_dot_max;
    } else if (time > tc() && time < tf - tc()) {
        // acceleration is 0, no change in velocity 
        return 0;
    } else {
        // acceleration is decreasing, velocity tends to 0
        return -q_double_dot_max;
    }
}

void Profile::updatePosition() {
    if (time >= 0 && time <= tc()) {
        q = p_start + 0.5 * q_double_dot_max * std::pow(time, 2);
        // std::cout << "Q is start " << q << "\n";
    } else if (time > tc() && time <= tf - tc()) {
        q = p_start + q_double_dot_max * tc() * (time - (tc() * 0.5));
    } else if(tf - tc() < time && time <= tf) {
        q = p_end - 0.5 * q_double_dot_max * std::pow(tf - time, 2);
    }
}

void Profile::updateTime(double dt) {
    time += dt;
}

void Profile::updateVelocity() {
    // update velocity value
    if (time <= tc()) {
        q_dot = time * q_double_dot_max;
    } else if (time > tf - tc()) {
        q_dot = q_dot_max - (time - (tf - tc())) * q_double_dot_max;
    }
    // q_dot += q_double_dot_max_current() * DT_TIME;
}

void Profile::update(double dt) {
    // update time frame
    if (time + dt >= tf) {
        hasEnded = true;
        return;
    }
    updateTime(dt);
    updatePosition();  
    updateVelocity();      
}
}
