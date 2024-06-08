//write here the trajectory generator: linear and circular

#ifndef VELOCITY_PROFILE_HPP_INCLUDED
#define VELOCITY_PROFILE_HPP_INCLUDED

#include <cmath>
#include <iostream>

namespace velocity_profile
{
    class Profile {
    private:
        // contstant acceleration 
        double q_double_dot;
        // maximum velocity 
        double q_dot_max;
        // start position
        double p_start;
        // end position 
        double p_end;

        // current and initial time
        float time = 0.000f;
        // current velocity
        double q_dot = 0; 
        // tf - total time
        double tf = 0;

        // current position
        double q;

        // if the trajectory has reached the end position
        bool hasEnded = false;
    public:
        Profile(double p_start, double p_end, double q_dot_max, double q_double_dot):
        q_double_dot(q_double_dot), q_dot_max(q_dot_max), p_start(p_start), p_end(p_end) {
            
            // d1 refers to the path of acceleration, and d2 the path of constant velocity period. Only when both of
            // them are positive is the velocity profile trapezoidal

            double d1 = 0.5 * q_double_dot * tc() * tc();
            double d2 = p_end - p_start - 2 * d1;
            if (d2 <= 0) {
                throw std::invalid_argument("Input for the acceleration and speed give non-trapezoidal speed");
            }
            tf = 2 * tc() + d2 / q_dot_max;

            // std::cout << "d1: " << d1 << ", d2: " << d2 << std::endl;

        };
        
        double tc() const {
            // maximum velocity / acceleration
            return q_dot_max / q_double_dot;
        }

        double q_double_dot_current() {
            if (time <= tc()) {
                // acceleration is constant positive
                return q_double_dot;
            } else if (time > tc() && time < tf - tc()) {
                // acceleration is 0, no change in velocity 
                return 0;
            } else {
                // acceleration is decreasing, velocity tends to 0
                return -q_double_dot;
            }
        }
        
        void updatePosition() {
            if (time >= 0 && time <= tc()) {
                q = p_start + 0.5 * q_double_dot * std::pow(time, 2);
            } else if (time > tc() && time <= tf - tc()) {
                q = p_start + q_double_dot * tc() * (time - (tc() * 0.5));
            } else if(tf - tc() < time && time <= tf) {
                q = p_end - 0.5 * q_double_dot * std::pow(tf - time, 2);
            }
        }

        void updateTime(double dt) {
            time += dt;
        }

        void updateVelocity() {
            // update velocity value
            if (time <= tc()) {
                q_dot = time * q_double_dot;
            } else if (time > tf - tc()) {
                q_dot = q_dot_max - (time - (tf - tc())) * q_double_dot;
            }
            // q_dot += q_double_dot_current() * DT_TIME;
        }

        void update(double dt) {
            // update time frame
            if (time + dt >= tf) {
                hasEnded = true;
                return;
            }
            updateTime(dt);
            updatePosition();  
            updateVelocity();      
        }

        // To get the current velocity
        double getQDot() {
            return q_dot;
        }

        double getTime() {
            return time;
        }

        // To get the current position
        double getQ() {
            return q;
        }

        bool isEnded() {
            return hasEnded;
        }
    };
}

#endif  // VELOCITY_PROFILE_HPP_INCLUDED
