//write here the trajectory generator: linear and circular

#ifndef VELOCITY_PROFILE_HPP_INCLUDED
#define VELOCITY_PROFILE_HPP_INCLUDED

#include <cmath>
#include <iostream>

namespace velocity_profile
{
    /**
    * Trapezoidal Velocity Profile
    * General idea is to accerelate and descerelate speed on certain 
    * areas of the path for a smoother movement.
    * For more information here:  
    * https://www.pmdcorp.com/resources/type/articles/get/mathematics-of-motion-control-profiles-article
    */
    class Profile {
    private:
        // constant acceleration 
        const double q_double_dot_max;
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

        void updatePosition();
        void updateTime(double dt);
        void updateVelocity();
    public:
        Profile(double p_start, double p_end, double q_dot_max, double q_double_dot_max);

        double q_double_dot_current();

        void update(double dt);

        inline double tc() const {
            // maximum velocity / acceleration
            return q_dot_max / q_double_dot_max;
        }

        // To get the current velocity
        inline double getQDot() const {
            return q_dot;
        }

        inline double getTime() const {
            return time;
        }

        // To get the current position
        inline double getQ() const {
            return q;
        }

        inline bool isEnded() const {
            return hasEnded;
        }
    };
}

#endif  // VELOCITY_PROFILE_HPP_INCLUDED
