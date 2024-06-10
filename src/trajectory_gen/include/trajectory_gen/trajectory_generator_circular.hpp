//write here the trajectory generator: linear and circular

#ifndef TRAJECTORY_GENERATOR_CIRCULAR_HPP_INCLUDED
#define TRAJECTORY_GENERATOR_CIRCULAR_HPP_INCLUDED

#include <string>
#include <tuple>
#include <math.h>
#include <geometry_msgs/Point.h>
#include "velocity_profile.hpp"
#include "trajectory.hpp"
#include <float.h>
#include "../helpers/triple.hpp"

namespace trajectory_generator
{   
    constexpr double PI = 3.141592653589793238463;

    class CircleTrajectory2d: public Trajectory {
    private:
        // start of the circle as \theta = 0
        triple p_start;
        // end of the circle as \theta = \pi
        triple p_end;
        // center of the circle
        triple c;
        // radious of the circle
        double radius;
        // angular velocity
        double w;
        
        // normal vector
        triple normal_vector_hat;

        // current position 
        triple current_position;

        triple_matrix rotational_matrix;
        // current time 
        double time;

        // end time 
        double end_time;

        // velocity profile
        velocity_profile::Profile *profile;

        // hasEnded bool
        bool has_ended = false;

        void updatePosition();
    public: 
        CircleTrajectory2d(triple c, triple normal_vector_hat, double radius, double q_dot_max, double q_double_dot);  

        void update(double dt, double force = 0) override;
        geometry_msgs::Point getPoint() override;

        bool isEnded() override {
            return has_ended;
        }

        std::string getName() override {
            return "Trajectory::CircleTrajectory2d";
        }

    };
}

#endif  // TRAJECTORY_GENERATOR_CIRCULAR_HPP_INCLUDED
