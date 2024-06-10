#ifndef TRAJECTORY_GENERATOR_LINEAR_HPP_INCLUDED
#define TRAJECTORY_GENERATOR_LINEAR_HPP_INCLUDED

#include <string>
#include <tuple>
#include <math.h>
#include <geometry_msgs/Point.h>
#include "velocity_profile.hpp"
#include "trajectory.hpp"
#include "../helpers/triple.hpp"

namespace trajectory_generator
{   
    class LinearTrajectory: public Trajectory {
    private:
        // start position
        triple p_start;
        // end position 
        triple p_end;
        // constant velocity 
        double q_dot;

        triple vec;

        triple current_position;

        triple unit_vector;

        double initial_diff;

        double time = 0; 
        // difference between the base point and the end effector point
        triple c;
        // Normal vector(normal_vector_hat coordinate)
        triple normal_vector_hat;
        // y coordinate
        triple y_hat;
        // x coordinate
        triple x_hat;
        // rotational matrix
        triple_matrix rotational_matrix;
        // velocity profile
        velocity_profile::Profile *profile;

        double force_threshold;
        
        bool has_ended = false;

        void updatePosition();
        // To get the distance between the start point and the end point
        double current_diff();
    public:
        LinearTrajectory(triple c, triple normal_vector_hat , triple p_start, triple p_end, double q_dot_max, double q_double_dot, double q_dot = 0, double force_threshold = 0);

        void update(double dt, double force = 0) override;
        geometry_msgs::Point getPoint() override;

        void setForceThreshold(double th);

        bool isEnded() override {
            return has_ended;
        }

        std::string getName() override {
            return "Trajectory::LinearTrajectory";
        }
    };
}


#endif // TRAJECTORY_GENERATOR_LINEAR_HPP_INCLUDED