//write here the trajectory generator: linear and circular

#ifndef TRAJECTORY_GENERATOR_HPP_INCLUDED
#define TRAJECTORY_GENERATOR_HPP_INCLUDED

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
    public:
        LinearTrajectory(triple c, triple normal_vector_hat , triple p_start, triple p_end, double q_dot_max, double q_double_dot, double q_dot = 0, double force_threshold = 0): 
            c(c), normal_vector_hat(normal_vector_hat) , q_dot(q_dot), current_position(p_start), force_threshold(force_threshold) {
            y_hat = triple_cross_product(normal_vector_hat , c) / triple_norm(triple_cross_product(normal_vector_hat, c));
            x_hat = triple_cross_product(y_hat, normal_vector_hat);
            rotational_matrix = std::make_tuple(x_hat,y_hat,normal_vector_hat);
            this->p_start = rotational_matrix * p_start + c;
            this->p_end = rotational_matrix * p_end + c;
            triple diff = this->p_end - this->p_start;
            initial_diff = triple_norm(diff);
            unit_vector = diff / initial_diff;

            // double p_start, double p_end, double q_dot_max, double q_double_dot
            profile = new velocity_profile::Profile(0, initial_diff, q_dot_max , q_double_dot);
         };

        void updatePosition() {
            // current position calculation
            current_position = p_start + unit_vector * profile->getQ();
            // print_triple(current_position, "Update [position]::");
        }

        // To get the distance between the start point and the end point
        double current_diff() {
            triple diff = p_start - current_position;
            return triple_norm(diff);
        }

        void update(double dt, double force = 0) override {
            // To stop linear movement once the force is beyond the threshold
            std::cout << "Force "  << force << ", f_th" << force_threshold << "\n";
            time += dt;
            profile->update(dt);
            updatePosition();
            if (force_threshold && force > force_threshold) {
                has_ended = true;
                return;
            }

            std::cout << "Getq" << profile->getQ() << ", initial diff" << initial_diff << "\n";
            if (precision(profile->getQ(), 3) >= precision(initial_diff, 3)) {
                // std::cout << "Diff reset" << std::endl;
                has_ended = true;
                return;
            }
        }

        bool isEnded() override {
            return has_ended;
        }

        void setForceThreshold(double th) {
            force_threshold = th;
        }

        geometry_msgs::Point getPoint() override {
            geometry_msgs::Point point;
            point.x = std::get<0>(current_position);
            point.y = std::get<1>(current_position);
            point.z = std::get<2>(current_position);
            return point;
        }

        std::string getName() override {
            return "Trajectory::LinearTrajectory";
        }
    };


    const double PI = 3.141592653589793238463;

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
    public: 
        CircleTrajectory2d(triple c, triple normal_vector_hat, double radius, double q_dot_max, double q_double_dot): 
        c(c), radius(radius), w(w), normal_vector_hat(normal_vector_hat) {
            triple y_hat = triple_cross_product(normal_vector_hat , c)  / triple_norm(triple_cross_product(normal_vector_hat, c));
            triple x_hat = triple_cross_product(y_hat, normal_vector_hat);

            // create start point relative to the center to draw a circle
            triple start_hat = std::make_tuple(radius, 0, 0);

            // The rotational matrix serves for transforming the position in the end effector coordination system back to the base coordination system
            rotational_matrix = std::make_tuple(x_hat,y_hat,normal_vector_hat);

            //Now position in the base coordination system has been obtained; c refers to the vector from origin of the base to the origin of the end effector
            p_start = rotational_matrix * start_hat + c; 

            // In order to use the previously defined trapezoidal velocity
            this->profile = new velocity_profile::Profile(0, 2 * PI * radius, q_dot_max , q_double_dot);
            end_time = 2 * PI / w;
        }   

        void updatePosition() {
            double q = profile->getQ();
            
            // This variable refers to the ratio of the travelled path to the radius 
            double portion = q / radius;

            // In this way we can get the exact position of the end effector while drawing a circle
            triple current_position_path = std::make_tuple(
                 std::cos(portion) * radius ,
                 radius * std::sin(portion),
                 0
            );
            current_position =  rotational_matrix * current_position_path + c; 
        }


        void update(double dt, double force = 0) override {
            std::cout << "Circular z" << std::get<2>(current_position) << "\n";
            if (precision(profile->getQ(), 3) >= precision(2 * PI * radius, 3)) {
                has_ended = true;
                return;
            }
            time += dt;
            profile->update(dt);
            updatePosition();
        }

          bool isEnded() override {
            return has_ended;
        }


        geometry_msgs::Point getPoint() override {
            geometry_msgs::Point point;
            point.x = std::get<0>(current_position);
            point.y = std::get<1>(current_position);
            point.z = std::get<2>(current_position);
            return point;
        }

        std::string getName() override {
            return "Trajectory::CircleTrajectory2d";
        }

    };
}

#endif  // TRAJECTORY_GENERATOR_HPP_INCLUDED
