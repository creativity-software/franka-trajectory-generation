//write here the trajectory generator: linear and circular

#ifndef TRAJECTORY_GENERATOR_HPP_INCLUDED
#define TRAJECTORY_GENERATOR_HPP_INCLUDED
#include <tuple>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <string>
#include "velocity_profile.hpp"
namespace trajectory_generator
{
    using triple = std::tuple<double, double, double>;
    using triple_matrix = std::tuple<triple,triple,triple>;

    triple triple_minus(triple p_end, triple p_start) {
        return std::make_tuple(
           std::get<0>(p_end) - std::get<0>(p_start),
           std::get<1>(p_end) - std::get<1>(p_start),
           std::get<2>(p_end) - std::get<2>(p_start)
        );
    };

    triple triple_plus(triple p_end, triple p_start) {
        return std::make_tuple(
           std::get<0>(p_end) + std::get<0>(p_start),
           std::get<1>(p_end) + std::get<1>(p_start),
           std::get<2>(p_end) + std::get<2>(p_start)
        );
    };

    // |V| = sqrt(x*x + y*y + normal_vector_hat*normal_vector_hat) 
    double triple_norm(triple p) {
        return std::sqrt(
           std::get<0>(p) * std::get<0>(p) + 
           std::get<1>(p) * std::get<1>(p) + 
           std::get<2>(p) * std::get<2>(p)
        );
    }
    

    triple triple_cross_product(triple vector_one, triple vector_two){

         return std::make_tuple(
           std::get<1>(vector_one) * std::get<2>(vector_two) - std::get<1>(vector_two) * std::get<2>(vector_one),
           std::get<2>(vector_one) * std::get<0>(vector_two) - std::get<0>(vector_one) * std::get<2>(vector_two),
           std::get<0>(vector_one) * std::get<1>(vector_two) - std::get<0>(vector_two) *  std::get<1>(vector_one)
        );
            
    }

    triple triple_divide(triple tr, double num) {
        return std::make_tuple(
           std::get<0>(tr) / num,
           std::get<1>(tr) / num,
           std::get<2>(tr) / num
        );
    }

    triple triple_mul(triple tr, double num) {
        return std::make_tuple(
           std::get<0>(tr) * num,
           std::get<1>(tr) * num,
           std::get<2>(tr) * num
        );
    }
    

    bool is_bigger(triple p_start, triple p_end) {
        return std::get<0>(p_end) <= std::get<0>(p_start) &&
        std::get<1>(p_end) <= std::get<1>(p_start) &&
        std::get<2>(p_end) <= std::get<2>(p_start);
    }

    void print_triple(triple tr, std::string str = "") {
        std::cout << str << "Triple: x =" << std::get<0>(tr)
         << ", y = " << std::get<1>(tr) 
         << ", normal_vector_hat = " << std::get<2>(tr) << "\n";
    }
       triple dot_product(triple_matrix rotational_matrix , triple p_start){
        return triple_plus(
            triple_mul(std::get<0>(rotational_matrix),std::get<0>(p_start)),
            triple_plus(triple_mul(std::get<1>(rotational_matrix),std::get<1>(p_start)),
            triple_mul(std::get<2>(rotational_matrix),std::get<2>(p_start)) )
            );
       }
    
    double precision( double f, int places )
    {
        double n = std::pow(10.0d, places ) ;
        return std::round(f * n) / n ;
    }
    class Trajectory {
    public:
        virtual ~Trajectory() {}
        virtual void update(double dt) = 0;
        virtual bool isEnded() = 0;
        virtual geometry_msgs::Point getPoint() = 0;
    };

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

        
        bool has_ended = false;
    public:
        LinearTrajectory(triple c, triple normal_vector_hat , triple p_start, triple p_end,double q_dot_max,double q_double_dot, double q_dot = 0): 
            c(c), normal_vector_hat(normal_vector_hat) , q_dot(q_dot), current_position(p_start) {
            y_hat = triple_divide(triple_cross_product(normal_vector_hat , c) , triple_norm(triple_cross_product(normal_vector_hat, c)) );
            x_hat = triple_cross_product(y_hat, normal_vector_hat);
            rotational_matrix = std::make_tuple(x_hat,y_hat,normal_vector_hat);
            this->p_start =  triple_plus(dot_product(rotational_matrix,p_start),c);
            this->p_end = triple_plus(dot_product(rotational_matrix,p_end),c);
            triple diff = triple_minus(this->p_end, this->p_start);
            initial_diff = triple_norm(diff);
            unit_vector = triple_divide(diff, initial_diff);

            // double p_start, double p_end, double q_dot_max, double q_double_dot
            profile = new velocity_profile::Profile(0, initial_diff, q_dot_max , q_double_dot);

            
            // std::cout << "diff: " << initial_diff << " " << std::endl;
            // print_triple(unit_vector , "Unit Vector: ");
            // print_triple(this->p_start , " p_start:");
            // print_triple(this->p_end , " p_end:");


            // print_triple(y_hat , " y_hat:::" );
            // print_triple(x_hat , " x_hat:::" );
            // print_triple(std::get<0>(rotational_matrix), "rotational_matrix_x");
            // print_triple(std::get<1>(rotational_matrix), "rotational_matrix_y");
            // print_triple(std::get<2>(rotational_matrix), "rotational_matrix_normal_vector_hat");
            // print_triple(unit_vector, "Unit vector:::");
            // print_triple(p_start, "P_start vector:::");
            // print_triple(p_end, "P_end vector:::");
         };

        void updatePosition() {
            // current position calculation
            current_position = triple_plus(p_start, triple_mul(unit_vector, profile->getQ()));
            // print_triple(current_position, "Update [position]::");
        }

        double current_diff() {
            triple diff = triple_minus(p_start, current_position);
            return triple_norm(diff);
        }

        void update(double dt) override {
            std::cout << "Current [diff]::" << profile->getQ() << "\n";
            std::cout << "Current [init_diff]::" << initial_diff << "\n";
            if (precision(profile->getQ(), 3) >= precision(initial_diff, 3)) {
                std::cout << "Diff reset" << std::endl;
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
        bool has_ended;
    public: 
        CircleTrajectory2d(triple c, triple normal_vector_hat, double radius, double q_dot_max, double q_double_dot): 
        c(c), radius(radius), w(w), normal_vector_hat(normal_vector_hat) {
            triple y_hat = triple_divide(triple_cross_product(normal_vector_hat , c) , triple_norm(triple_cross_product(normal_vector_hat, c)) );
            triple x_hat = triple_cross_product(y_hat, normal_vector_hat);

            // create start point relative to the center
            triple start_hat = std::make_tuple(radius, 0, 0);
            // transform stat point back to the origin 
            rotational_matrix = std::make_tuple(x_hat,y_hat,normal_vector_hat);

            p_start = triple_plus(dot_product(rotational_matrix, start_hat), c); 
            // current_position = std::make_tuple(
            //     std::get<0>(p_start),
            //     std::get<1>(p_start),
            //     std::get<2>(p_start)
            // );

            this->profile = new velocity_profile::Profile(0, 2 * PI * radius, q_dot_max , q_double_dot);
            end_time = 2 * PI / w;
        }   

        void updatePosition() {
            double q = profile->getQ();
            double portion = q / radius;

            triple current_position_path = std::make_tuple(
                 std::cos(portion) * radius ,
                 radius * std::sin(portion),
                 0
            );

            current_position = triple_plus(dot_product(rotational_matrix, current_position_path), c); 
        }


        void update(double dt) override {
            if (profile->getQ() >= 2 * PI * radius) {
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

    };
}

#endif  // TRAJECTORY_GENERATOR_HPP_INCLUDED
