//write here the trajectory generator: linear and circular

#ifndef TRAJECTORY_GENERATOR_HPP_INCLUDED
#define TRAJECTORY_GENERATOR_HPP_INCLUDED
#include <tuple>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <string>

namespace trajectory_generator
{
    using triple = std::tuple<double, double, double>;

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

    // |V| = sqrt(x*x + y*y + z*z) 
    double triple_norm(triple p) {
        return std::sqrt(
           std::get<0>(p) * std::get<0>(p) + 
           std::get<1>(p) * std::get<1>(p) + 
           std::get<2>(p) * std::get<2>(p)
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
         << ", z = " << std::get<2>(tr) << "\n";
    }

    class LinearTrajectory {
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

        bool has_ended = false;
    public:
        LinearTrajectory(triple p_start, triple p_end, double q_dot = 1): 
         p_start(p_start), p_end(p_end), q_dot(q_dot), current_position(p_start) {
            triple diff = triple_minus(p_end, p_start);
            initial_diff = triple_norm(diff);
            unit_vector = triple_divide(diff, initial_diff);
            // print_triple(unit_vector, "Unit vector:::");
            // print_triple(p_start, "P_start vector:::");
            // print_triple(p_end, "P_end vector:::");
         };

        void updatePosition() {
            // current position calculation
            current_position = triple_plus(p_start, triple_mul(triple_mul(unit_vector, time), q_dot));
            print_triple(current_position, "Update [position]::");
        }

        double current_diff() {
            triple diff = triple_minus(p_start, current_position);
            return triple_norm(diff);
        }

        void update(double dt) {
            std::cout << "Current [diff]::" << current_diff() << "\n";
            std::cout << "Current [init_diff]::" << initial_diff << "\n";
            if (current_diff() >= initial_diff) {
                has_ended = true;
                return;
            }
            time += dt;
            updatePosition();
        }

        bool isEnded() {
            return has_ended;
        }

        geometry_msgs::Point getPoint() {
            geometry_msgs::Point point;
            point.x = std::get<0>(current_position);
            point.y = std::get<1>(current_position);
            point.z = std::get<2>(current_position);
            return point;
        }
    };


    const double PI = 3.141592653589793238463;

    class CircleTrajectory2d {
    private:
        // start of the circle as \theta = 0
        triple p_start;
        // end of the circle as \theta = \pi
        triple p_end;
        // center of the circle
        triple center;
        // radious of the circle
        double radius;
        // angular velocity
        double w;

        // current position 
        triple current_position;
        // current time 
        double time;

        // end time 
        double end_time;

        // hasEnded bool
        bool has_ended;
    public: 
        CircleTrajectory2d(triple center, double radius, double w): 
        center(center), radius(radius), w(w) {
            p_start = triple_plus(center, std::make_tuple(radius, 0.0d, 0.0d));
            p_end = triple_plus(center, std::make_tuple(-radius, 0.0d, 0.0d));
            current_position = std::make_tuple(
                std::get<0>(p_start),
                std::get<1>(p_start),
                std::get<2>(p_start)
            );

            end_time = 2 * PI / w;
        }   

        void updatePosition() {
            // current position calculation
            current_position = std::make_tuple(
                 std::get<0>(center) + radius * std::cos(w * time),
                 std::get<1>(center) + radius * std::sin(w * time),
                 std::get<2>(center)
            );
        }


        void update(double dt) {
            if (time > end_time) {
                has_ended = true;
                return;
            }
            time += dt;
            updatePosition();
        }

    };
}

#endif  // TRAJECTORY_GENERATOR_HPP_INCLUDED
