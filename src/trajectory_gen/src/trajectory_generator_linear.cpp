#include "trajectory_gen/trajectory_generator_linear.hpp"

namespace trajectory_generator
{

LinearTrajectory::LinearTrajectory(triple c, triple normal_vector_hat , triple p_start, triple p_end, double q_dot_max, double q_double_dot, double q_dot, double force_threshold): 
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

void LinearTrajectory::updatePosition() {
    // current position calculation
    current_position = p_start + unit_vector * profile->getQ();
    // print_triple(current_position, "Update [position]::");
}

// To get the distance between the start point and the end point
double LinearTrajectory::current_diff() {
    triple diff = p_start - current_position;
    return triple_norm(diff);
}

void LinearTrajectory::update(double dt, double force) {
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

void LinearTrajectory::setForceThreshold(double th) {
    force_threshold = th;
}

geometry_msgs::Point LinearTrajectory::getPoint() {
    geometry_msgs::Point point;
    point.x = std::get<0>(current_position);
    point.y = std::get<1>(current_position);
    point.z = std::get<2>(current_position);
    return point;
}
}
