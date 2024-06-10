#include "trajectory_gen/trajectory_generator_circular.hpp"

namespace trajectory_generator 
{
CircleTrajectory2d::CircleTrajectory2d(triple c, triple normal_vector_hat, double radius, double q_dot_max, double q_double_dot): 
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

void CircleTrajectory2d::updatePosition() {
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


void CircleTrajectory2d::update(double dt, double force) {
    std::cout << "Circular z" << std::get<2>(current_position) << "\n";
    if (precision(profile->getQ(), 3) >= precision(2 * PI * radius, 3)) {
        has_ended = true;
        return;
    }
    time += dt;
    profile->update(dt);
    updatePosition();
}

geometry_msgs::Point CircleTrajectory2d::getPoint() {
    geometry_msgs::Point point;
    point.x = std::get<0>(current_position);
    point.y = std::get<1>(current_position);
    point.z = std::get<2>(current_position);
    return point;
}
}