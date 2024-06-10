
#ifndef TRAJECTORY_HPP_INCLUDED
#define TRAJECTORY_HPP_INCLUDED

#include <geometry_msgs/Point.h>
#include <string>

namespace trajectory_generator {
    class Trajectory {
    public:
        virtual ~Trajectory() {}
        virtual void update(double dt, double force = 0) = 0;
        virtual bool isEnded() = 0;
        virtual geometry_msgs::Point getPoint() = 0;
        virtual std::string getName() = 0;
    };


    // To create better precision for critical value region such as near 0
    inline double precision(const double f, const int places)
    {
        double n = std::pow(10.0d, places) ;
        return std::round(f * n) / n ;
    };
}
    

#endif // TRAJECTORY_HPP_INCLUDED