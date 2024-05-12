#include <tuple>

using triple = std::tuple<double, double, double>;

class Trajectory {
private:
    // contstant acceleration 
    double q_double_dot;
    // maximum velocity 
    double q_dot_max;
    // start position
    triple p_start;
    // end position 
    triple p_end;

    bool hasEnded = false;
public:
    Trajectory(triple p_start, triple p_end): p_start(p_start), p_end(p_end) {};

    virtual void updatePosition();
    virtual void updateVelocity();
    void update(double dt);
}