# NAME-OF-THE-PROJECT

TODO: Write general information about the project and what it does

## Installation and usage 

TODO: Refect to INSTALL.md file 

## Main features 

## Program structure and core design 

How the program is structures and what are the key components 

## Mathematical model for the velocity_profile 
With links, drawings, etc

Four user-defined arguments are needed for the inputs of this part, namely start point(double), end point(double), the maximal velocity(double), and the constant acceleration(double). The start point and the end point provide the total distance of the movement. The maximal velocity and the constant acceleration together determine the time for both acceleration and deceleration periods (tc = maximal velocity / acceleration). Since the total distance is already given, the distance for each period (acceleration, deceleration, or constant velocity) and the total movement time(tf = 2 * tc + (total distance - acceleration * tc^2) / maximal velocity ) can also be calculated. 

A trapezoidal velocity profile can be ensured at the beginning as well, if distance d1 (1/2 * acceleration * t^2) for acceleration or deceleration (the same distance) and distance d2 (total distance - 2 * d1) for constant velocity are both positive. The formulas for a trapezoidal velocity profile from the reference are applied in the code for all related functions: current acceleration (q_double_dot_current()), current velocity (updateVelocity()), and the current position (updatePosition()). 

## Mathematical model behind linear trajectory 
With links, drawings, etc

Eight inputs are required for this function, which are c vector, unit normal vector, start point(vector), end point(vector), maximal velocity(double), constant acceleration(double), current velocity(double), and force threshold(double). The c vector points from the origin of the base to the origin of the new coordination system. The unit normal vector stands for the orientation of the plane of the new coordination system (in the same direction of z axis). Furthermore, cross product of the unit normal vector and the c vector (after normalization) gives the unit vector y, cross product of y and unit normal vector yields unit vector z. Subsequently, all these vectors x, y, z build the rotation matrix, which with the c vector are able to transform the coordinates in the new CoS back to the base coordination system. This method offers us applicability even to the coordinate transformations.  

In addition, start point and end point (in the new CoS) define the direction of movement, and where the linear movement starts and ends, while maximal velocity, and constant acceleration serve for the inputs of velocity profile function, in order to create trapezoidal velocity profile. Force threshold should be set by the user, and once the detected force exceeds this threshold, the robot will stop. This is useful for real cases when robot touchs the surface of the table and stops to draw a circle on it. The current position value (double) from the velocity profile function times unit vector in the moving direction plus start point should give the current position in vector form.

## Mathematical model behind circular trajectory 
With links, drawings, etc

For the circular trajectory function, inputs are c vector, unit normal vector, radius(double), maximal velocity(double), and constant acceleration(double). The c vector and unit normal vector are the same as in the linear trajectory function, a rotation matrix is obtained based on them and coordinate transformations can thus be realized, which has already been described in detail previously. The radius means the circumference of the circle, which is also the total path needs to be travelled. The movement starts from the origin of the new CoS, it first moves to the (radius, 0, 0), to perform the circle drawing. Similarly, the last two inputs: maximal velocity and constant acceleration together with 0 (for start point) and circumference (for end point) will be input to the velocity profile function to generate trapezoidal velocity. Using the current position value acquired from the function, the ratio of the travelled path to the radius can also be gained. Futhermore, this ratio is of great importance, which gives the current position in vector form in the new CoS: (cos(ratio) * radius, radius * sin(ratio), 0). Finally, the movement should stop if the travelled path is larger than or equal to the circumference(2 * Pi * radius).
 

## How it all works together in Gazebo 

## How it all works together in IRL