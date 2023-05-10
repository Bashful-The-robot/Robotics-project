
PURE PURSUIT CONTROLLER 

1-Determine the current location of the robot

2-Find the path point closest to the robot. It is possible that there are multiple
points one lookahead distance from the robot’s current location. The robot should steer toward
the closest point one lookahead distance from its current location. Therefore, the path point closest
to the robot will first be found, and the search for a point 1 lookahead distance away from the
robot will start at this point and commence up the path

3-Find the target point. The goal point is found by moving up the path and calculating the distance
between that path point and the vehicle’s current location. Path point locations are recorded in the
global frame; this calculation is done in global coordinates.

4-Transform the goalpoint to robot coordinates

5-Calculate the twist


