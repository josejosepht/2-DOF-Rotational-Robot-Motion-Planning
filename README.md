## 2-DOF-Rotational-Robot-Motion-Planning
This repository implements collision-free path planning for a 2-link rotational planar robot in a 2D workspace. It includes functionalities such as plotting the robot in the workspace, converting the problem into configuration space, computing distance transforms, finding paths, converting paths into configurations, detecting swept-volume collisions, and incorporating obstacle padding for conservative planning.

Here are the different functions scripts defined with associated plots it would generate when run:

`plotRobotWorkspace.m`: This function plots the 2-link rotational planar robot in the 2D workspace at a specified configuration. It handles appropriate transformations of the robot's links' polygons and pivot points. The function takes the robot configuration as input and visualizes the robot's position in the workspace.
```matlab
% Set up obstacles as 2-D polygons
p1 = polyshape([1.5 1.5 3.5 3.5], [1.4 0.7 0.7 1.4]);
p2 = polyshape([9.1 8.3 8.3 9.8 9.8], [1.9 1.4 0.5 0.5 1.4]);
p3 = polyshape([0.9 0.9 2.0 2.0 2.8 3.3 2.8 2.0 2.0], [7.2 3.7 3.7 4.8 4.8 5.5 6.2 6.2 7.2]);
p4 = polyshape([9.0 9.0 8.3 8.3 9.0 9.0 10.0 10.0], [7.3 6.2 6.2 4.8 4.8 3.8 3.8 7.3]);
p5 = polyshape([6.9 6.3 6.9], [7.1 5.6 5.6]);
obstacles = [p1 p2 p3 p4 p5];

% The 2-DOF 2-link rotational planar robot is encapsulated in a MATLAB cell
robot = {};
% Robot links are 2-D polygons as well
% Pivot point of link 1, with respect to base frame (at origin)
robot.pivot1 = [6.4; 2.5];
% Pivot point of link 2, with respect to frame 1 (at pivot1)
robot.pivot2 = [2.1; 0];
% Corners of link 1 polygon, with respect to frame 1
robot.link1 = [-1.2 -1.2 2.3 2.3; 0.5 -0.5 -0.4 0.4];
% Corners of link 2 polygon, with respect to frame 2
robot.link2 = [-0.3 -0.3 2.7 2.7; 0.4 -0.4 -0.2 0.2];

q_start = [0.85; 0.9];

plotRobotWorkspace(robot, q_start)
```
Consider the above code snippet that give the below plot:

![image](https://github.com/josejosepht/2-DOF-Rotational-Robot-Motion-Planning/assets/97187460/1f415cfd-f33c-465d-914c-ca05f8283abc)



`computeConfigurationSpace.m`: This function converts the problem into configuration space by discretizing the configuration space grid. It checks for collisions at each discrete grid point by intersecting the robot's links' polygons with the obstacles' polygons in the workspace. The output is a matrix representing the configuration space, indicating collision or non-collision at each grid point.



`computeDistanceTransform.m`: Given a specified goal configuration and the configuration space grid from computeConfigurationSpace(), this function computes the distance transform from the nearest grid point to the goal configuration. The distance transform represents the distance from each grid point to the goal configuration, providing a measure of closeness.



`findPath.m`: Using the distance transform from computeDistanceTransform(), this function finds a path from the start configuration's closest grid point to the goal configuration's grid point. It descends the distance transform in a greedy fashion, breaking ties based on a chosen strategy. Diagonal neighbors are allowed in the path exploration.



`convertPathToConfigurations.m`: This function converts the path found in findPath(), which is in grid point indices, into a path in actual robot configurations. It includes the actual start and goal configurations, enabling visualization of the path in the workspace.



`detectSweptVolumeCollisions.m`: Given the path obtained from convertPathToConfigurations(), this function detects any swept-volume collisions along the trajectory. It approximates the swept volume using appropriate convex hulls of the robot's links' polygons. It checks if any segments of the trajectory are in collision and returns the number of collisions.



`incorporateObstaclePadding.m`: To avoid collisions caused by planning paths too close to obstacles, this function pads the obstacles in the configuration space by one grid cell, considering diagonal neighbors as well. It verifies that the resulting trajectory, after incorporating the obstacle padding, does not contain any swept-volume collisions.

Plots generated on running `passed_motion_plan.m` :
Configuration space:
![image](https://github.com/josejosepht/2-DOF-Rotational-Robot-Motion-Planning/assets/97187460/1a66126d-6082-43f1-b159-0cd78530b961)

Distance transform from goal configuration:

![image](https://github.com/josejosepht/2-DOF-Rotational-Robot-Motion-Planning/assets/97187460/a8db7a12-de00-46f5-8dab-307b7abd49d2)

Path generated from start to goal:

![image](https://github.com/josejosepht/2-DOF-Rotational-Robot-Motion-Planning/assets/97187460/9a52b30d-8f15-48fc-aa0f-8dc072327cf7)

Trajectory from start to goal:

![image](https://github.com/josejosepht/2-DOF-Rotational-Robot-Motion-Planning/assets/97187460/f7b78ffa-9256-4a1c-a71e-1443ccf92b94)

Swept-volume collisions along path:
According to the path generated in `findPath.m`, it is seen that there is only one set of consecutive path workspace configurations that have a swept volume with a collision with one of the obstacles:
![image](https://github.com/josejosepht/2-DOF-Rotational-Robot-Motion-Planning/assets/97187460/94110e58-a73c-4f98-a5c4-f71bbf7c1209)

More conservative(padded) trajectory from start to goal, with no swept-volume collisions:
It can observed that around the triangular object, the 2 link manipulator maintains a distance and swerves along the path as compared to before.
![image](https://github.com/josejosepht/2-DOF-Rotational-Robot-Motion-Planning/assets/97187460/b6c5f16f-94a6-4c09-8cc3-b6d5e3c65ae3)
