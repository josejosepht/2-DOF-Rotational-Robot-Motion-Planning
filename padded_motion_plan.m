clear all
close all
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
q_goal = [3.05; 0.05];

cspace_resolution = 100;

q_grid = linspace(0, 2*pi, cspace_resolution);

cspace = computeConfigurationSpace(robot, obstacles, q_grid);
padded_cspace = incorporateObstaclePadding(cspace);

% visualize configuration space
imshow(1 - padded_cspace');
set(gca, 'YDir', 'normal');

distances = computeDistanceTransform(padded_cspace, q_grid, q_goal);

path = findPath(distances, q_grid, q_start);

% visualize distance transform and found path
imshow(distances', [min(min(distances)), max(max(distances))]);
hold on;
scatter(path(:,1), path(:,2), 'rs', 'MarkerFaceColor', 'r');
set(gca, 'YDir', 'normal');
plot_obstacles(obstacles);
% Convert path in discretized grid into configuration-space path
q_path = convertPathToConfigurations(q_grid, q_start, q_goal, path);
% FPS (frames per second) controls time between frames
fps = 4;
% Use plotRobotWorkspace to plot each robot configuration along the path
for i = 1:size(q_path, 1)
    plotRobotWorkspace(robot, q_path(i,:)')
    pause(1.0 / fps);
end
plot_obstacles(obstacles);
% Plot swept-volume collisions, if any
num_collisions = detectSweptVolumeCollisions(robot, obstacles, q_path);
fprintf('Path contains %d collisions.\n', num_collisions);