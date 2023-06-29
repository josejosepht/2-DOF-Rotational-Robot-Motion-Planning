% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = computeConfigurationSpace(robot, obstacles, q_grid)
cspace = zeros(length(q_grid));
plot_obstacles(obstacles);
xlim([0 10]);
ylim([0 10]);
for i = 1:length(q_grid)
    for j = 1:length(q_grid)
        [poly1, poly2, pivot1, pivot2] = q2poly(robot,[q_grid(1,i) q_grid(1,j)]);
        for k =1:length(obstacles)
            collision1=intersect(poly1,obstacles(k));
            collision2=intersect(poly2,obstacles(k));
            if ~(isempty(collision1.Vertices))| ~(isempty(collision2.Vertices))
                cspace(i,j)=1;
                break
            else
                continue
            end
        end
    end
end
end