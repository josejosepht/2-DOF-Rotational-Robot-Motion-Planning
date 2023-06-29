% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = detectSweptVolumeCollisions(robot, obstacles, q_path)
num_collisions=0;
col_path=[]
for i=1:(length(q_path)-1)
        [poly1, poly2, pivot1, pivot2] = q2poly(robot,[q_path(i,1);q_path(i,2)]);
        [poly3, poly4, pivot1, pivot2] = q2poly(robot,[q_path(i+1,1);q_path(i+1,2)]);
        p1=[poly1.Vertices;poly3.Vertices];
        p2=[poly2.Vertices;poly4.Vertices];
        [k1,av] = convhull(p1);
        [k2,av] = convhull(p2);
        hull1=polyshape(p1(k1,1),p1(k1,2));
        hull2=polyshape(p2(k2,1),p2(k2,2));
        for j=1:length(obstacles)
            a1=intersect(hull1,obstacles(j));
            a2=intersect(hull2,obstacles(j));
            if (~isempty(a1.Vertices)||~isempty(a2.Vertices))
                col_path = [col_path;q_path(i,:);q_path(i+1,:)];
                num_collisions = num_collisions + 1;
        hold on
            plot(hull1, 'FaceColor', 'b');
            plot(hull2, 'FaceColor', 'b');
            hold off
            end
        end
end
plot_obstacles(obstacles)
for i = 1:size(col_path, 1)
            plotRobotWorkspace(robot, col_path(i,:)');
end
end