% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
% Translate frame origins
origin1_at0 = robot.pivot1;
origin2_at0 = origin1_at0 + robot.pivot2;
% Compute link polygon corners
link1_at0 = robot.link1 + origin1_at0;
link2_at0 = robot.link2 + origin2_at0;
link1_at1 = link1_at0 - robot.pivot1;
tq11=rt2tr(rotz(-q(1)),[0 0 0]);
link1_atq1 = (tq11^-1)*[link1_at1;zeros(2,4)];
link1_atq1 = link1_atq1(1:2,:);
link1_atq1 = link1_atq1 + robot.pivot1;
robot.pivot2 = robot.pivot1+[2.1*cos(q(1));2.1*sin(q(1))];
origin2_atq = robot.pivot2;
xlim([0 10]);
ylim([0 10]);
grid on
hold on
%plot(polyshape(link1_atq1(1,:), link1_atq1(2,:)), 'FaceColor', 'r');
%plot(origin2_atq(1), origin2_atq(2), 'k.', 'MarkerSize', 10);

tq2q1=rt2tr(rotz(-(q(1)+q(2))),[0 0 0]);
link2_atq2=link2_at0-(origin2_at0 - origin2_atq);
link2_atq2 = link2_atq2 - origin2_atq;
link2_atq2 = (tq2q1^-1)*[link2_atq2;zeros(2,4)];
link2_atq2 = link2_atq2(1:2,:);
link2_atq2 = link2_atq2 + origin2_atq;
%plot(polyshape(link2_atq2(1,:), link2_atq2(2,:)), 'FaceColor', 'b');
hold off
poly1 = polyshape(link1_atq1(1,:),link1_atq1(2,:));
poly2 = polyshape(link2_atq2(1,:),link2_atq2(2,:));
pivot1 = robot.pivot1;
pivot2 = origin2_atq;
end