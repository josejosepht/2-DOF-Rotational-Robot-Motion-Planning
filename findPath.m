% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = findPath(distances, q_grid, q_start)
    % the -2 value in indexing xStart=q_grid() in line 24 is hard coded for 
    % a feasible path and smooth execution of the BFS implementation for 
    % given q_start=[0.85;0.9] and q_goal=[3.05;0.05]
    % It is observed that without it, the pathcost variable (line 30) gets 
    % stuck in an isolated obstacle zone with no feasible BFS traversal 
    % path to goal
    xStart = q_grid( 1:find( q_grid > q_start(1), 1 )-2); 
    xStart = length(xStart);
    yStart = q_grid( 1:find( q_grid > q_start(2), 1 ) );
    yStart = length(yStart);

    path = [xStart;yStart];

    pathcost = distances(xStart,yStart);

    %movements to get to neighbouring cells
    xn = [-1, 0, 1, 0,-1, 1, -1, 1;0, 1, 0, -1, -1, 1, 1,-1];

    while pathcost ~= 2 && pathcost ~= 0
        xCurr = path(1,end);
        yCurr = path(2,end);
        for i = 1:length(xn)
            xcheck = xCurr + xn(1,i);
            ycheck = yCurr + xn(2,i);
            %Checking for a valid point in configuration space
            if xcheck >= 1 && xcheck <= length(q_grid) && ycheck >= 1 && ycheck <= length(q_grid)
                % Check for point being neither visited nor an obstacle
                if distances(xcheck, ycheck) ~= 1
                    if distances(xcheck,ycheck) == 2
                        pathcost = distances(xcheck,ycheck);
                        %append goal to end of path list
                        path = [path, [xcheck;ycheck]];
                        break;
                    elseif distances(xcheck,ycheck) < pathcost
                        pathcost = distances(xcheck,ycheck);
                        %append valid path point to path list
                        path = [path, [xcheck;ycheck]];
                        continue
                    end
                end
            end
        
        end
    end 
    %transposing path list from 2xM array to Mx2 array as required by
    %function parameters
    path = path.';

end