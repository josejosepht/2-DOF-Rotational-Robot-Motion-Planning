% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = computeDistanceTransform(cspace, q_grid, q_goal)
    %Find the indices of the goal in q_grid
    xStart = q_grid( 1:find( q_grid > q_goal(1), 1 ) );
    xStart = length(xStart);
    yStart = q_grid( 1:find( q_grid > q_goal(2), 1 ) );
    yStart = length(yStart);

    % Start with the distance grid being same as cspace
    %where goal is assigned with 2
    distances = cspace;
    distances(xStart, yStart) = 2;

    %movements to get to neighbouring cells
    xn = [-1, 0, 1, 0,-1, 1, -1, 1;0, 1, 0, -1, -1, 1, 1,-1];
        
    % Start a bfs search in the current cspace from the goal point
    queue = [xStart, yStart];
    while ~isempty(queue)
        % Get the current node
        xCurr = queue(1, 1);
        yCurr = queue(1, 2);
        %Start with an empty queue list
        queue(1, :) = [];
        
        % Checking if neighbouring 8 cells are visited and marked
        for i = 1:8
            xcheck = xCurr + xn(1,i);
            ycheck = yCurr + xn(2,i);
            if xcheck >= 1 && xcheck <= length(q_grid) && ycheck >= 1 && ycheck <= length(q_grid)
                if distances(xcheck, ycheck) == 0 && distances(xcheck, ycheck) ~= 1
                    distances(xcheck, ycheck) = distances(xCurr, yCurr) + 1;
                    %add to queue to mark as visited
                    queue = [queue; xcheck ycheck];
                end
            end
        end
    end
end