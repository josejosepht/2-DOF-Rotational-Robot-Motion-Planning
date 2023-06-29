% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = incorporateObstaclePadding(cspace)
%assign padded_cspace to be cspace initially
padded_cspace = cspace;
%create an empty list of obstacle indices in cspace
obstacles_idx = []
% traverse through configuration space in row order and index the obstacles
  for i = 1:length(cspace)
      for j = 1:length(cspace)
          if cspace(i,j) == 1
              obstacles_idx = [obstacles_idx,[i;j]];
          end
      end
  end 
  %movements to get to neighbouring cells
    xn = [-1, 0, 1, 0,-1, 1, -1, 1;0, 1, 0, -1, -1, 1, 1,-1];


  for j = 1:length(obstacles_idx)
    xCurr = obstacles_idx(1,j);
    yCurr = obstacles_idx(2,j);

        for i = 1:length(xn)

            x = xCurr + xn(1,i);
            y = yCurr + xn(2,i);
            %Traverse through the neighbours of obstacles(==1) and assign
            %them as 1 as well and hence padding the obstacle
            if x >= 1 && x <= length(cspace) && y >= 1 && y <= length(cspace)      
                padded_cspace(x,y) = 1;
            end
        end 
  end 

end