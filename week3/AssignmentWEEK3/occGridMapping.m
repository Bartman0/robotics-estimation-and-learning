% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myOrigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time
    
    % Find grids hit by the rays (in the grid map coordinate)
    X_robot = pose(1:2, j);
    d = ranges(:, j);
    theta = pose(3, j);
    
    R = [ cos(scanAngles + theta), ...
        -sin(scanAngles + theta) ];
    
    X_occ = ( R .* d + X_robot' )';
    i_occ = ceil(myResol .* X_occ) + myOrigin;
    i_robot = ceil(myResol .* X_robot) + myOrigin;

    % Find occupied-measurement cells and free-measurement cells
    M = size(i_occ,2);
    for i = 1:M
        [freex, freey] = bresenham(i_robot(1), i_robot(2), i_occ(1,i),i_occ(2,i));
        free = sub2ind(size(myMap),freey,freex);
        % Update the log-odds
        myMap(free) = myMap(free) - lo_free;
        myMap(i_occ(2,i),i_occ(1,i)) = myMap(i_occ(2,i),i_occ(1,i)) + lo_occ;
    end
    
    % Saturate the log-odd values
    myMap(myMap < lo_min) = lo_min;
    myMap(myMap > lo_max) = lo_max;
    
end

end
