% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 1000;                       % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
map_threshold_low  = mode(mode(map))-0.2;
map_threshold_high = mode(mode(map))+0.5;
resample_threshold = 0.8;
sigma_m = 0.01 * [ 1; 1; 2 ];
radius = 0.02;
direction = myPose(3,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
W = repmat(1.0/M, [1, M]);

% figure;
% imagesc(map); 
% hold on;
% colormap('gray');
% axis equal;
% hold on;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
%     M_prev = M;
%     M = ceil(M - j/10);
%     M = max(M, 500);
%     fprintf('step: %d, M: %d\n', j, M);
    % 1) Propagate the particles 
    P = repmat(myPose(:,j-1), [1, M]);
    % add a random value to the orientation of the pose
%     P(3,:) = P(3,:) + randn(1,M)*sigma_m(3);
%     P(3,1:M) = myPose(3,j-1) + randn(1,M)*sigma_m(3);
    % randomize the radius travelled
%     R = radius * (1+randn(1,M)*sigma_m(1));
    R = radius;
    % calculate new positions
    P(1,1:M) = P(1,1:M) + R.*cos(P(3,1:M));
    P(2,1:M) = P(2,1:M) - R.*sin(P(3,1:M));
%     P(1,1:M) = P(1,1:M) + R.*cos(myPose(3,j-1));
%     P(2,1:M) = P(2,1:M) - R.*sin(myPose(3,j-1));
%     P(3,1:M) = myPose(3,j-1) + randn(1,M)*sigma_m(3);
    P = P + randn(3, M).*sigma_m;
    W = repmat(1.0/M, [1, M]);    % reset the weights every cycle
    
    P_corr = zeros(1, M);
    
    % 2) Measurement Update 
    for i = 1:M
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
        lidar_global(:,1) = ceil(( ranges(:,j).*cos(scanAngles + P(3,i)) + P(1,i))*myResolution + myOrigin(1));
        lidar_global(:,2) = ceil((-ranges(:,j).*sin(scanAngles + P(3,i)) + P(2,i))*myResolution + myOrigin(2));
        %fprintf('LIDAR max coords (x, y) = %d, %d\n', max(lidar_global(:,1)), max(lidar_global(:,2)));
        
    %   2-2) For each particle, calculate the correlation scores of the particles
        % for out of range positions, assume the original position on the map, resulting in a low correlation score
        lidar_global(lidar_global(:,1) < 1, 1) = myOrigin(1);
        lidar_global(lidar_global(:,1) < 1, 2) = myOrigin(2);
        lidar_global(lidar_global(:,2) < 1, 1) = myOrigin(1);
        lidar_global(lidar_global(:,2) < 1, 2) = myOrigin(2);
        lidar_global(lidar_global(:,1) > size(map,2), 1) = myOrigin(1);
        lidar_global(lidar_global(:,1) > size(map,2), 2) = myOrigin(2);
        lidar_global(lidar_global(:,2) > size(map,1), 1) = myOrigin(1);
        lidar_global(lidar_global(:,2) > size(map,1), 2) = myOrigin(2);
        s2i = sub2ind(size(map), lidar_global(:,2), lidar_global(:,1));
        corr_values = map(s2i);
        P_corr(i) = -3*sum(corr_values<=map_threshold_low)+10*sum(corr_values>=map_threshold_high);
    end
    P_corr = P_corr - min(P_corr);    % make range from 0, inf
    
    %   2-3) Update the particle weights
    W = (W(1:M) .* P_corr) / sum(P_corr);
    W = W / sum(W);
    
    %   2-4) Choose the best particle to update the pose
    [ value, index ] = max(W);    % simply the one with the largest weight
    myPose(:,j) = P(:,index);
%     myPose(:,j) = P * W';
%     myPose(:,j) = P * (W > 1.0/M)' ./ sum(W > 1.0/M);    % the average P based on the weights that are larger than average
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    N_eff = sum(W)^2/sum(W.^2);
%     fprintf('step: %d, N_eff: %f =====\n',j, N_eff);
    if (N_eff < resample_threshold*M)    % resample if effective size is below threshold%
        fprintf('step: %d, N_eff: %f =====\n',j, N_eff);
%         j = j - 1;
%         W_cum = cumsum(W);
%         for i = 1:M
%             index = find(rand <= W_cum,1);
%             P_update(:,i) = P(:,index);
%             W_update(i) = W(index);
%         end
%         P = P_update;
%         W = W_update;
%         M = M_prev + 50;    % reset number of samples
    end
    
    % 4) Visualize the pose on the map as needed
    fprintf('step: %d, P: (%f, %f, %f)\n', j, myPose(1,j), myPose(2,j), myPose(3,j));
%     plot(myPose(1,j)*param.resol+param.origin(1), ...
%     myPose(2,j)*param.resol+param.origin(2), 'r.-');

end

end
