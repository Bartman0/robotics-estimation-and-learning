% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 
load('mu.mat', 'mu');
load('sigma.mat', 'sigma');
thre = 0.00001;

D = 3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
[ r, c, d ] = size(I);
muM = repmat(mu, [r*c, 1]);
IM = reshape(I, [r*c, d]);
diff = double(IM) - muM;
p = exp(-0.5*sum(diff * inv(sigma) .* diff, 2))/((2*pi)^(D/2) * sqrt(det(sigma)));

bw = reshape(p,[r,c,1]) > thre; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
bw_biggest = false(size(bw));
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
segI = bw_biggest;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
