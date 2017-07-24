function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
b = video_pts;
a = logo_pts;
A = [];
for i = 1:4
    axi = [-b(i,1) -b(i,2) -1 0 0 0 b(i,1)*a(i,1) b(i,2)*a(i,1) a(i,1)];
    ayi = [0 0 0 -b(i,1) -b(i,2) -1 b(i,1)*a(i,2) b(i,2)*a(i,2) a(i,2)];
    A = [A; axi; ayi];
end
[U, S, V] = svd(A);
% h is last column of V
h1 = V(1:3,9);
h2 = V(4:6,9);
h3 = V(7:9,9);
H = [h1'; h2'; h3'];

% check:
% for i = 1:4
%     vid1 = [a(i,1); a(i,2); 1];
%     log1 = [b(i,1); b(i,2); 1];
%     log_proj = H*log1;
%     lamda = vid1(3)/log_proj(3)
%     log_proj*lamda;
% end
end