clear;clc;

syms phy theta psi;

% Define the rotation matrices around the x, y, and z axes
% using the symbolic variables phy, theta, and psi

% Rotation matrix around the z-axis
Rz = [cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0 0 1];
% Rotation matrix around the y-axis
Ry = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

% Rotation matrix around the x-axis
Rx = [1 0 0;
    0 cos(phy) -sin(phy);
    0 sin(phy) cos(phy)];

% The final rotation matrix is obtained by multiplying the individual rotation matrices
% The order of multiplication is important and depends on the convention used
% Here we assume the order is ZYX (yaw-pitch-roll)
R = Rz*Ry*Rx;
% Display the final rotation matrix
disp('Rotation matrix R:');
disp(R);


% Inverse rotation matrix
R_inv = inv(R);
disp('Inverse rotation matrix R_inv:');
disp(R_inv);