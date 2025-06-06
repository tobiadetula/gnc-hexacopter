syms phi theta psi real

% Rotation matrix around x-axis (Roll)
R_xphi = [ 1        0           0;
           0   cos(phi)  -sin(phi);
           0   sin(phi)   cos(phi) ];

% Rotation matrix around y-axis (Pitch)
R_ytheta = [ cos(theta)   0   sin(theta);
                  0       1        0;
            -sin(theta)   0   cos(theta) ];

% Rotation matrix around z-axis (Yaw)
R_zpsi = [ cos(psi)  -sin(psi)   0;
           sin(psi)   cos(psi)   0;
               0          0      1 ];
% ZYX (yaw-pitch-roll)
R_body_to_inertial = R_zpsi * R_ytheta * R_xphi;

% XYZ (roll-pitch-yaw)
% R_body_to_inertial = R_xphi * R_ytheta * R_zpsi;


% YZX (pitch-yaw-roll)
% R_body_to_inertial = R_ytheta * R_zpsi * R_xphi;

disp(R_body_to_inertial)
disp(R_body_to_inertial(:,3))