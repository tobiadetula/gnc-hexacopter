% Hexacopter parameters, GNC Mini Project
clc;clear;

% Hexacopter Model Values

m = 2.5;              % mass in kg
g = 9.81;             % gravity constant

L = 0.275;            % Arm Length = 27.5cm

b = 1.177*10^-5;      % Thrust Coefficient N/m2
d = 1.855*10^-7;      % Drag Coefficient



% Allocation matrix
T = [ones(1,6); % All thrust upward
    -L , L , L/2, -L/2, -L/2, L/2 ;  % Roll moments
    0,  0, L*sqrt(3)/2, -L*sqrt(3)/2, L*sqrt(3)/2, -L*sqrt(3)/2;  % Pitch moments
    -d/b, d/b, -d/b , -d/b, d/b, d/b;  % Yaw moments (alternating)
];

disp(T);
% T_psinv = T'*inv(T*T');
T_psinv = pinv(T); % Safer Alternative


% % Motor Positions assuming 1m
% M1_6_x = [cos(deg2rad(0)), cos(deg2rad(180)), cos(deg2rad(120)),...
%     cos(deg2rad(300)), cos(deg2rad(60)), cos(deg2rad(240))]; 
% M1_6_y = [sin(deg2rad(0)), sin(deg2rad(0)), sin(deg2rad(120)),...
%     sin(deg2rad(300)), sin(deg2rad(60)), sin(deg2rad(240))]; 
% M1_6_z = zeros(1,6);
% 
% M1_6 = [L*M1_6_x; L*M1_6_y; M1_6_z];

% Motor Positions assuming 1m and standard hexacopter layout
M1_6_x = [cosd(0), cosd(60), cosd(120), cosd(180), cosd(240), cosd(300)];
M1_6_y = [sind(0), sind(60), sind(120), sind(180), sind(240), sind(300)];
M1_6_z = zeros(1,6);

M1_6 = [L*M1_6_x; L*M1_6_y; M1_6_z];




% inertia matrix
% Hexacopter Inertia
gain = 1;
Ix = 0.00915*gain; Iy = 0.00915*gain; Iz = 0.01187*gain;
Ic = diag([ 0.00915 ; 0.00915 ; 0.01187 ])*gain; % Gotten from Paper

% Hexacopter Rotor Inertia
m_r = 0.02; % Rotor mass 20g
R_r = 0.1;  % Rotor radis 10cm
% Axial Moment of Inertia for Solid Disk
J_T = 1/2 *m_r* R_r^2; % For each rotor


% Coefficients
% Drag Coefficients 
Kx = .1; Ky = .1; Kz = .1; K_phi = 0.03; K_theta = 0.03; K_psi = 0.1;
% During Hover conditions
K_xyz = [Kx;Ky;Kz];
K_ptp = [K_phi;K_theta;K_psi];

% Disturbance - Specified as the speed of the rotor
% Omega = Omega3 + Omega4 - Omega1 - Omega2;

% Check if the variable Ic_temp exists in the workspace
if ~exist('Ic','var')
    n = 6; % no of motors 
    Ic = zeros(3);
    for i = 1:n
        temp = inertia_calc(M1_6(:,i)); % Calculate the inertia matrix for this point
        Ic = Ic + temp; % Add the result to the accumulator
        disp(Ic) % Display the running sum
    end
    Ix = Ic(1,1); Iy = Ic(2,2); Iz = Ic(3,3);
    disp('Calculation completed - Ic was not previously defined.');
else
    % Variable already exists
    disp('Ic already exists - calculation skipped.');
end




% Controller Parameters
A = [zeros(4), eye(4); zeros(4),zeros(4)];
B = [zeros(4);
0 ,  -L/Ix, 0 , L/Ix;
-L/Iy, 0 , L/Iy, 0;
d/(b*Iz), - d/(b*Iz), d/(b*Iz), -d/(b*Iz);
1/m, 1/m, 1/m, 1/m];

C = [eye(4),zeros(4)];
C_obs = eye(8);
C_ls = [1 0 0 0 0 0 0 0;   % phi
     0 0 1 0 0 0 0 0;   % theta
     0 0 0 0 1 0 0 0;   % psi
     0 0 0 0 0 0 1 0];  % z

K = [eye(4)*1.1 ; eye(4)*0.3025];

D = 0;      % Direct transmission matrix

sys = ss(A, B, C, D);

% Compute observability matrix
Wo = obsv(A, C);
unobsv = length(A) - rank(Wo); % No of states should be equal to the rank of obsv matrix


epsilon = 0.01;  % Small positive constant
% Q = (1 - epsilon) * (C' * C);
% P_e = lyap(A', Q);   % Now in standard Lyapunov form
% K = P_e \ C';        % Equivalent to inv(P_e) * C'

% rigid-body mass matrix
M_RB = [ m*eye(3) , zeros(3) ; zeros(3) , Ic ];



% Method 2: LQR-based Observer Design  
Q = eye(8)*10;      % State weighting
R = eye(8)*10;      % Output weighting
K_lqr = lqr(A', C_obs', Q, R)';

fprintf('\nObserver Gain Matrix K (LQR-based):\n');
disp(K_lqr);


poles = ones(8,1)*-1;
K_place = place(A',C_obs',poles)';



% Specifiy Motor Conditions

motor_index = 3;
fault_level = 0;
faulty_motors = [0 0 0 0 0 0];  % motor 1 failed


% rigid-body mass matrix
M_RB = [ m*eye(3) , zeros(3) ; zeros(3) , Ic ];