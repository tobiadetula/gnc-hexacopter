% Spaceship parameters, taken from Crew Dragon
% Jerome Jouffroy, February 2022

m = 25; % mass in kg

% compute inertia matrix, approximating spacecraft as cylinder
% Ic = diag([ 1/12*m*H^2+1/4*m*R^2 ; 1/12*m*H^2+1/4*m*R^2 ; 1/2*m*R^2 ]);
Ic = diag([ 2 ; 2 ; 4]);

% rigid-body mass matrix
M_RB = [ m*eye(3) , zeros(3) ; zeros(3) , Ic ];