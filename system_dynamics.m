function [xp] = system_dynamics(h, u, F_ext, L)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% System parameters
g = L(1);
m = L(2);
l = L(3);
Jxx = L(4);
Jyy = L(5);
Jzz = L(6);

%% Inertia Mmatrix
J = [Jxx, 0 0;...
     0, Jyy, 0;...
     0, 0, Jzz];
 

%% Torques inputs
tau = [u(2); u(3); u(4)];

e = [0;0;1];

%% Velocity of the system
pos = h(1:3);
vel = h(4:6);
quat = h(7:10);
omega = h(11:13);

%% Rotational matrix
[R] = QuatToRot(quat);

%% Force body frame Drone

F = [0; 0; u(1)];
F_ext_B = inv(R)*F_ext;
F_ext = F_ext;
%% Aceleration system 
acceleration = (R*F)/m- e*g + F_ext/m;
q_dot = quat_dot(quat, omega);
aux = J*omega;
pqr_dot = inv(J)*(tau - cross(omega, aux));

%% General vector of the system
xp = zeros(13, 1);
xp(1:3) = vel;
xp(4:6) = acceleration;
xp(7:10) = q_dot;
xp(11:13) = pqr_dot;
end

