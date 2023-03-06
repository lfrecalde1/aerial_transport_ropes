function [u, e_pose_i, e_angle_i] = low_level_controller(h, hd, hdp, hdpp, L, e_pose_i, e_angle_i, ts)
%Low Level Controller
%   Code to compute f and tau

%% Control vector
u = zeros(4, 1);

%% Gains Matrices Position
kp_pos = 50;
kv_pos = 1;
Kp_pos = kp_pos*eye(3,3);
Kd_pos = kv_pos*eye(3,3);
Ki_pos = 30*eye(3,3);

%% Gains Angles
kp_e = 0.03;
Kpe = kp_e*eye(2);
Kde = 0.008*eye(2);
kie = 0.02*eye(2);

%% System Parameters
g = L(1);
m = L(2);
l = L(3);
Jxx = L(4);
Jyy = L(5);
Jzz = L(6);

%% Get pose Drone
pos = h(1:3);
vel = h(4:6);
angles = quat2eul(h(7:10)','XYZ');
angles_d = euler_dot(angles, h(11:13));
%% Desired Positions

%% Desired Velocities
vel_d = hdp(1:3);
psid_d = hdp(4);

%% Desired Acceleration
accel_d = hdpp(1:3);

%% Velocity error
e_vel = vel_d - vel;
e_pose_i = e_pose_i + e_vel*ts;

%% Aux acceleration for the z velocity
aux_aceleration = accel_d + Kp_pos*e_vel + Ki_pos*e_pose_i;

%% Desired Angles of attack
desired_angles = [hd(1);hd(2)];

%% Error pith and theta
e_angle = desired_angles - angles(1:2)';
e_omega = [0;0] - angles_d(1:2);
e_angle_i = e_angle_i + e_angle*ts;
%% COntrol Law Momentum 
%% PID control Pith theta
M(1:2) = Kpe*e_angle + Kde*e_omega + kie*e_angle_i;

%% PID control  rotation
M(3) = 0.01*(psid_d-angles_d(3));

%% COntrol law Force upper
F = m*g + aux_aceleration(3);

%% Merge Control laws
u(1) = F;
u(2:4) = M;
end

