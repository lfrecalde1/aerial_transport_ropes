function [E] = optimization_pid_drone(x, t_final, ts, hd, hdp, hdpp)

%% Set time parameter
t = (0:ts:t_final);

%% Initial state
x_init = [0;  0; 0]; %% Initial position of the system
xp_init = [0; 0; 0]; %% Initial Velocity of the system
quat_init = [1; 0; 0; 0]; %% Initial quaternios
w_init = [0; 0; 0]; %%

%% Initial Rotational Matrix
R = zeros(3,3,length(t)+1);
R(:, :, 1) = QuatToRot(quat_init);

%% Initial Euler Angles
euler = zeros(3, length(t)+1);
euler(:, 1) = quat2eul(quat_init','XYZ');

%% Euler dot
euler_d = zeros(3, length(t) +1);
euler_d(:, 1) = euler_dot(euler(:,1), w_init);
%% Aux Rotational Matrix
R_1 = zeros(3, 3, length(t)+1);
R_1(:, :, 1) = Rot_zyx(euler(:, 1));
%% Initial vector State
h = zeros(13, length(t) +1);
h(:, 1) = [x_init; xp_init; quat_init; w_init];

%% System parameters
g = 9.80;
factor = 10;
m_drone = 0.33*factor;
l_drone = 0.0325*factor;
Jxx_drone = (1.395e-4)*factor;
Jyy_drone = (1.395e-4)*factor;
Jzz_drone = (2.173e-4)*factor;
%% Vector of system Parametes
L_drone = [g; m_drone; l_drone; Jxx_drone; Jyy_drone; Jzz_drone];

%% Control vector
u = zeros(4, length(t));


%% Desired Values
hd = hd;
hd_d = hdp;
hd_dd = hdpp;

e_pos_i = [0;0;0];
e_angular_i = [0;0;0];
%% Simulation system

ERROR = [];
CONTROL = [];
for k = 1:length(t)
    %% Controller section
    e_pos = [hd_d(1:3,k)-h(4:6,k)];
    ERROR = [ERROR;e_pos];
    [u(:, k), e_pos_i, e_angular_i]= low_level_controller(h(:, k ), hd(:, k), hd_d(:, k), hd_dd(:, k), L_drone, x, e_pos_i, e_angular_i, ts);
    CONTROL = [CONTROL;u(:,k)];
    %% System evolution
    h(:, k+1) = system_simulation(h(:, k), u(:, k), L_drone, ts);
    R(:, :, k+1) = QuatToRot(h(7:10, k+1));
    euler(:, k+1) = quat2eul(h(7:10, k+1)','XYZ');
    euler_d(:, k+1) = euler_dot(euler(:, k+1), h(11:13, k+1));
    R_1(:, :, k+1) = Rot_zyx(euler(:, k+1));
end
E = 1*(ERROR'*ERROR) + 0.8*(CONTROL'*CONTROL);
end

