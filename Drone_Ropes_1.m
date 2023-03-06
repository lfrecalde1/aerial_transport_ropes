%% Code to simulate a rope
clc, clear all, close all;
%% Set time parameters
ts = 0.001;
t_final = 10;
t = (0:ts:t_final);

%% Time auxiliar
t_init = (0:ts:5);

%% Particles Definition
m = 0.01;
fm = 1.5;

%% Just for two ropes
nr = 1;
nm = 70;

%% Spring Length
spring_length = 0.05;
spring_k = 6000;
damper_k = 4.9;

%% Gravitational Vector
g = 9.8;
wind_drag = 0.06;

%% Angle
alpha = 80*(pi/180);

%% Pose Load
load_pose = [1; 0; 1];

%% Vector with all the parameters
L = [g; spring_length; spring_k; damper_k; wind_drag; ts; alpha; length(t)];

%% Ropes Definition
Matrix_ropes = Ropes_general(nr, nm, m, fm, L, load_pose);

%% Velociy Of the first Mass
v_extern = zeros(3, nr, length(t));

%% Velocity rope 1
v_extern(1, 1, :) = 0*sin(0.8*t);
v_extern(2, 1, :) = 0.0*cos(1*t);
v_extern(3, 1, :) = 0.0;


%% Velocity Rope 2
v_extern(1, 2, :) = 0*sin(0.8*t);
v_extern(2, 2, :) = 0.0*cos(1*t);
v_extern(3, 2, :) = 0.0*sin(0.8*t);

%% Vector to save forces of each rope
Forces_rope = zeros(3, nr, length(t));
norm_force = zeros(nr, length(t));

%% Rope Initialization
for k = 1:length(t_init)
    tic
    Matrix_ropes.system_forces();
    Matrix_ropes.f_system_ropes(2);
    toc;
end

[initial_pose, initial_velocity] = Matrix_ropes.get_position_initial();
%% Drone Parameters 
%% Initial state
x_init = initial_pose'; %% Initial position of the system
xp_init = initial_velocity'; %% Initial Velocity of the system
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

%% Desired Trajectory of the system
phi_d = (15*pi/180)*sin(1*t);
theta_d = (0*pi/180)*ones(1, length(t));
zd = 0*ones(1, length(t));
psid = 0*ones(1, length(t));

phid_d = 0*ones(1, length(t));
thetad_d = 0*ones(1, length(t));
zd_d = 0*ones(1, length(t));
psid_d = 0.0*ones(1, length(t));

xd_dd = 0*ones(1, length(t));
yd_dd = 0*ones(1, length(t));
zd_dd = 0*ones(1, length(t));

hd = [phi_d;theta_d;zd;psid];
hd_d = [phid_d;thetad_d;zd_d;psid_d];
hd_dd = [xd_dd;yd_dd;zd_dd];

%% PID 
e_pose_i = [0;0;0];
e_angle_i = [0;0];

%% Simulation
for k = 1:length(t)
    tic
    % Low level control aerial vehicle
    [u(:, k), e_pose_i, e_angle_i]= low_level_controller(h(:, k ), hd(:, k), hd_d(:, k), hd_dd(:, k), L_drone,  e_pose_i, e_angle_i, ts);    
    
    %% Get velocity Aerial vehicle
    v_extern(:, 1, k) = h(4:6, k);
    
    % Velocity to the rope
    Matrix_ropes.apply_Velocity(v_extern(:, :, k));
    % Get system Information
    tic
    Matrix_ropes.system_forces();
    %% Get Forces in the rope
    [Forces_rope(:, 1:nr, k), norm_force(:, k)]= Matrix_ropes.get_total_force();
    F_external_rope = Forces_rope(:, 1, k);
    
    % System Evolution
    Matrix_ropes.f_system_ropes(k+1);
    h(:, k+1) = system_simulation(h(:, k), u(:, k), F_external_rope, L_drone, ts);
    R(:, :, k+1) = QuatToRot(h(7:10, k+1));
    euler(:, k+1) = quat2eul(h(7:10, k+1)','XYZ');
    euler_d(:, k+1) = euler_dot(euler(:, k+1), h(11:13, k+1));
    R_1(:, :, k+1) = Rot_zyx(euler(:, k+1));
    toc;
   
    
end

close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf,'Position',[300 200 800 700])
myVideo = VideoWriter('myVideoFile_1'); 
myVideo.FrameRate = 10;  
open(myVideo)
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
grid minor;
%% Plot Ropes Section
rope_1_body = plot3(Matrix_ropes.data(1,:,1,1), Matrix_ropes.data(2,:,1,1), Matrix_ropes.data(3,:,1,1),'.-','Color',[56,171,217]/255,'linewidth',5*Matrix_ropes.m);
%rope_2_body = plot3(Matrix_ropes.data(1,:,2,1), Matrix_ropes.data(2,:,2,1), Matrix_ropes.data(3,:,2,1),'.-','Color',[56,171,217]/255,'linewidth',5*Matrix_ropes.m);
rope_final = plot3(Matrix_ropes.data_final(1,1), Matrix_ropes.data_final(2,1), Matrix_ropes.data_final(3,1),'o','Color',[100,100,100]/255,'linewidth',5*Matrix_ropes.fm);

%% Plot Drone
Drone_Parameters(0.09);
G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),R(:, :, 1));hold on
plot3(h(1,1),h(2,1),h(3,1),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on


for k = 2:100:length(t)
    drawnow;
    delete(rope_1_body);
    delete(rope_final);
    delete(G2);
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),R(:, :, k));hold on
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    rope_1_body = plot3(Matrix_ropes.data(1,:,1,k), Matrix_ropes.data(2,:,1,k), Matrix_ropes.data(3,:,1,k),'.-','Color',[56,171,217]/255,'linewidth',5*Matrix_ropes.m); hold on;
    rope_final = plot3(Matrix_ropes.data_final(1,k), Matrix_ropes.data_final(2,k), Matrix_ropes.data_final(3,k),'o','Color',[100,100,100]/255,'linewidth',5*Matrix_ropes.fm); hold on;
    title('$\textrm{Executed Movement}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    axis([-9.5 9.5 -9.5 9.5 -9.5 9.5]);
    %grid minor;
    %view([-90 0]);
    %% Video Save
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);

end

Forces_rope_1 = reshape(Forces_rope(:,1,:), 3, length(t));
close(myVideo)

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(t)),Forces_rope_1(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(t)),Forces_rope_1(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(t)),Forces_rope_1(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
grid('minor')
grid on;
legend({'$F_{x}$','$F_{y}$','$F_{z}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N]$','Interpreter','latex','FontSize',9);

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(t)),norm_force(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
grid('minor')
grid on;
legend({'$||F||$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N]$','Interpreter','latex','FontSize',9);

