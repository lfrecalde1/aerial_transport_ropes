%% Code to simulate Aerial vehicle in quadternions
%% Init time
clc, clear all, close all;
%% Set time parameters
ts = 0.001;
t_final = 5;
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

%% Desired Trajectory of the system
phi_d = (15*pi/180)*ones(1, length(t));
theta_d = (-25*pi/180)*ones(1, length(t));
zd = 0*ones(1, length(t));
psid = (0)*ones(1, length(t));

phid_d = 0*ones(1, length(t));
thetad_d = 0*ones(1, length(t));
zd_d = 2*ones(1, length(t));
psid_d = 0.2*ones(1, length(t));

xd_dd = 0*ones(1, length(t));
yd_dd = 0*ones(1, length(t));
zd_dd = 0*ones(1, length(t));

hd = [phi_d;theta_d;zd;psid];
hd_d = [phid_d;thetad_d;zd_d;psid_d];
hd_dd = [xd_dd;yd_dd;zd_dd];

%% PID 
e_pose_i = [0;0;0];
e_angle_i = [0;0];

%% Simulation system
for k = 1:length(t)
    %% Controller section
    [u(:, k), e_pose_i, e_angle_i]= low_level_controller(h(:, k ), hd(:, k), hd_d(:, k), hd_dd(:, k), L_drone,  e_pose_i, e_angle_i, ts);    
    tic
    %% System evolution
    h(:, k+1) = system_simulation(h(:, k), u(:, k), L_drone, ts);
    R(:, :, k+1) = QuatToRot(h(7:10, k+1));
    euler(:, k+1) = quat2eul(h(7:10, k+1)','XYZ');
    euler_d(:, k+1) = euler_dot(euler(:, k+1), h(11:13, k+1));
    R_1(:, :, k+1) = Rot_zyx(euler(:, k+1));
    toc
end

%% Simulation System
close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
Drone_Parameters(0.02);
G2=Drone_Plot_3D(h(1,1),h(2,1),h(3,1),R(:, :, 1));hold on
plot3(h(1,1),h(2,1),h(3,1),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on

axis([-3.5 3.5 -3.5 3.5 -3.5 3.5]);
view(20,15);
for k = 1:200:length(t)-1
    drawnow
    delete(G2);
    G2=Drone_Plot_3D(h(1,k),h(2,k),h(3,k),R(:, :, k));hold on
    plot3(h(1,1:k),h(2,1:k),h(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end

%% System pictures

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,euler(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,euler(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,euler_d(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,h(6,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,hd(1,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,hd(2,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,hd_d(4,1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,hd_d(3,1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$\phi$','$\theta$','$\psi$','$\dot{z}$','$\phi_d$','$\theta_d$','$\psi_d$','$\dot{z}_d$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,2,1)
plot(t,u(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$f$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
subplot(1,2,2)
plot(t,u(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,u(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,u(4,1:length(t)),'Color',[26,50,160]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_{\phi}$','$\tau_{\theta}$','$\tau_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


