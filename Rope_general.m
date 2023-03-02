%% Code to simulate a rope
clc, clear all, close all;
%% Set time parameters
ts = 0.001;
t_final = 20;
t = (0:ts:t_final);

%% Time auxiliar
t_init = (0:ts:5);
%% Particles Definition
m = 0.003;
fm = 0.5;
nr = 1;
nm = 50;
%% Spring Length
spring_length = 0.01;
spring_k = 100;
damper_k = 0.2;

%% Gravitational Vector
g = 9.8;
wind_drag = 0.02;

%% Angle
alpha = 90*(pi/180);

%% Vector with all the parameters
L = [g; spring_length; spring_k; damper_k; wind_drag; ts; alpha; length(t)];

%% Ropes Definition
Matrix_ropes = Rope(nr, nm, m, fm, L);

%% Velociy Of the first Mass
v_extern = zeros(3, length(t));

%% Aux time variable 

v_extern(1, :) = 0.0*sin(1*t);
v_extern(2, :) = 0.0*cos(1*t);
v_extern(3, :) = 0;

t_aux_2 = (t >= 10) & (t <= t_final);
v_extern(1, t_aux_2) = 0.0*cos(1*t(t_aux_2));
v_extern(2, t_aux_2) = 0.0*sin(1*t(t_aux_2));
v_extern(3, t_aux_2) = 0; 

%% Init System
% for k = 1:length(t_init)
%     Matrix_ropes.system_forces();
%     Matrix_ropes.f_system_ropes(2);
% end

%% Simulation System
for k = 1:length(t)
    % Apply velocity to the system
    Matrix_ropes.apply_Velocity(v_extern(:, k));
    % Get system Information
    tic
    Matrix_ropes.system_forces();
    [Forces_rope(:, k), norm_force(k)]= Matrix_ropes.get_total_force();
    Matrix_ropes.f_system_ropes(k+1);
    toc;
end

close all; paso=1; 
%a) Parámetros del cuadro de animación
figure
set(gcf,'Position',[300 200 800 700])
myVideo = VideoWriter('myVideoFile'); 
myVideo.FrameRate = 10;  
open(myVideo)
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
grid minor;
rope_1_body = plot3(Matrix_ropes.data(1,:,1,2), Matrix_ropes.data(2,:,1,2), Matrix_ropes.data(3,:,1,2),'.-','Color',[56,171,217]/255,'linewidth',5*Matrix_ropes.m);
rope_1_final = plot3(Matrix_ropes.data(1,end,1,2), Matrix_ropes.data(2,end,1,2), Matrix_ropes.data(3,end,1,2),'o','Color',[100,100,100]/255,'linewidth',5*Matrix_ropes.fm);
view([0 90])
for k = 2:50:length(t)
    drawnow;
    delete(rope_1_body);
    delete(rope_1_final);
    rope_1_body = plot3(Matrix_ropes.data(1,:,1,k), Matrix_ropes.data(2,:,1,k), Matrix_ropes.data(3,:,1,k),'.-','Color',[56,171,217]/255,'linewidth',5*Matrix_ropes.m); hold on;
    rope_1_final = plot3(Matrix_ropes.data(1,end,1,k), Matrix_ropes.data(2,end,1,k), Matrix_ropes.data(3,end,1,k),'o','Color',[100,100,100]/255,'linewidth',5*Matrix_ropes.fm); hold on;
    title('$\textrm{Movement Executed by the Aerial Robots}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    axis([-3.5 3.5 -3.5 3.5 -3.5 3.5]);
    %grid minor;
    view([0 90]);
    %% Video Save
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);

end
close(myVideo)

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t(1:length(Forces_rope)),Forces_rope(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
plot(t(1:length(Forces_rope)),Forces_rope(2,:),'Color',[46,188,89]/255,'linewidth',1); hold on;
plot(t(1:length(Forces_rope)),Forces_rope(3,:),'Color',[26,115,160]/255,'linewidth',1);hold on;
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
plot(t(1:length(Forces_rope)),norm_force(1,:),'Color',[226,76,44]/255,'linewidth',1); hold on;
grid('minor')
grid on;
legend({'$||F||$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[N]$','Interpreter','latex','FontSize',9);