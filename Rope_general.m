%% Code to simulate a rope
clc, clear all, close all;
%% Set time parameters
ts = 0.001;
t_final = 10;
t = (0:ts:t_final);

%% Particles Definition
m = 0.015;
fm = 0.5;
nr = 1;
nm = 30;
%% Spring Length
spring_length = 0.05;
spring_k = 3000;
damper_k = 3;

%% Gravitational Vector
g = 9.8;
wind_drag = 0.03;

%% Angle
alpha = 45*(pi/180);

%% Vector with all the parameters
L = [g; spring_length; spring_k; damper_k; wind_drag; ts; alpha; length(t)];

%% Ropes Definition
Matrix_ropes = Rope(nr, nm, m, fm, L);

for k = 1:length(t)
    
    % get system Information
    tic
    Matrix_ropes.f_system_ropes(k+1);
    toc
end

%% Video
myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 100;  %can adjust this, 5 - 10 works well for me
open(myVideo)
for k = 1:20:length(t)
    tic
    drawpend_ropes(Matrix_ropes, k)
    toc
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)