function [euler_d] = euler_dot(euler, omega)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%% Split values of the system
psi = euler(3);
theta =euler(2);
phi = euler(1);

S = [1, 0, -sin(theta);...
     0 ,  cos(phi), sin(phi)*cos(theta);...
     0, -sin(phi), cos(phi)*cos(theta)];
 
euler_d = inv(S)*omega;
end

