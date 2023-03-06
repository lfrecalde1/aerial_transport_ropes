function [h] = system_simulation(h, u, F_ext, L, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = system_dynamics(h, u, F_ext, L);
k2 = system_dynamics(h + ts/2*k1, u, F_ext, L); % new
k3 = system_dynamics(h + ts/2*k2, u, F_ext, L); % new
k4 = system_dynamics(h + ts*k3, u, F_ext, L); % new

h = h +ts/6*(k1 +2*k2 +2*k3 +k4); % new

end