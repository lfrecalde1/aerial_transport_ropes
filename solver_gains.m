%% Optimization Solver Gains Matrices
clc, clear all, close all;

%% Time Definition
ts = 0.001;
t_final = 5;
t = (0:ts:t_final);
%% Inital Guess Optimization
X = [1;1;1];

%% Desired Trajectory of the system
xd = 0*ones(1, length(t));
yd = 0*ones(1, length(t));
zd = 0*ones(1, length(t));
psid = (0)*ones(1, length(t));

xd_d = 0*ones(1, length(t));
yd_d = 0*ones(1, length(t));
zd_d = 1*ones(1, length(t));
psid_d = 0*ones(1, length(t));

xd_dd = 0*ones(1, length(t));
yd_dd = 0*ones(1, length(t));
zd_dd = 0*ones(1, length(t));

hd = [xd;yd;zd;psid];
hdp = [xd_d;yd_d;zd_d;psid_d];
hdpp = [xd_dd;yd_dd;zd_dd];

% Optimization parameters
options = optimset('Display','iter',...
                'TolFun', 1e-8,...
                'MaxIter', 10,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-8);   
f_obj1 = @(x)optimization_pid_drone(x, t_final, ts, hd, hdp, hdpp);
% Optimization restriccions

% Limits Gains
LB = [0.1;0.1;0.1];
UB = [10;50;50];

chi = fmincon(f_obj1,X,[],[],[],[],LB,UB,[],options);