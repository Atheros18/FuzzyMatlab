
clc; close all; rng(42);

%% ====== Physical & robot params ======
pars.M   = 0.5;      % cart mass [kg]
pars.m   = 0.2;      % pendulum mass [kg]
pars.l   = 0.3;      % pendulum length [m]
pars.g   = 9.81;     
pars.I   = (1/3)*pars.m*pars.l^2;   % pendulum inertia about pivot
pars.b_v = 0.1;      % cart viscous friction (forward)
pars.b_r = 0.02;     % yaw viscous friction
pars.R   = 0.05;     % wheel radius [m]
pars.d   = 0.12;     % half-track [m]  (wheel-to-center)
pars.Iz  = 0.02;     % chassis yaw inertia [kg m^2]

% Force / moment limits (and torques derived)
pars.Fmax   = 30;    % N
pars.Mz_max = 5;     % N·m
pars.tau_max = 2;    % N·m  (per wheel)

%% ====== Fuzzy/PD gains (placeholders; you can GA-tune) ======
pars.Ke_pos  = 1.5;  pars.Kde_pos = 0.8;  pars.Umax_pos = 10;
pars.Ke_th   = 3.0;  pars.Kde_th  = 0.8;  pars.Umax_th  = 25;

% Normalization bounds (replace magic numbers /3 /5 /pi /8)
pars.EX_MAX  = 3;      % [m]
pars.DX_MAX  = 5;      % [m/s]
pars.DTH_MAX = 8;      % [rad/s]  (for theta_dot)

% Yaw PD (can be fuzzy later)
pars.Kyaw_e = 3.0;   % [N·m / rad]
pars.Kyaw_d = 0.5;   % [N·m / (rad/s)]

%% ====== Load FIS (or make defaults if not present) ======
fis_theta = readfis('files_created/fis_theta.fis');
fis_pos   = readfis('files_created/fis_pos.fis');

%% ====== References (turn left 30° after 2s; hold x=0; upright θ=0) ======
ref_fun = @(t) struct('x',0,'psi', (t>=2)*deg2rad(30), 'th',0);

%% ====== Initial state ======
% y = [x; yW; psi; v; r; theta; thetadot]
y0 = [0; 0; 0; 0; 0; 0.2; 0];  % start near-upright

%% ====== ODE solve ======
ctrl = @(t,y) fuzzy_control_drive(t,y,fis_pos,fis_theta,pars,ref_fun);  % returns [tauL,tauR,F_cmd,Mz_cmd]
dyn  = @(t,y) pendcart_dyn_2d(t,y,pars,ctrl);

tspan = [0 10];
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.01,'Events',@(t,y) stop_events_2d(t,y,pars));
[t,Y,te] = ode45(dyn,tspan,y0,opts); %#ok<ASGLU>

% Recompute control histories for plotting
n = numel(t); tauL = zeros(n,1); tauR=zeros(n,1); Fcmd=zeros(n,1); Mzc=zeros(n,1);
for k=1:n
    [tauL(k),tauR(k),Fcmd(k),Mzc(k)] = ctrl(t(k),Y(k,:).');
end

%% ====== Plots ======
figure('Name','Cart-Pendulum 2D');
subplot(3,2,1); plot(t,Y(:,1)); grid on; ylabel('x [m]'); xlabel('t [s]');
title('Longitudinal position');

subplot(3,2,2); plot(t,Y(:,2)); grid on; ylabel('y [m]'); xlabel('t [s]');
title('Lateral position');

subplot(3,2,3); plot(t,rad2deg(Y(:,3))); grid on; ylabel('\psi [deg]'); xlabel('t [s]');
title('Yaw');

subplot(3,2,4); plot(t,rad2deg(Y(:,6))); grid on; ylabel('\theta [deg]'); xlabel('t [s]');
title('Pendulum angle (0 = upright)');

subplot(3,2,5); plot(t,Fcmd); grid on; ylabel('F_{cmd} [N]'); xlabel('t [s]');
title('Forward force command');

subplot(3,2,6); plot(t,tauL, t,tauR); grid on; ylabel('\tau_L, \tau_R [N·m]'); xlabel('t [s]');
legend('\tau_L','\tau_R','Location','best'); title('Wheel torques');

figure('Name','Trajectory'); plot(Y(:,1),Y(:,2),'LineWidth',1.5);
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); title('Planar path');

if ~isempty(te)
    fprintf('Simulation stopped early (safety event) at t = %.3f s\n', te(end));
end
