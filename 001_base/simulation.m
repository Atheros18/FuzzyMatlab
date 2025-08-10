clear all; close all; clc

% params0 = [1.43 1.35 20 1.88 0.00432 28.2]; Fmax = 30; %Best Fitness J = 0.0353
params0 = [2.41 0.756 18.9 0.298 1.34 23]; Fmax = 30; %Best Fitness J = 151
M=0.5; m=0.2; l=0.3; g=9.81; I=(1/3)*m*l^2; b1=0.1; b2=0.05; 

fis_theta = readfis('files_created/fis_theta.fis');
fis_pos   = readfis('files_created/fis_pos.fis');

% y0 = [-0.2; -0.4; +0.6; 0.3];
y0 = [0; 0; 0.1; 0];
tspan = [0 10];
ref_pos = 0;
ref_theta = 0;

opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-3);

            dyn = @(t,y) pendcart_dyn(t,y, M,m,l,g,I,b1,b2, @(yy) fuzzy_control_force(yy, fis_theta, fis_pos, params0, ref_pos, ref_theta,Fmax));

[t,Y] = ode45(dyn, tspan , y0, opts);

subplot(3,1,1), plot(t, Y(:,1)); grid on; ylabel('X (m)')
subplot(3,1,2), plot(t, Y(:,3)*180/pi); grid on; ylabel('\theta (deg)')
subplot(3,1,3), plot(t, Y(:,2)); grid on; ylabel('Xdot (m/s)'), xlabel('time (s)')
