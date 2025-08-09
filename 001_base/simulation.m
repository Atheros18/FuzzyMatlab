% === Sanity run ===
% params0 = [2, 2, 20, 4, 2, 25]; Fmax=30;   % seed guess
clear all; close all; clc
% params0 = [0,0,0, 1.0,0.6, 10]; Fmax = 15;
% params0 = [0.3,0.2,3, 5.9235,0.7504, 47.2253]; Fmax = 15;
params0 = [2.2493,4.1911,-10.6015, 2.7464,0.77702, 49.714]; Fmax = 15;
% params0 = [0.3, 0.2, 2,   2.0, 0.8, 25];
% Fmax    = 25;

M=0.5; m=0.2; l=0.3; g=9.81; I=(1/3)*m*l^2; b1=0.1; b2=0.05; 

fis_theta = readfis('files_created/fis_theta.fis');
fis_pos   = readfis('files_created/fis_pos.fis');

y0 = [0.2; 0; -0.1; 0];
tspan = [0 10];
ref_pos =-0.2;
ref_theta = 0;
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-3);

dyn = @(t,y) pendcart_dyn(t, y, M,m,l,g,I,b1,b2, ...
        fuzzy_control_force(y, fis_theta, fis_pos, params0, ref_pos, ref_theta,Fmax));

[t,Y] = ode45(dyn, tspan , y0, opts);

subplot(3,1,1), plot(t, Y(:,1)); grid on; ylabel('X (m)')
subplot(3,1,2), plot(t, Y(:,3)*180/pi); grid on; ylabel('\theta (deg)')
subplot(3,1,3), plot(t, Y(:,2)); grid on; ylabel('Xdot (m/s)'), xlabel('time (s)')
