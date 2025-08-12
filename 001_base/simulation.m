clear all; close all; clc

% params0 = [2.2493,4.1911,10.6015, 2.7464,0.77702, 49.714]; Fmax = 15;

params0 = [2.2666, 2.5862, 13.1821, 3.94, 0.60381, 45.4725]; Fmax = 30; %J = 0.27639

M=0.5; m=0.2; l=0.3; g=9.81; I=(1/3)*m*l^2; b1=0.1; b2=0.05; 


fis_pos   = readfis('files_created/fis_pos.fis');
fis_theta = readfis('files_created/fis_theta.fis');

y0 = [0.2; 0; -0.1; 0];
tspan = [0 10];
ref_pos =-0.2;
ref_theta = 0;
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-3, ...
              'Events', @stop_events);   % <— ensure this is ON


dyn = @(t,y) pendcart_dyn(t, y, M,m,l,g,I,b1,b2, fuzzy_control_force(y,params0, fis_pos, fis_theta, ref_pos, ref_theta,Fmax));

[t,Y] = ode45(dyn, tspan , y0, opts);

subplot(2,1,1), plot(t, Y(:,1)); grid on; ylabel('X (m)')
subplot(2,1,2), plot(t, Y(:,3)*180/pi); grid on; ylabel('\theta (deg)')
