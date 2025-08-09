clear; close all; clc

% Physical params
M=0.5; m=0.2; l=0.3; g=9.81; I=(1/3)*m*l^2; b1=0.1; b2=0.05;

% Controllers
fis_theta = readfis('files_created/fis_theta.fis');
fis_pos   = readfis('files_created/fis_pos.fis');

% Seed that behaves
params = [1.0 0.5 15,  2.0 0.6 18];

% Sim setup
ref_pos=0; ref_theta=0;
tspan=[0 8];
y0=[ 0.2, 0,  0.25, 0];   % small position offset + angle offset

opts=odeset('RelTol',1e-3,'AbsTol',1e-4,'MaxStep',0.02);

[t,y]=ode45(@(t,y) pendcart(y, params, M,m,l,g,I,b1,b2, ...
                 fis_theta,fis_pos, ref_theta,ref_pos), tspan, y0, opts);

figure;
subplot(2,1,1); plot(t,y(:,1),'LineWidth',1.5); grid on
ylabel('X (m)'); title('Cart position')
subplot(2,1,2); plot(t,y(:,3)*180/pi,'LineWidth',1.5); grid on
ylabel('\theta (deg)'); xlabel('Time (s)'); title('Pendulum angle')

% sanity: angles should settle near 0°, X near 0 m, no crazy chattering
