clear all; close all; clc

% Parámetros
m = 0.2; M = 0.5; L = 0.3; g = -9.81; d = 1;


% Cargar FIS
fis_theta = readfis('fis_theta_7mf - Copy (2).fis');
fis_pos   = readfis('fis_pos_7mf - Copy (2).fis');

% Simulación
tspan = 0:0.5:20;
ref_theta = pi;
ref_pos = 0.05;
y0 = [0; 0; pi-0.2; 0];

global error_theta_log error_pos_log U_pos_log U_theta_log
error_theta_log = [];
error_pos_log = [];
U_pos_log = [];
U_theta_log = [];

[t, y] = ode45(@(t,y)pendcart(y,m,M,L,g,d,fis_theta,fis_pos,ref_theta,ref_pos), tspan, y0);

% Gráfica
figure;
subplot(2,1,1);
plot(t, y(:,1), 'LineWidth', 1.5);
ylabel('Posición del carro X (m)');
grid on;

subplot(2,1,2);
plot(t, y(:,3)*180/pi, 'LineWidth', 1.5);
ylabel('Ángulo del péndulo \theta (°)');
xlabel('Tiempo (s)');
grid on;


min_len = min([length(t), length(error_theta_log), length(error_pos_log), length(U_pos_log),length(U_theta_log)]);

figure;
subplot(2,2,1);
plot(t(1:min_len), error_theta_log(1:min_len), 'b');
xlabel('Time (s)');
ylabel('e_{\theta}');
title('Angular Error e_\theta (normalized)');

subplot(2,2,3);
plot(t(1:min_len), error_pos_log(1:min_len), 'r');
xlabel('Time (s)');
ylabel('e_{x}');
title('Positional Error e_x (normalized)');

subplot(2,2,2);
plot(t(1:min_len), U_theta_log(1:min_len), 'r');
xlabel('Time (s)');
ylabel('u theta');
title('U theta');

subplot(2,2,4);
plot(t(1:min_len), U_pos_log(1:min_len), 'r');
xlabel('Time (s)');
ylabel('U pos');
title('U pos');

figure;
subplot(1,2,1);
gensurf(fis_theta);
title('Superficie de inferencia - fis\_theta');
subplot(1,2,2);
gensurf(fis_pos);
title('Superficie de inferencia - fis\_pos');
