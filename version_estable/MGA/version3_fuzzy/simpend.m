clear all; close all; clc
%% debug valores
global error_theta_log error_pos_log U_pos_log U_theta_log
error_theta_log = [];
error_pos_log = [];
U_pos_log = [];
U_theta_log = [];
%% definicion de constantes del sistema
    M = 0.5;     % masa del carro(kg)
    m = 0.2;     % masa del pendulo (kg)
    l = 0.3;     % Longitud del pendulo al centro de la masa(m)
    g = 9.81;    % gravedad (m/s^2)
    I = (1/3)*m*l^2;  % momento de inercia del pendulo(rod)
    b1 = 0.1;    % coeficiente de friccion carro
    b2 = 0.05;   % coeficiente de friccion pendulo.
    
    %% paramteros MGA
    params(1) = 130.3592;
    params(2) = 26.1267;
    params(3) = 34.9811;
    params(4) = -14.8822;
    params(5) = -26.6244;
    params(6) = 80.1018;

%% Cargar controladores .fis
fis_theta = readfis('fis_theta.fis');
fis_pos   = readfis('fis_pos.fis');
 %% valores iniciales y tiempos de simulacion
tspan = [0 40];  %segundos
y0 = [0, 0, pi-0.3, 0];  % estado inicial [X, X', theta, theta']

ref_theta = pi;
ref_pos = 0.25;

% [t, y] = ode45(@(t,y) pendcart(t, y, M, m, l, g, I, b1, b2,fis_theta,fis_pos, ref_theta, ref_pos), tspan, y0);
 [t, y] = ode45(@(t, y) pendcart(y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0);
%% Gráfica
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
