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
    params(1) = 7.5657;
    params(2) = 0.3665;
    params(3) = 74.3359;
    params(4) = 5.9235;
    params(5) = 0.7504;
    params(6) = 47.2253;
    
%     params(1) = 17.8486;
%     params(2) = 0.4330;
%     params(3) = 57.9401;
%     params(4) = 71.6748;
%     params(5) = 18.8881;
%     params(6) = 40.7262;
% % % % params(1) = 4.9265;
% % % % params(2) = 4.9177;
% % % % params(3) = 3.5341;
% % % % params(4) = 0.0156;
% % % % params(5) = 2.2080;
% % % % params(6) = 0.2344;

% params(1) = 2.3449;
% params(2) = 0.6320;
% params(3) = 20.3712;
% params(4) = 3.7986;
% params(5) = 3.9075;
% params(6) = 1.4040;


% params(1) = 3.0512;
% params(2) = 3.9457;
% params(3) = 9.3254;
% params(4) = 0.7258;
% params(5) = 4.9376;
% params(6) = 19.1497;

%% Cargar controladores .fis
fis_theta = readfis('files_created/fis_theta.fis');
fis_pos   = readfis('files_created/fis_pos.fis');
 %% valores iniciales y tiempos de simulacion
tspan = [0 10];  %segundos
y0 = [1, -1, -0.2, 0];  % estado inicial [X, X', theta, theta']

ref_pos = 0;
ref_theta = 0;


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
%%
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

% figure;
% subplot(1,2,1);
% gensurf(fis_theta);
% title('Superficie de inferencia - fis\_theta');
% subplot(1,2,2);
% gensurf(fis_pos);
% title('Superficie de inferencia - fis\_pos');
