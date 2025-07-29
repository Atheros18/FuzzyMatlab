clear all; close all; clc
%% debug valores
global e_pos_log de_pos_log e_theta_log de_theta_log F_log ref_pos_list  
e_pos_log = [];
de_pos_log = [];
e_theta_log = [];
de_theta_log = [];
F_log = [];
%% definicion de constantes del sistema
    M = 0.5;     % masa del carro(kg)
    m = 0.2;     % masa del pendulo (kg)
    l = 0.3;     % Longitud del pendulo al centro de la masa(m)
    g = 9.81;    % gravedad (m/s^2)
    I = (1/3)*m*l^2;  % momento de inercia del pendulo(rod)
    b1 = 0.1;    % coeficiente de friccion carro
    b2 = 0.05;   % coeficiente de friccion pendulo.
    
    %% paramteros MGA
    params(1) = 110.5893 %130.3592
    params(2) = 27.6783; %26.1267
    params(3) = 52.0960; %34.9811
    params(4) = -29.6620; %-14.8822
    params(5) = -42.5612; %-26.6244
    params(6) = 37.5098; %80.1018

%% Cargar controladores .fis
fis_theta = readfis('fis_theta.fis');
fis_pos   = readfis('fis_pos.fis');
 %% valores iniciales y tiempos de simulacion
tspan = [0 40];  %segundos
y0 = [0, 0, pi-0.1, 0];  % estado inicial [X, X', theta, theta']

ref_theta = pi;
ref_pos_list = [0, 0.2, 0.5, -0.3, -0.6];
% 
%     % Duración de cada tramo (en segundos)
%     duration = 5;
% 
%     % Índice de la referencia actual
%     idx = floor(tspan / duration) + 1;
% 
%     % Limitar al último índice
%     idx = min(idx, length(ref_pos_list));
% 
%     % Retornar la referencia correspondiente
%     ref_pos = ref_pos_list(idx);
% ref_pos = 0.5;

% [t, y] = ode45(@(t,y) pendcart(t, y, M, m, l, g, I, b1, b2,fis_theta,fis_pos, ref_theta, ref_pos), tspan, y0);
 [t, y] = ode45(@(t, y) pendcart(t,y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta), tspan, y0);
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


min_len = min([length(t), length(e_pos_log), length(de_pos_log), length(e_theta_log),length(de_theta_log),length(F_log)]);

figure;
subplot(2,2,1);
plot(t(1:min_len), e_theta_log(1:min_len), 'b');
xlabel('Time (s)');
ylabel('e_{\theta}');
title('Angular Error e_\theta (normalized)');

subplot(2,2,3);
plot(t(1:min_len), e_pos_log(1:min_len), 'r');
xlabel('Time (s)');
ylabel('e_{x}');
title('Positional Error e_x (normalized)');

subplot(2,2,2);
plot(t(1:min_len), de_theta_log(1:min_len), 'r');
xlabel('Time (s)');
ylabel('de_{\theta}');
title('de_{\theta}');

subplot(2,2,4);
plot(t(1:min_len), de_pos_log(1:min_len), 'r');
xlabel('Time (s)');
ylabel('de_pos');
title('de_pos');


figure;
plot(t(1:min_len), F_log(1:min_len), 'r');
xlabel('Time (s)');
ylabel('F');
title('F e_x (normalized)');
% figure;
% subplot(1,2,1);
% gensurf(fis_theta);
% title('Superficie de inferencia - fis\_theta');
% subplot(1,2,2);
% gensurf(fis_pos);
% title('Superficie de inferencia - fis\_pos');


% data_inputs  = [e_pos_log(1:min_len), de_pos_log(1:min_len)];
% data_outputs = F_log(1:min_len);
data_inputs  = [e_pos_log(:), de_pos_log(:)];  % [N x 2]
data_outputs = F_log(:);     
% Guardar a archivo .mat
save('datos_fis_pos.mat', 'data_inputs', 'data_outputs', 't');