clear all; close all; clc

% Parámetros
m = 0.2; M = 0.5; L = 0.3; g = -9.81; d = 1;

% Cargar FIS
fis_theta = readfis('fis_theta.fis');
fis_pos   = readfis('fis_pos.fis');

% Simulación
tspan = 0:0.05:7;
y0 = [0; 0; pi-0.1; 0];
[t, y] = ode45(@(t,y)pendcart(y,m,M,L,g,d,fis_theta,fis_pos), tspan, y0);

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
