clear all; close all; clc

m = 0.2; M = 0.5; L = 0.3; g = -9.81; d = 1;
Kp = 150; Kd = 30;

tspan = 0:0.05:10;
y0 = [0; 0; pi-0.8; 0];
[t, y] = ode45(@(t,y)pendcart_pd(y,m,M,L,g,d,Kp,Kd), tspan, y0);

figure;
subplot(2,1,1);
plot(t, y(:,1), 'LineWidth', 1.5); ylabel('X (m)'); grid on;
subplot(2,1,2);
plot(t, y(:,3)*180/pi, 'LineWidth', 1.5); ylabel('\theta (°)'); xlabel('t (s)'); grid on;
