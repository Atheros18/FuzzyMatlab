clear all, close all, clc

m = 0.2;%1
M = 0.5;%5
L = 0.3;%2
g = -9.81;%-10
d = 1;%1

%     M = 0.5;     % masa del carro(kg)
%     m = 0.2;     % masa del pendulo (kg)
%     l = 0.3;     % Longitud del pendulo al centro de la masa(m)
%     g = 9.81;    % gravedad (m/s^2)
%     I = (1/3)*m*l^2;  % momento de inercia del pendulo(rod)
%     b1 = 0.1;    % coeficiente de friccion carro
%     b2 = 0.05;   % coeficiente de friccion pendulo.

tspan = 0:.1:40;
y0 = [0; 0; pi-0.1; .5];
[t,y] = ode45(@(t,y)pendcart(y,m,M,L,g,d,0),tspan,y0);


    figure;
    subplot(2,1,1);
    plot(t, y(:,1), 'LineWidth', 1.5);
    ylabel('Posicion del carro X (m)');
    grid on;

    subplot(2,1,2);
    plot(t, y(:,3)*180/pi, 'LineWidth', 1.5);
    ylabel('Angulo del pendulo \theta ');
    xlabel('Tiempo (s)');
    grid on;
    
% for k=1:length(t)
%     drawpend(y(k,:),m,M,L);
% end
