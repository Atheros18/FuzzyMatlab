close all; clear all;clc;
    %-----------------------------
    % Parametros generales
    %-----------------------------
    M = 0.5;       % masa del carro (kg)
    m = 0.2;       % masa del pendulo (kg)
    l = 0.3;       % longitud al centro de masa (m)
    g = 9.81;     % gravedad (m/s^2)
    I = (1/3)*m*l^2;  % momento de inercia del péndulo
    b1 = 0.1;      % friccion del carro
    b2 = 0.05;     % friccion del péndulo

    %-----------------------------
    % Parametros del controlador PD
    %-----------------------------
    Kp = 100;       % Ganancia proporcional
    Kd = 20;        % Ganancia derivativa
    theta_ref = pi;  % Referencia: péndulo en vertical

    %-----------------------------
    % Simulación
    %-----------------------------
    tspan = [0 10];  % tiempo (s)
    y0 = [0, 0, pi-1, 0];  % estado inicial [X, X', theta, theta']

    % Llamada a ODE con controlador
    [t, Y] = ode45(@(t,y) pend_cart_antiguo(t, y, M, m, l, g, I, b1, b2, Kp, Kd, theta_ref), tspan, y0);

    %-----------------------------
    % Plot resultados
    %-----------------------------    
    figure;
    subplot(2,1,1);
    plot(t, Y(:,1), 'LineWidth', 1.5);
    ylabel('Posición del carro X (m)');
    grid on;

    subplot(2,1,2);
    plot(t, Y(:,3)*180/pi, 'LineWidth', 1.5);
    ylabel('Ángulo del péndulo \theta (°)');
    xlabel('Tiempo (s)');
    grid on;
    
