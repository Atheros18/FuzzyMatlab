function simulacion_fuzzy_IP()
    %% STEP 1: Create Controllers
    clear all;close all;clc;
    [fis_theta, fis_x] = crear_fis_controladores();  % Updated FIS generation
    %% STEP 2: System Parameters
    M = 0.5; %0.5
    m = 0.2;%0.2
    l = 0.3;%0.3
    g = 9.81;%9.81
    I = (1/3)*m*l^2;
    b1 = 0.1;%0.1
    b2 = 0.05;%0.05

    %% STEP 3: Simulation
    tspan = [0 100];
    y0 = [0, 0, pi+0.1, 0];  % [X, X_dot, theta, theta_dot]
    global F_storage t_storage;
    F_storage = [];
    t_storage = [];

    [t, Y] = ode45(@(t,y) dinamica_sistema(t, y, M, m, l, g, I, b1, b2, fis_theta, fis_x), tspan, y0);

    %% graficamos la fuerza
    figure;
    plot(t_storage, F_storage, 'LineWidth', 1.5);
    ylabel('Control Force F (N)');
    xlabel('Time (s)');
    title('Control Action Over Time');
    grid on;

    %% STEP 4: Plot Results
    figure;
    subplot(2,1,1);
    plot(t, Y(:,1), 'LineWidth', 1.5);
    ylabel('Cart Position X (m)');
    title('Cart Position vs Time');
    grid on;

    subplot(2,1,2);
    plot(t, Y(:,3)*180/pi, 'LineWidth', 1.5);
    ylabel('Pendulum Angle \theta (°)');
    xlabel('Time (s)');
    title('Pendulum Angle vs Time');
    grid on;
end