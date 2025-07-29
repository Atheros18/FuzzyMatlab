function simulacion_lqr()
    % Parameters
    M = 0.5;      % mass of the cart
    m = 0.2;      % mass of the pendulum
    l = 0.3;      % length to pendulum center of mass
    g = 9.81;     % gravity
    I = (1/3)*m*l^2;  % moment of inertia
    b1 = 0.1;     % cart friction
    b2 = 0.05;    % pendulum friction

    % Linearized system matrices around theta = pi
    D = I + m*l^2;
    den = D*(M + m) - (m*l)^2;

    A = [0 1 0 0;
         0 -b1*D/den  m^2*g*l^2/den  -b2*m*l/den;
         0 0 0 1;
         0 b1*m*l/den -m*g*l*(M + m)/den  -b2*(M + m)/den];

    B = [0;
         D/den;
         0;
        -m*l/den];

    % LQR weights
    Q = diag([10, 1, 100, 1]);  % higher weight on angle
    R = 0.01;

    % Compute LQR gain
    K = lqr(A, B, Q, R);

    % Initial state: near upright
    y0 = [0, 0, pi-0.1, 0];  % [X, X_dot, theta, theta_dot]

    % Time span
    tspan = [0 10];

    % Simulate
    [t, Y] = ode45(@(t, y) dinamica_lqr(t, y, M, m, l, g, I, b1, b2, K), tspan, y0);

    % Plot
    figure;
    subplot(2,1,1);
    plot(t, Y(:,1), 'LineWidth', 1.5);
    ylabel('Cart Position X (m)');
    grid on;

    subplot(2,1,2);
    plot(t, (Y(:,3))*180/pi, 'LineWidth', 1.5);
    ylabel('Pendulum Angle θ (° from upright)');
    xlabel('Time (s)');
    grid on;
end
