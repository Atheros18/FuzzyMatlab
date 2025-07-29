function J = mGA_fitness(x)
    % x = [Ke, Kde, Umax]
    Ke   = x(1);
    Kde  = x(2);
    Umax = x(3);

    % Parámetros del sistema físico
    M = 1; m = 0.1; l = 0.5; g = 9.81;
    I = m * l^2 / 3;
    b1 = 0.1; b2 = 0.05;

    % Condición inicial y tiempo de simulación
    y0 = [0, 0, pi - 0.1, 0];
    tspan = [0 5];

    % Simulación con controlador
    [~, Y] = ode45(@(t, y) pendcart(y, Ke, Kde, Umax, M, m, l, g, I, b1, b2), tspan, y0);
%     [t, y] = ode45(@(t,y) pendcart(t, y, M, m, l, g, I, b1, b2,fis_theta,fis_pos, ref_theta, ref_pos), tspan, y0);

    % Cálculo del ISE (Integral of Squared Error)
    theta_error = Y(:,3) - pi;
    J = trapz(theta_error.^2);  % Costo a minimizar
end
