function dydt = pendcart(y, Ke, Kde, Umax, M, m, l, g, I, b1, b2)
    X         = y(1);
    X_dot     = y(2);
    theta     = y(3);
    theta_dot = y(4);

    % Controlador tipo PD (centrado en theta = pi)
    e_theta    = wrapToPi(pi - theta);
    de_theta   = -theta_dot;

    % Saturar entradas si es necesario
    e_theta    = max(min(e_theta, 1), -1);
    de_theta   = max(min(de_theta, 1), -1);

    % Control PD
    U = Ke * e_theta + Kde * de_theta;
    U = max(min(U, Umax), -Umax);

    % Ecuaciones del péndulo invertido
    D = I + m * l^2;
    theta_ddot = (-m*l*cos(theta)*(U + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) ...
                  - m*g*l*sin(theta) - b2*theta_dot) ...
                  / (D - (m^2 * l^2 * cos(theta)^2)/(M + m));

    X_ddot = (U + m*l*(theta_dot^2*sin(theta) - theta_ddot*cos(theta)) - b1*X_dot) / (M + m);

    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];