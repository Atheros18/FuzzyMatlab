function dydt = dinamica_sistema_fuzzy(~, y, M, m, l, g, I, b1, b2, fis_theta,fis_x)
    X = y(1);
    X_dot = y(2);
    theta = y(3);
    theta_dot = y(4);

    % Error centrado en pi
    e_theta = -pi + theta;
    de_theta = -theta_dot;

    %
    F_theta = -evalfis([e_theta, de_theta], fis_theta);

    % Control posición del carro
    e_x = 2-X;
    de_x = -X_dot;
    F_x = evalfis([e_x, de_x], fis_x);

    % Fuerza total
    F = F_theta + F_x;

    % Dinámica
    D = I + m * l^2;
    theta_ddot_num = -m*l*cos(theta)*(F + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) - m*g*l*sin(theta) - b2*theta_dot;           
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2)/(M + m));

    % X''
    X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);

    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
end 