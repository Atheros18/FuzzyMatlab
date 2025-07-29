function dydt = dinamica_sistema(~, y, M, m, l, g, I, b1, b2)
    % variables
    X = y(1);
    X_dot = y(2);
    theta = y(3);
    theta_dot = y(4);

    % Fuerza aplicada.
    F = 0;

    % expresion
    D = I + m*l^2;
    
    % Thetha''
    theta_ddot_num = -m*l*cos(theta)*(F + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) - m*g*l*sin(theta) - b2*theta_dot;           
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2)/(M + m));

    % X''
    X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);

    % regresa el vector derivado
    dydt = zeros(4,1);
    dydt(1) = X_dot;
    dydt(2) = X_ddot;
    dydt(3) = theta_dot;
    dydt(4) = theta_ddot;
end