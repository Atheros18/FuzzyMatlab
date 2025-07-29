function dydt = dinamica_sistema(t, y, M, m, l, g, I, b1, b2, fis_theta, fis_x)

    X = y(1);
    X_dot = y(2);
    theta = y(3);
    theta_dot = y(4);

    % Errors
    e_theta = pi - theta;
    de_theta = -theta_dot;
    X_ref = 5;
    e_x = X_ref - X;
    de_x = -X_dot;
    %
    
    % Normalize inputs to [-1, 1]
    e_theta_n  = max(min(e_theta, 1), -1);
    de_theta_n = max(min(de_theta, 1), -1);
    e_x_n      = max(min(e_x, 1), -1);
    de_x_n     = max(min(de_x, 1), -1);
    
    F_theta = -evalfis(fis_theta, [e_theta_n, de_theta_n]);
    F_x     =  evalfis(fis_x, [e_x_n, de_x_n]);
    %F_theta     =  0;
    F = F_theta + F_x; % suma fuerzas

    % Dinámica
    D = I + m * l^2;
    theta_ddot_num = -m*l*cos(theta)*(F + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) - m*g*l*sin(theta) - b2*theta_dot;           
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2)/(M + m));

    % X''
    X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);
    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
    
    global F_storage t_storage;
    F_storage(end+1) = F;
    t_storage(end+1) = y(1);  % use time tracking, optional fallback


end