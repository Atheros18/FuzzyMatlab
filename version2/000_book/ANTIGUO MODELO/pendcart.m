function dydt = pendcart(~, y, M, m, l, g, I, b1, b2, fis_theta,fis_pos, theta_ref, pos_ref)
    % Variables del estado
    X = y(1);
    X_dot = y(2);
    theta = y(3);
    theta_dot = y(4);

    
    
    
    
    %%%%%%%%%%%%%%%%%
    
    e_pos = pos_ref - X;
    de_pos = -X_dot;
%      e_pos  = max(min(e_x/8, 1), -1);
%     de_pos = max(min(e_x_dot/10, 1), -1);
    %e_x     = X / 3;
    %e_x_dot = X_dot / 5;

    % Apply fuzzy controllers
    u_pos     = evalfis(fis_pos, [e_pos, de_pos]);
    % Positional control is disabled
    %u_pos = 0;
    
    %%%%%%%%%%%%%%%%%
    % Control PD sobre el ángulo theta
e_theta = wrapToPi(-theta + theta_ref);  % error relativo a π, centrado en 0
de_theta = -theta_dot;
    u_theta = evalfis(fis_theta, [e_theta, de_theta]);
    %F = Kp * e_theta + Kd * e_theta_dot;
    %u_pos = 0;
    F = u_theta + u_pos;
    % Dinámica
    D = I + m*l^2;
    theta_ddot_num = -m*l*cos(theta)*(F + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) - m*g*l*sin(theta) - b2*theta_dot;
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2)/(M + m));
    X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);

    % Salida
    dydt = zeros(4,1);
    dydt(1) = X_dot;
    dydt(2) = X_ddot;
    dydt(3) = theta_dot;
    dydt(4) = theta_ddot;
end
