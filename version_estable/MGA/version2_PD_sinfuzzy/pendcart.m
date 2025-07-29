function dydt = pendcart(t,y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta)
    %% 
    global e_pos_log de_pos_log e_theta_log de_theta_log F_log ref_pos_list  
    ref_pos = ref_pos_func(t);
    %% Parámetros del controlador desde el vector 'params'
    Ke_theta   = params(1);
    Kde_theta  = params(2);
    Umax_theta = params(3);
    Ke_pos     = params(4);
    Kde_pos    = params(5);
    Umax_pos   = params(6);

    %% Variables del sistema
    X     = y(1); 
    X_dot = y(2);
    theta = y(3);
    theta_dot = y(4);

    %% Errores y normalización
    e_theta  = ref_theta - theta;
    de_theta = -theta_dot;
    e_theta  = max(min(e_theta / pi, 0.999), -0.999);
    de_theta = max(min(de_theta / 5, 0.999), -0.999);

    e_pos  = ref_pos - X;
    de_pos = -X_dot;
    e_pos  = max(min(e_pos / 3, 0.999), -0.999);
    de_pos = max(min(de_pos / 5, 0.999), -0.999);

    %% FIS (no adaptativo)
    u_theta_fis = evalfis(fis_theta, [e_theta, de_theta]);
    u_pos_fis   = evalfis(fis_pos, [e_pos, de_pos]);
    u_pos_fis   = max(min(u_pos_fis, 30), -30);

    %% Control PD ajustado por mGA
    U_theta = Ke_theta * e_theta + Kde_theta * de_theta;
    U_pos   = Ke_pos * e_pos + Kde_pos * de_pos;

    % Saturación
    U_theta = max(min(U_theta, Umax_theta), -Umax_theta);
    U_pos   = max(min(U_pos, Umax_pos), -Umax_pos);

    %% Fuerza total
    F = U_theta + U_pos;
    
    %% LOGS
    e_theta_log(end+1) = e_theta;
    de_theta_log(end+1) = de_theta;
    e_pos_log(end+1) = e_pos;
    de_pos_log(end+1) = de_pos;
    F_log(end+1) = F;
    %% Dinámica
    D = I + m * l^2;
    theta_ddot_num = -m * l * cos(theta) * (F + m * l * sin(theta) * theta_dot^2 - b1 * X_dot) / (M + m) ...
                     - m * g * l * sin(theta) - b2 * theta_dot;
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2) / (M + m));
    X_ddot = (F - m * l * cos(theta) * theta_ddot + m * l * theta_dot^2 * sin(theta) - b1 * X_dot) / (M + m);

    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
end
