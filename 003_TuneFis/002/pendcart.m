function dydt = pendcart(y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos)
    %% Parámetros del controlador desde el vector 'params'
    k11 = params(1);  % Ganancia de error de posición
    k12 = params(2);  % Ganancia de derivada del error de posición
    k13 = params(3);  % Ganancia de salida del FIS de posición
    k21 = params(4);  % Ganancia de error angular
    k22 = params(5);  % Ganancia de derivada del error angular
    k23 = params(6);  % Ganancia de salida del FIS de ángulo
    w_theta= params(7);
%     k11 = 0;  % Ganancia de error de posición
%     k12 = 0;  % Ganancia de derivada del error de posición
%     k13 = 0;  % Ganancia de salida del FIS de posición
%     k21 = 0;  % Ganancia de error angular
%     k22 = 0;  % Ganancia de derivada del error angular
%     k23 = 0;  % Ganancia de salida del FIS de ángulo
%     global ref_pos_list
%     ref_pos = ref_pos_func(t);
%     w_theta = 0.6;
%     w_pos   = 0.4;
    %% Variables del sistema
    X         = y(1); 
    X_dot     = y(2);
    theta     = y(3);
    theta_dot = y(4);

    %% Errores
    e_x      = ref_pos - X;
    de_x     = -X_dot;
    e_theta  = ref_theta - theta;
    de_theta = -theta_dot;
    %% Entradas escaladas para FIS
    e_x_norm     = max(min( k11 *  e_x   / 3,  0.999), -0.999);
    de_x_norm    = max(min( k12 *  de_x  / 5,  0.999), -0.999);
    in_pos       = [e_x_norm, de_x_norm];
    
    e_theta_norm  = max(min( k21 * e_theta   / pi,  0.999), -0.999);
    de_theta_norm = max(min( k22 * de_theta / 5,   0.999), -0.999);
    in_theta      = [ e_theta_norm, de_theta_norm];


    %% Evaluación de FIS
    
    u_pos_fis   = evalfis(fis_pos, in_pos);
    u_theta_fis = evalfis(fis_theta, in_theta);
%     
%     if any(isnan([u_pos_fis, u_theta_fis])) || all(u_pos_fis == 0) || all(u_theta_fis == 0)
%         dydt = [0; 0; 0; 0];  % Forzar detención de la dinámica
%         return;
%     end
    %% Ganancia de salida
    U_pos   = k13 * u_pos_fis;
    U_theta = k23 * u_theta_fis;

    %% Saturación
    F_max = 30;
    max_theta = w_theta * F_max;
    max_pos   = (1-w_theta) * F_max;
    U_theta = max(min(U_theta,  max_theta), -max_theta);
    U_pos   = max(min(U_pos,    max_pos),   -max_pos);
%     U_pos = 0;
    %% Fuerza total
    F = U_theta + U_pos;
%     F=0;
%     if abs(F) = 30
%         disp("⚠️ Fuerza F extrema: " + F)
%         dydt = [0; 0; 0; 0];
%         return;
%     end
    %% Dinámica del sistema
    D = I + m * l^2;
    theta_ddot_num = -m * l * cos(theta) * (F + m * l * sin(theta) * theta_dot^2 - b1 * X_dot) / (M + m) + m * g * l * sin(theta) - b2 * theta_dot;
    den = (D - (m^2 * l^2 * cos(theta)^2) / (M + m));
    theta_ddot = theta_ddot_num / den;
    X_ddot     = (F - m * l * cos(theta) * theta_ddot + m * l * theta_dot^2 * sin(theta) - b1 * X_dot) / (M + m);

    
    %% debug
%     disp("▶ Denominator = " + den)
%     if t == 0  % o muy cercano a 0
%         disp("▶ F = " + F)
%         disp("▶ theta = " + theta + ", theta_dot = " + theta_dot)
%         disp("▶ theta_ddot = " + theta_ddot)
%         disp("▶ X_ddot = " + X_ddot)
%     end
    %%
    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
end
