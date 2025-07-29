function dydt = pendcart(y,params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos)
    %% Parámetros del controlador desde el vector 'params'
    k11 = params(1);  % Ganancia de error de posición
    k12 = params(2);  % Ganancia de derivada del error de posición
    k13 = params(3);  % Ganancia de salida del FIS de posición
    k21 = params(4);  % Ganancia de error angular
    k22 = params(5);  % Ganancia de derivada del error angular
    k23 = params(6);  % Ganancia de salida del FIS de ángulo
% k11,k12,k13,k21,k22,k23
global data_log_pos

if any(abs(y) > 1000) || any(isnan(y)) || any(isinf(y))
    dydt = [0; 0; 0; 0];
    return;
end
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
    in_pos = [max(min(k11 * e_x/3,  0.999), -0.999),max(min(k12 * de_x/5, 0.999), -0.999)];
    in_theta = [max(min(k21 * e_theta/pi,  0.999), -0.999),max(min(k22 * de_theta/5, 0.999), -0.999)];

    %% Evaluación de FIS
    u_pos_fis   = evalfis(fis_pos, in_pos);
    u_theta_fis = evalfis(fis_theta, in_theta);

    %% Ganancia de salida
    U_pos   = k13 * u_pos_fis;
    U_theta = k23 * u_theta_fis;

    input_pos = in_pos;       % tamaño 1x2
    output_pos = u_pos_fis;   % salida del FIS antes de multiplicar por k13

% Guardar entrada/salida
    data_log_pos = [data_log_pos; input_pos, output_pos];
    %% Saturación
    U_pos   = max(min(U_pos,  30), -30);
    U_theta = max(min(U_theta, 30), -30);

    %% Fuerza total
    F = U_theta + U_pos;
%     F = max(min(F, Umax_pos + Umax_theta), - (Umax_pos + Umax_theta));  % saturación final
    %% Dinámica del sistema
    D = I + m * l^2;
    theta_ddot_num = -m * l * cos(theta) * (F + m * l * sin(theta) * theta_dot^2 - b1 * X_dot) / (M + m) - m * g * l * sin(theta) - b2 * theta_dot;
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2) / (M + m));
    X_ddot     = (F - m * l * cos(theta) * theta_ddot + m * l * theta_dot^2 * sin(theta) - b1 * X_dot) / (M + m);

    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
end
