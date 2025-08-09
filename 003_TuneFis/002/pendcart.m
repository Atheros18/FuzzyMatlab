function dydt = pendcart(y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos)

    %% Control parameters
    k11 = params(1);  % Ke_pos
    k12 = params(2);  % Kde_pos
    k13 = params(3);  % Umax_pos
    k21 = params(4);  % Ke_theta
    k22 = params(5);  % Kde_theta
    k23 = params(6);  % Umax_theta

    %% System states
    X         = y(1);
    X_dot     = y(2);   
    theta     = y(3);
    theta_dot = y(4);

    %% Error calculations
    e_x      = ref_pos - X;
    de_x     = -X_dot;
    e_theta = wrapToPi(ref_theta - theta);
%     e_theta  = ref_theta - theta;
    de_theta = -theta_dot;

    %% Normalized inputs for FIS
    e_x_norm     = max(min(k11 * e_x     / 3,   0.999), -0.999);
    de_x_norm    = max(min(k12 * de_x    / 5,   0.999), -0.999);
    in_pos       = [e_x_norm, de_x_norm];

    e_theta_norm  = max(min(k21 * e_theta  / pi, 0.999), -0.999);
    de_theta_norm = max(min(k22 * de_theta / 5,  0.999), -0.999);
    in_theta      = [e_theta_norm, de_theta_norm];

    %% Evaluate FIS outputs
    u_pos_fis   = evalfis(fis_pos, in_pos);
    u_theta_fis = evalfis(fis_theta, in_theta);

    %% Output scaling
    U_pos   = k13 * u_pos_fis;
    U_theta = k23 * u_theta_fis;

    %% Optional: individual saturation for debugging
    F_max = 100;
    w_theta = 0.9;
    max_theta = w_theta * F_max;
    max_pos   = (1 - w_theta) * F_max;
    U_theta = max(min(U_theta,  max_theta), -max_theta);
    U_pos   = max(min(U_pos,    max_pos),   -max_pos);
    

    %% Store control actions globally
    global Utheta_global Upos_global
    Utheta_global(end+1) = U_theta;
    Upos_global(end+1)   = U_pos;

    %% Total force with saturation
%     if abs(e_theta) < deg2rad(10) && abs(de_theta) < deg2rad(5)
%         F_ = U_theta + U_pos;
%     else
%         F_ = U_theta;  % Solo mantener equilibrio angular
%     end
    F = U_theta + U_pos;
%     F  = max(min(F_, F_max), -F_max);

    %% System dynamics
    D = I + m * l^2;
    theta_ddot_num = -m * l * cos(theta) * (F + m * l * sin(theta) * theta_dot^2 - b1 * X_dot) / (M + m) ...
                     + m * g * l * sin(theta) - b2 * theta_dot;
    den = D - (m^2 * l^2 * cos(theta)^2) / (M + m);
    theta_ddot = theta_ddot_num / den;

    X_ddot = (F - m * l * cos(theta) * theta_ddot + m * l * theta_dot^2 * sin(theta) - b1 * X_dot) / (M + m);

    %% Output state derivatives
    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
end
