function dydt = pendcart(t,y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos)
% PENDCART – Dual FIS PD controller with angle-priority gating
% y = [X; X_dot; theta; theta_dot];  theta = 0 is upright

    %% === Tunable scaling factors (from GA) ===
    k11 = params(1);  % Ke_pos
    k12 = params(2);  % Kde_pos
    k13 = params(3);  % Umax_pos (output scale)
    k21 = params(4);  % Ke_theta
    k22 = params(5);  % Kde_theta
    k23 = params(6);  % Umax_theta (output scale)

    %% === States ===
    X         = y(1);
    X_dot     = y(2);
    theta     = y(3);
    theta_dot = y(4);

    %% === Errors (upright target at 0 rad) ===
    e_x      = ref_pos - X;
    de_x     = -X_dot;
    e_theta  = wrapToPi(ref_theta - theta);
    de_theta = -theta_dot;

    %% === Normalization to [-1,1] before FIS ===
    ex_n   = max(min(k11 * (e_x   / 3),   0.999), -0.999);   % ~±3 m span
    dex_n  = max(min(k12 * (de_x  / 5),   0.999), -0.999);   % ~±5 m/s span
    eth_n  = max(min(k21 * (e_theta / pi),0.999), -0.999);   % ±pi rad span
    deth_n = max(min(k22 * (de_theta/ 5), 0.999), -0.999);   % ~±5 rad/s span

    %% === FIS evaluations ===
    u_pos_fis   = evalfis(fis_pos,   [ex_n,  dex_n]);
    u_theta_fis = evalfis(fis_theta, [eth_n, deth_n]);

    %% === Output scaling ===
    U_pos   = k13 * u_pos_fis;
    U_theta = k23 * u_theta_fis;

    %% === Angle-priority gating of position command ===
% --- priority gating (angle inner loop) ---
theta_thr = deg2rad(10);               % 12–20° is fine
omega_thr = deg2rad(40);               % also gate on angular rate
if (abs(e_theta) > theta_thr) || (abs(de_theta) > omega_thr)
    U_pos = 0;                % position loop OFF while angle is large/fast
end
alpha_theta = 1 - min(1, abs(e_theta)/theta_thr);
alpha_omega = 1 - min(1, abs(de_theta)/omega_thr);
alpha = max(0, min(1, min(alpha_theta, alpha_omega)));   % 0..1
U_pos = alpha * U_pos;

% --- startup ramp to avoid first-step kick (200 ms) ---
ramp = min(1, t/0.2);
U_theta = ramp * U_theta;

% --- single actuator saturation ---
F_max = 100;                              % give room; GA will tame it
F = U_theta + U_pos;
F = max(min(F, F_max), -F_max);


%% === Plant dynamics (signs aligned with reference) ===
D = I + m*l^2;
num = -m*l*cos(theta) * (F + m*l*sin(theta)*theta_dot^2 - b1*X_dot) / (M + m) ...
      + m*g*l*sin(theta) - b2*theta_dot;   % gravedad positiva para θ=0 arriba

den = D - (m^2*l^2*cos(theta)^2)/(M + m);
theta_ddot = num / den;

X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);

dydt = [X_dot; X_ddot; theta_dot; theta_ddot];

    %% === Lightweight logging for debugging ===
    persistent t_last; if isempty(t_last), t_last = 0; end
    global error_theta_log error_pos_log U_pos_log U_theta_log F_log
    if isempty(error_theta_log), error_theta_log = []; end
    if isempty(error_pos_log),   error_pos_log   = []; end
    if isempty(U_pos_log),       U_pos_log       = []; end
    if isempty(U_theta_log),     U_theta_log     = []; end
    if isempty(F_log),           F_log           = []; end

    % Log sparsely to reduce overhead
    if true || (t_last > 0)
        error_theta_log(end+1,1) = eth_n; %#ok<AGROW>
        error_pos_log(end+1,1)   = ex_n;  %#ok<AGROW>
        U_pos_log(end+1,1)       = U_pos; %#ok<AGROW>
        U_theta_log(end+1,1)     = U_theta; %#ok<AGROW>
        F_log(end+1,1)           = F;     %#ok<AGROW>
    end
    t_last = t_last + 1; %#ok<NASGU>
end