function dydt = pendcart(~, y, Ke,Kde,Umax, M, m, l, g, I, b1, b2, fis_theta, fis_pos ,ref_theta,ref_pos)
    %% definicion de variables
    X     = y(1); 
    X_dot = y(2);
    theta = y(3);
    theta_dot = y(4);
    %% Definicion de controladores
    e_theta  = ref_theta-theta;
    de_theta = -theta_dot;
    e_theta  = max(min(e_theta/pi, 0.999), -0.999);
    de_theta = max(min(de_theta/5, 0.999), -0.999);
    
    e_pos = ref_pos-X;
    de_pos = -X_dot;
    e_pos  = max(min(e_pos/3, 0.999), -0.999);
    de_pos = max(min(de_pos/5, 0.999), -0.999);
    
    u_theta = evalfis(fis_theta, [e_theta, de_theta]);
    u_pos   = evalfis(fis_pos, [e_pos, de_pos]);
    u_pos   = max(min(u_pos, 30), -30);  
%     u_pos   = 0;
    alpha = 1;
    beta = 1;  % o empieza con 2 y ve subiendo
%     u = Ke * u_theta + beta * u_pos;
%     F = max(min(u, 100), -100);
    % Control PD
    U = Ke * e_theta + Kde * de_theta;
    F = max(min(U, Umax), -Umax);
    
    %% debug valores
    global error_theta_log error_pos_log U_pos_log U_theta_log
    error_theta_log(end+1) = e_theta;
    error_pos_log(end+1) = e_pos;
    U_pos_log(end+1) = u_pos;
    U_theta_log(end+1) = u_theta;
    
    fprintf('e_pos = %.5f\n', e_pos);
    fprintf('e_theta = %.5f\n', e_theta);
    %% Dynamics
    D = I + m * l^2;
    theta_ddot_num = -m*l*cos(theta)*(F + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) - m*g*l*sin(theta) - b2*theta_dot;           
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2)/(M + m));
    % X''
    X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);
    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
    
end
