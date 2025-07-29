function dydt = dinamica_sistema_fuzzy(~, y, M, m, l, g, I, b1, b2, fis_theta, fis_x)

    X     = y(1);
    X_dot = y(2);
    theta = y(3);
    theta_dot = y(4);

e_theta = -(theta);
e_x = X
    %e_x     = X / 3;
    %e_x_dot = X_dot / 5;

    % Apply fuzzy controllers
    U_theta = evalfis(fis_theta, [e_theta, e_theta_dot]);
    U_x     = evalfis(fis_x, [e_x, e_x_dot]);
    
    F = U_theta + U_x;
    F = max(min(F, 100), -100);  % Limita   la fuerza total entre [-100, 100]

    % Dynamics
    D = I + m * l^2;
    theta_ddot_num = -m*l*cos(theta)*(F + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) - m*g*l*sin(theta) - b2*theta_dot;           
    theta_ddot = theta_ddot_num / (D - (m^2 * l^2 * cos(theta)^2)/(M + m));

    % X''
    X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);


    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
    
    %%% sections debugger
    disp([theta, e_theta, theta_dot, e_theta_dot, F]);
end
