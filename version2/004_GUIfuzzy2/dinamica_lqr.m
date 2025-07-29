function dydt = dinamica_lqr(~, y, M, m, l, g, I, b1, b2, K)
    % State variables
    X         = y(1);
    X_dot     = y(2);
    theta     = y(3);
    theta_dot = y(4);

    % Full state
    x = [X; X_dot; theta; theta_dot];

    % Desired upright state: [0; 0; pi; 0]
    x_des = [0; 0; 0; 0];

    % Control input
    F = -K * (x - x_des);
     disp([x-x_des]);
    % Dynamics
    D = I + m*l^2;
    theta_ddot_num = -m*l*cos(theta)*(F + m*l*sin(theta)*theta_dot^2 - b1*X_dot)/(M + m) ...
                     - m*g*l*sin(theta) - b2*theta_dot;
    theta_ddot_den = D - (m^2 * l^2 * cos(theta)^2)/(M + m);
    theta_ddot = theta_ddot_num / theta_ddot_den;

    X_ddot = (F - m*l*cos(theta)*theta_ddot + m*l*theta_dot^2*sin(theta) - b1*X_dot) / (M + m);

    % Derivatives
    dydt = [X_dot; X_ddot; theta_dot; theta_ddot];
end
