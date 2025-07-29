function dx = pendcart(x,m,M,L,g,d,fis_theta,fis_pos)
    Sx = sin(x(3));
    Cx = cos(x(3));
    D = m*L*L*(M+m*(1-Cx^2));

    % ----- Fuzzy Controllers -----
    % Angular
    % ----- Fuzzy Controllers -----
    % Angular
    e_theta  = pi-x(3);
    de_theta = -x(4);
    
        % Normalización
        e_theta  = max(min(e_theta/pi, 0.999), -0.999);
        de_theta = max(min(de_theta/10, 0.999), -0.999);

    u_theta = evalfis(fis_theta, [e_theta, de_theta]);
    e_x = 1-x(1);
    e_x_dot = -x(2);
     e_pos  = max(min(e_x/8, 0.999), -0.999);
    de_pos = max(min(e_x_dot/10, 0.999), -0.999);
    %e_x     = X / 3;
    %e_x_dot = X_dot / 5;

    % Apply fuzzy controllers
    u_pos     = evalfis(fis_pos, [e_pos, de_pos]);
    % Positional control is disabled
%     u_pos = 0;

    % Control force
    u = u_theta + u_pos;
%     u = 0;
    %u = max(min(u, 100), -100);
    %u=0;
    fprintf('u_pos = %.2f\n', u_pos);
    fprintf('e_theta = %.2f\n', e_theta);
    % Dinámica
    dx(1,1) = x(2);
    dx(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
    dx(3,1) = x(4);
    dx(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u;
end
