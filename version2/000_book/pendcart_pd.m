function dx = pendcart_pd(x,m,M,L,g,d,Kp,Kd)
    Sx = sin(x(3));
    Cx = cos(x(3));
    D = m*L^2*(M + m*(1 - Cx^2));

    % Controlador PD angular
    e_theta = pi - x(3);
    de_theta = -x(4);
    u = Kp*e_theta + Kd*de_theta;
    u = max(min(u, 100), -100);  % saturación

    dx = zeros(4,1);
    dx(1) = x(2);
    dx(2) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L^2*(1/D)*u;
    dx(3) = x(4);
    dx(4) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u;
end
