function dydt = pendcart_dyn(~, y, M, m, l, g, I, b1, b2, ufun)
    X  = y(1);  Xd = y(2);  th = y(3);  thd = y(4);

    % Call control law
    F = ufun(y);

    % Dynamics (θ = 0 upright)
    D = I + m*l^2 - (m^2*l^2*cos(th)^2)/(M+m);

    thdd_num = - m*l*cos(th) * (F + m*l*sin(th)*thd^2 - b1*Xd)/(M+m) ...
               - m*g*l*sin(th) ...     % <-- fix sign (was +)
               - b2*thd;

    thdd = thdd_num / D;
    Xdd  = (F - m*l*cos(th)*thdd + m*l*thd^2*sin(th) - b1*Xd)/(M+m);

    dydt = [Xd; Xdd; thd; thdd];
end
