function J = mga_fitness(params)
% Parámetros físicos del sistema
    M = 0.5;
    m = 0.2;
    l = 0.3;
    g = 9.81;
    I = (1/3)*m*l^2;
    b1 = 0.1;
    b2 = 0.05;

    % Cargar controladores fuzzy
    fis_theta = readfis('fis_theta.fis');
    fis_pos   = readfis('fis_pos.fis');

    % Condiciones iniciales y tiempo de simulación
    tspan = [0 10];
    y0 = [0, 0, pi - 0.1, 0];  % Estado inicial: [X, X_dot, theta, theta_dot]
    
    ref_theta = pi;
    ref_pos = 0;

    % Inicializar logs de error
    global error_theta_log error_pos_log
    error_theta_log = [];
    error_pos_log = [];
    [~, Y] = ode45(@(t, y) pendcart(y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0);
        rho = 10;  % peso del error angular
        e_theta = ref_theta-Y(:,3);
        e_pos = ref_pos-Y(:,1);
%         J = trapz(e_theta.^2);
    J = trapz(rho*e_theta.^2 + e_pos.^2);

end
