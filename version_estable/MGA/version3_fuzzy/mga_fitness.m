function J = mga_fitness(params)

    persistent fis_theta fis_pos
    if isempty(fis_theta)
        fis_theta = readfis('fis_theta.fis');
        fis_pos   = readfis('fis_pos.fis');
    end
  tic
    k11 = params(1);  % Ke_pos
    k12 = params(2);  % Kde_pos
    k13 = params(3);
    k21 = params(4);  % Ke_theta
    k22 = params(5);  % Kde_theta
    k23 = params(6);
    
    M = 0.5;
    m = 0.2;
    l = 0.3;
    g = 9.81;
    I = (1/3)*m*l^2;
    b1 = 0.1;
    b2 = 0.05;

    % Cargar controladores fuzzy
%     fis_theta = readfis('fis_theta.fis');
%     fis_pos   = readfis('fis_pos.fis');

    % Condiciones iniciales y tiempo de simulación
    tspan = [0 3];
    y0 = [0, 0, pi - 0.1, 0];  % Estado inicial: [X, X_dot, theta, theta_dot]
    
    ref_theta = pi;
    ref_pos = 0;

    % Inicializar logs de error
    global error_theta_log error_pos_log
    error_theta_log = [];
    error_pos_log = [];
    try
    [~, Y] = ode45(@(t, y) pendcart(y, k11,k12,k13,k21,k22,k23, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0);
        rho = 5;  % peso del error angular
        e_theta = ref_theta-Y(:,3);
        e_pos = ref_pos-Y(:,1);
%         J = trapz(e_theta.^2);
    if any(isnan(Y), 'all') || max(abs(Y(:))) > 1000
        J = 1e6;
        return;
    end
    J = trapz(e_theta.^2 + rho*e_pos.^2);
    catch
        disp("Falló con:"); disp(params);
        
        J = 1e6;
    end
 disp(['Tiempo ejecución individuo: ', num2str(toc), ' s']);
end
