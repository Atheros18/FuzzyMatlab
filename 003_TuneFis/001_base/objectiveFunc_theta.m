function J = objectiveFunc_theta(fis_theta)

% disp(">>> START simulation")
tic
    % Constantes físicas
    M = 0.5; m = 0.2; l = 0.3; g = 9.81;
    I = (1/3)*m*l^2; b1 = 0.1; b2 = 0.05;

    % FIS fijo para ángulo
%     fis_theta = readfis('fis_theta.fis');
    fis_pos = readfis('fis_pos.fis');
    % Parámetros optimizados (mGA)
    params(1) = 5.9800;
    params(2) = 1.8462;
    params(3) = 17.9850;
    params(4) = 5.9235;
    params(5) = 0.7504;
    params(6) = 47.2253;

    % Condiciones iniciales y referencias
    y0 = [0 0 pi-0.1 0];  
    tspan = [0 10];
    ref_theta = pi;       
    ref_pos = 0;

%     try
        opts = odeset('RelTol',1e-3,'AbsTol',1e-4,'MaxStep',0.05);
    try
        [~, y] = ode45(@(t,y) pendcart(y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0,opts);
        e_pos = y(:,1) - ref_pos;
        e_theta = y(:,3) - ref_theta;
        J = trapz(t, e_theta.^2); 
    catch
        J = 1e6; % Penalizar soluciones inválidas
    end

    disp("Tiempo de evaluación: " + toc + " s")
end
