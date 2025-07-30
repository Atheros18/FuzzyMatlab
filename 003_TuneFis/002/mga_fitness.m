function J = mga_fitness(params)
    tic    
% Constantes físicas
    M = 0.5; m = 0.2; l = 0.3; g = 9.81;
    I = (1/3)*m*l^2; b1 = 0.1; b2 = 0.05;

    ref_theta = pi; 
    ref_pos = 0;
    y0 = [0.1 0 pi-0.1 0];
    tspan = [0 10];

    fis_theta = readfis('fis_theta.fis');
    fis_pos   = readfis('fis_pos.fis');

        opts = odeset('RelTol',1e-3,'AbsTol',1e-4,'MaxStep',0.05);
    try
        [~, y] = ode45(@(t,y) pendcart(y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0,opts);
        e_x = y(:,1) - ref_pos;
        e_theta = y(:,3) - ref_theta;
        J = trapz(e_x.^2 + 5*e_theta.^2);
    catch
        J = 1e6; % Penalizar soluciones inválidas
    end
    disp("Tiempo de evaluación: " + toc + " s")
end
