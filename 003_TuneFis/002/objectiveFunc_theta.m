function J = objectiveFunc_theta(fis_theta)

% disp(">>> START simulation")
tic
    % Constantes físicas
    M = 0.5; m = 0.2; l = 0.3; g = 9.81;
    I = (1/3)*m*l^2; b1 = 0.1; b2 = 0.05;
    
    params(1) = 1.0683;
    params(2) = 0.6854;
    params(3) = 25.3671;
    params(4) = 0.3940;
    params(5) = 2.0989;
    params(6) = 24.9118;
    params(7) = 1;

    % Condiciones iniciales y referencias
    ref_theta = 0; ref_pos = 0;
    tspan = [0 10];
    rho = 5;

%     fis_theta = readfis('fis_theta.fis');
    fis_pos   = readfis("files_created/fis_pos.fis");

    % Lista de casos
    y0_list = [
        0.6 0   0.2  0.1;
        1.0 0   0.3  0.2;
        0.4 0   0.1  0.1
    ];

    total_J = 0;
    opts = odeset('RelTol',1e-3,'AbsTol',1e-4,'MaxStep',0.05);
    try
        for i = 1:size(y0_list,1)
            y0 = y0_list(i,:);
            [t, y] = ode45(@(t,y) pendcart(y, params, M, m, l, g, I, b1, b2, fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0, opts);
            e_pos = ref_pos - y(:,1);
            e_theta = ref_theta - y(:,3);
            J_i = trapz(t, e_theta.^2);
            total_J = total_J + J_i;
        end

        J = total_J / size(y0_list,1);  % promedio
    catch
        J = 1e6; % penalización
    end

    disp("Tiempo de evaluación: " + toc + " s")
end