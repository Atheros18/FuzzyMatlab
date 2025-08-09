function J = mga_fitness(params)
% Objective: robust ISE with soft penalties + debug
    %% Physical constants
    M = 0.5; m = 0.2; l = 0.3; g = 9.81; I = (1/3)*m*l^2; b1 = 0.1; b2 = 0.05;

    %% References
    ref_theta = 0; ref_pos = 0; rho = 5;

    %% Load FIS (persist for speed)
    persistent fis_theta fis_pos
    if isempty(fis_theta)
        fis_theta = readfis('files_created/fis_theta.fis');
        fis_pos   = readfis('files_created/fis_pos.fis');
    end

    %% Scenarios (start simple; luego agrega más)
    y0_list = [0 0 0.2 0];    % near upright
    T = 6; tspan = [0 T];
    opts = odeset('RelTol',1e-3,'AbsTol',1e-4,'MaxStep',0.02);

    %% Debug flags
    DEBUG = false;  shown = 0;   % pon true si quieres prints

    J = 0;
    for k = 1:size(y0_list,1)
        y0 = y0_list(k,:);
        try
            % reset logs each run
            global error_theta_log error_pos_log U_pos_log U_theta_log F_log
            error_theta_log = []; error_pos_log = []; U_pos_log = []; U_theta_log = []; F_log = [];

            [t, y] = ode45(@(t,y) pendcart(t,y, params, M, m, l, g, I, b1, b2, ...
                                 fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0, opts);

            % Errors
            e_pos   = (ref_pos - y(:,1));
            e_theta = (ref_theta - y(:,3));

            % ---- Soft penalties instead of hard abort ----
            t_grace = 0.3;                           % 300 ms sin castigo
            mask = (t > t_grace);

            % over-angle/rail amounts
            ang_over  = max(0, abs(y(:,3)) - 0.9*pi);
            rail_over = max(0, abs(y(:,1)) - 1.5);

            % penalización integrada (ajusta coeficientes si hace falta)
            pen = trapz(t(mask), 1e4*(ang_over(mask).^2)) + ...
                  trapz(t(mask), 1e3*(rail_over(mask).^2));

            if DEBUG && shown < 5 && (pen > 0)
                t_hit = t(find(ang_over>0 | rail_over>0, 1, 'first'));
                fprintf('[PEN] t=%.3f  ang=%g  rail=%g  params=[%.2f %.2f %.1f | %.2f %.2f %.1f]\n',...
                    t_hit, max(ang_over), max(rail_over), params);
                shown = shown + 1;
            end
            % ---------------------------------------------

            % Control effort penalty (muy pequeño)
            if isempty(F_log), ce = 0; else, ce = trapz(t, 1e-3*(F_log.^2)); end

            % Fitness total
            Jk = trapz(t, e_pos.^2 + rho*(e_theta/pi).^2) + ce + pen;
            J  = J + Jk;

        catch
            % solo si ODE falla de verdad
            J = J + 1e6;
        end
    end
end
