function J = mga_fitness(params)
    %% Start timing (opcional)
    tic
    global BAD_PARAMS
    if isempty(BAD_PARAMS)
        BAD_PARAMS = [];
    end
    %% Constantes físicas del sistema
    M = 0.5; m = 0.2; l = 0.3; g = 9.81;
    I = (1/3)*m*l^2; b1 = 0.1; b2 = 0.05;

    %% Referencias
    ref_theta = 0;        % θ = 0 rad es vertical hacia arriba
    ref_pos = 0;          % posición deseada del carro
    rho = 5;            % peso relativo del error angular
    tspan = [0 10];

    %% FIS (controladores difusos)
    fis_theta = readfis('files_created/fis_theta.fis');
    fis_pos   = readfis('files_created/fis_pos.fis');

    %% Condición inicial (puedes usar más casos si deseas robustez)
    y0 = [0 0 0.3 0];  % [X X_dot θ θ_dot]

    %% Preparar almacenamiento global para las señales de control
    global Utheta_global Upos_global
    Utheta_global = 0;  % valor inicial en caso de falla
    Upos_global   = 0;

    %% Solución del sistema
    opts = odeset('RelTol',1e-3,'AbsTol',1e-4,'MaxStep',0.05);
    try
        Utheta_global = [];
        Upos_global   = [];   
        [t, y] = ode45(@(t,y) pendcart(y, params, M, m, l, g, I, b1, b2, ...
                        fis_theta, fis_pos, ref_theta, ref_pos), tspan, y0, opts);

        %% Errores normalizados
        e_pos   = (ref_pos - y(:,1)) / 1.0;  % asumiendo rango de ±1 m
        e_theta = (ref_theta - y(:,3)) / pi; % normalizado a ±π rad
        
        
% % %         if any(abs(y(:,3)) > pi)
% % %         BAD_PARAMS(end+1,:) = params;
% % %         J = 1e6;
% % %         return;
% % %         end
%         if isempty(Utheta_global) || isempty(Upos_global)
%             J = 1e6;  % penalización: no se registraron controles
%             return
%         end
% 
%         if length(Utheta_global) ~= length(t)
%             Utheta_global = interp1(linspace(tspan(1), tspan(2), length(Utheta_global)), Utheta_global, t, 'linear', 'extrap');
%             Upos_global   = interp1(linspace(tspan(1), tspan(2), length(Upos_global)), Upos_global, t, 'linear', 'extrap');
%         end
%         %% Penalización combinada: errores + esfuerzo de control
%         
%         if ~isempty(BAD_PARAMS)
%             % Tolerancia para no exigir exactitud flotante
%             if any(ismembertol(BAD_PARAMS, params, 'ByRows', true, 'DataScale', 1))
%                 J = 1e6;
%                 return;
%             end
%         end
%         J = trapz(t, e_pos.^2 + rho * e_theta.^2 + 0.01*(Utheta_global.^2 + Upos_global.^2));
            J = trapz(t, e_pos.^2 + rho * e_theta.^2);


    catch
        J = 1e6;  % penalización alta si la simulación falla
    end

    %% Mostrar tiempo de evaluación (opcional)
    disp("Tiempo de evaluación: " + toc + " s")
end
