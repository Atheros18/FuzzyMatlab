clc; clear;

% === Parámetros mGA (basado en tu tabla) ===
Nmax = 50;                       % Para prueba rápida (antes 1000)
Npop = 20;                       % Igual que en tabla
Nvar = 6;                        % Ahora se optimizan 6 variables
Lc   = zeros(1, Nvar);           % Lower constraints [0 0 0 0 0 0]
Hc   = 100 * ones(1, Nvar);      % Upper constraints [100 100 100 100 100 100]
cr   = 0.6;                      % Crossing factor
mt   = 0.4;                      % Mutation factor
sigma_mut = 0.5;                 % Mutación gaussiana estándar

% Guardar valores del mejor fitness en cada generación
global BESTS
BESTS = [];

% === Activar paralelización ===
if isempty(gcp('nocreate'))
    parpool;  % inicia el pool si no está activo
end

% === Configuración del GA ===
options = optimoptions(@ga, ...
    'PopulationSize', Npop, ...
    'MaxGenerations', Nmax, ...
    'EliteCount', 2, ...
    'CrossoverFraction', cr, ...
    'MutationFcn', {@mutationgaussian, mt, sigma_mut}, ...
    'SelectionFcn', @selectiontournament, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf}, ...
    'OutputFcn', @guardarFitness, ...
    'UseParallel', true);  % <== activa ejecución paralela

[bestParams, bestCost] = ga(@mga_fitness, Nvar, [], [], [], [], Lc, Hc, [], options);

% === Mostrar resultados ===
fprintf('\n🔍 Mejores parámetros encontrados:\n');
fprintf('Ke_theta  = %.4f\n', bestParams(1));
fprintf('Kde_theta = %.4f\n', bestParams(2));
fprintf('Umax_theta= %.4f\n', bestParams(3));
fprintf('Ke_pos    = %.4f\n', bestParams(4));
fprintf('Kde_pos   = %.4f\n', bestParams(5));
fprintf('Umax_pos  = %.4f\n', bestParams(6));
fprintf('ISE mínimo = %.6f\n', bestCost);

% Guardar resultados
% save('resultados_mGA.mat', 'bestParams', 'bestCost', 'BESTS');

% Graficar evolución del ISE
figure;
plot(BESTS, 'LineWidth', 2);
xlabel('Generación');
ylabel('ISE (fitness)');
title('Evolución del ISE por generación');
grid on;
