clc; clear all, close all;

% === Parámetros mGA (basado en tu tabla) ===
Nmax = 20;                       % Para prueba rápida (antes 1000)
Npop = 10;                       % Igual que en tabla
Nvar = 6;                         % Ahora se optimizan 6 variables
Lc = [0.1, 0.1,  5,   0.1, 0.1,  5];   % Ke_pos, Kde_pos, Umax_pos, Ke_theta, Kde_theta, Umax_theta
Hc = [5.0, 5.0, 30,   5.0, 5.0, 30];
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
    'MutationFcn', {@mutationadaptfeasible, mt, sigma_mut}, ...
    'SelectionFcn', @selectiontournament, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf}, ...
    'OutputFcn', @guardarFitness, ...
    'UseParallel', true);  % <== activa ejecución paralela

[bestParams, bestCost] = ga(@mga_fitness, Nvar, [], [], [], [], Lc, Hc, [], options);
% === Mostrar resultados ===
fprintf('\n🔍 Mejores parámetros encontrados:\n');
fprintf('Ke_pos    = %.4f\n', bestParams(1));
fprintf('Kde_pos   = %.4f\n', bestParams(2));
fprintf('Umax_pos  = %.4f\n', bestParams(3));
fprintf('Ke_theta  = %.4f\n', bestParams(4));
fprintf('Kde_theta = %.4f\n', bestParams(5));
fprintf('Umax_theta= %.4f\n', bestParams(6));
fprintf('ISE mínimo = %.6f\n', bestCost);

% Guardar resultados
%  save('resultados_mGA.mat', 'bestParams', 'bestCost', 'BESTS');

% Graficar evolución del ISE
figure;
plot(BESTS, 'LineWidth', 2);
xlabel('Generación');
ylabel('ISE (fitness)');
title('Evolución del ISE por generación');
grid on;
