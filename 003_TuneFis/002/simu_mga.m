% clc; clear;
% 
% % === Parámetros mGA (basado en tu tabla) ===
% x = (0:0.1:10)';
% options = genfisOptions("GridPartition");
% options.NumMembershipFunctions = 5;
% fisin = genfis(x,@mga_fitness,options);
% 

clc; clear all; close all;

% === Parámetros mGA (basado en tu tabla) ===
Nmax = 50;                       % Para prueba rápida (antes 1000)
Npop = 20;                       % Igual que en tabla
Nvar = 6;                        % Ahora se optimizan 6 variables
Lc = [0.5, 0.5, 1,    0.5, 0.5, 1];
Hc = [5,   5,   30,   5,   5,   30];
cr   = 0.6;                      % Crossing factor
mt   = 0.4;                      % Mutation factor
sigma_mut = 0.5;                 % Mutación gaussiana estándar

% Guardar valores del mejor fitness en cada generación
global BESTS
BESTS = [];

% === Activar paralelización ===
delete(gcp('nocreate'))
parpool('local')

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
    'UseParallel', false);  % <== activa ejecución paralela

[bestParams, bestCost] = ga(@mga_fitness, Nvar, [], [], [], [], Lc, Hc, [], options);

% === Mostrar resultados ===
fprintf('\n🔍 Mejores parámetros encontrados:\n');
fprintf('params(1) = %.4f\n', bestParams(1));
fprintf('params(2) = %.4f\n', bestParams(2));
fprintf('params(3) = %.4f\n', bestParams(3));
fprintf('params(4) = %.4f\n', bestParams(4));
fprintf('params(5) = %.4f\n', bestParams(5));
fprintf('params(6) = %.4f\n', bestParams(6));
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
