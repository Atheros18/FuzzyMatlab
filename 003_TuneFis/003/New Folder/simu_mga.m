clc; clear; close all

% === GA config ===
Nmax = 50;           % try 300–500 for final runs
Npop = 20;
Nvar = 6;             % [k11 k12 k13 k21 k22 k23]
% simu_mga.m
% simu_mga.m
Lc = [0 0 -50,   0 0 -50];   % allow k13,k23 negative if needed
Hc = [5 5  50,   5 5  50];  

seed   = [1.0 0.5 15,   2.0 0.6  -18];   % k23 < 0 (angle scale flipped)

spread = [0.5 0.5  8,  0.5 0.5  8];
initPop = repmat(seed,Npop,1) + (rand(Npop,Nvar)-0.5).*repmat(spread,Npop,1);

% options = optimoptions(@ga, ..., 'InitialPopulationMatrix', initPop, ...);
cr = 0.6; mt = 0.4; sigma_mut = 0.5;

% Parallel pool
if isempty(gcp('nocreate')), parpool('local'); end

global BESTS; BESTS = [];

options = optimoptions(@ga, ...
    'PopulationSize', Npop, 'MaxGenerations', Nmax, 'EliteCount', 2, ...
    'CrossoverFraction', cr, 'UseParallel', true, ...
    'MutationFcn', {@mutationadaptfeasible, mt, sigma_mut}, ...
    'SelectionFcn', @selectiontournament, 'Display','iter', ...
    'InitialPopulationMatrix',initPop,...
    'PlotFcn', {@gaplotbestf}, 'OutputFcn', @guardarFitness);

[bestParams, bestCost] = ga(@mga_fitness, Nvar, [], [], [], [], Lc, Hc, [], options);

fprintf('\nBest params found:\n'); disp(bestParams)
fprintf('Min ISE = %.6f\n', bestCost)

figure; plot(BESTS, 'LineWidth', 2); grid on
xlabel('Generation'); ylabel('Fitness (ISE)'); title('GA progress')