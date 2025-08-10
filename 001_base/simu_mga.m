clc; clear; rng('shuffle');

% --- Bounds (EDIT #3, PATH B assumed) ---
lb = [0,   0,   0.5,   0,   0,   10];
ub = [3.5, 5.0, 20,    pi,  8.0, 40];

seed = [1.2, 1.0, 6,   0.6, 2.0, 25];  % nonzero Umax_pos


opts = optimoptions('ga', ...
    'PopulationSize',30, ...
    'MaxGenerations',100, ...
    'EliteCount',2, ...
    'CrossoverFraction',0.65, ...
    'UseParallel',true, ...
    'Display','iter', ...
    'MutationFcn','mutationadaptfeasible', ...
    'CreationFcn','gacreationlinearfeasible', ...
    'CrossoverFcn', 'crossoverscattered', ...
    'ConstraintTolerance', 1e-6, ...
    'PlotFcn',{@gaplotbestf,@gaplotstopping}, ...
    'UseParallel', true);

% Re-seed (EDIT #4)
initPop = lb + (ub-lb).*rand(29,6);
initPop = [seed; initPop];
opts = optimoptions(opts, 'InitialPopulationMatrix', initPop);
[bestParams,bestJ] = ga(@mga_fitness, 6, [], [], [], [], lb, ub, [], opts);

paramNames = {'Ke_pos','Kde_pos','Umax_pos','Ke_theta','Kde_theta','Umax_theta'};
assert(numel(bestParams) == numel(paramNames), 'Param count mismatch.');

namesStr  = strjoin(paramNames, ' ');
valuesStr = sprintf('%.3g ', bestParams(:).');  % full precision, force row

% === Show results ===
disp('--- Best parameters found ---');
disp('parameters :]');
disp('[Ke_pos Kde_pos Umax_pos Ke_theta Kde_theta Umax_theta]');
disp(['[' namesStr '] = [' strtrim(valuesStr) '];']);  % copy-pasteable parallel assignment
fprintf('Best Fitness J = %.3g\n', bestJ);

