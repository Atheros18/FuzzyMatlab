clc; close all; clear all;
% === Load base FIS ===
delete(gcp('nocreate'))
parpool('local')
% fis0 = readfis("fis_theta.fis");  % debe ser versión 2.0
fis0 = readfis("fis_pos.fis");  % debe ser versión 2.0
% === Get tunable parameters ===
[in, out, rule] = getTunableSettings(fis0);
paramset = [in; out; rule];  % puedes filtrar aquí si deseas solo input/output

% === Define optimization options ===
options = tunefisOptions("Method", "ga");
options.MethodOptions.PopulationSize = 40;
options.MethodOptions.MaxGenerations = 35;
options.UseParallel = false;  % ✅ Opcional: si tienes Parallel Computing Toolbox

% === Run tuning ===
fprintf("🔁 Starting fuzzy tuning using GA...\n");
[fis_opt, eval_result] = tunefis(fis0, paramset, @objectiveFunc_theta, options);
% [fis_opt, eval_result] = tunefis(fis0, paramset, @objectiveFunc_dummy, options);
% === Save optimized FIS ===
writefis(fis_opt, "fis_pos_optimized.fis");
fprintf("✅ Saved: fis_pos_optimized.fis\n");

