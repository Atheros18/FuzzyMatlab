clc; clear;

% === Parámetros mGA (basado en tu tabla) ===
x = (0:0.1:10)';
options = genfisOptions("GridPartition");
options.NumMembershipFunctions = 5;
fisin = genfis(x,@mga_fitness,options);
