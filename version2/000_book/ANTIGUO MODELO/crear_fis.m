fis = mamfis('Name', 'fis_theta_pi');
% Entrada: error angular e_theta ∈ [-pi, pi] → normalizado [-1, 1]
fis = addInput(fis, [-1 1], 'Name', 'e_theta');
fis = addMF(fis, 'e_theta', 'trimf', [-1 -1 0], 'Name', 'NL'); % Negativo grande
fis = addMF(fis, 'e_theta', 'trimf', [-1 0 1],  'Name', 'Z');  % Cero
fis = addMF(fis, 'e_theta', 'trimf', [0 1 1],  'Name', 'PL'); % Positivo grande

% Entrada: derivada de theta (de_theta ∈ [-10, 10]) → [-1, 1]
fis = addInput(fis, [-1 1], 'Name', 'de_theta');
fis = addMF(fis, 'de_theta', 'trimf', [-1 -1 0], 'Name', 'NL');
fis = addMF(fis, 'de_theta', 'trimf', [-1 0 1],  'Name', 'Z');
fis = addMF(fis, 'de_theta', 'trimf', [0 1 1],  'Name', 'PL');
fis = addOutput(fis, [-1 1], 'Name', 'u_theta');
fis = addMF(fis, 'u_theta', 'trimf', [-1 -1 0], 'Name', 'NL');
fis = addMF(fis, 'u_theta', 'trimf', [-1 0 1],  'Name', 'Z');
fis = addMF(fis, 'u_theta', 'trimf', [0 1 1],   'Name', 'PL');

rules = [...
    1 1 3 1 1;  % e_theta NL, d_theta NL → PL (corregir hacia atrás)
    1 2 3 1 1;
    1 3 2 1 1;
    2 1 3 1 1;
    2 2 2 1 1;
    2 3 1 1 1;
    3 1 2 1 1;
    3 2 1 1 1;
    3 3 1 1 1;
];

fis = addRule(fis, rules);

% Guardar
writeFIS(fis, 'fis_theta_pi.fis');