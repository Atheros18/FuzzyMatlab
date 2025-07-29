function [fis_theta, fis_x] = crear_fis_controladores()
    %% FIS_theta
    fis_theta = mamfis('Name','FIS_theta');
    fis_theta = addInput(fis_theta, [-2*pi 2*pi], 'Name', 'e_theta');
    fis_theta = addMF(fis_theta, 'e_theta', 'trimf', [-2*pi -pi 0], 'Name', 'NB');
    fis_theta = addMF(fis_theta, 'e_theta', 'trimf', [-pi 0 pi], 'Name', 'Z');
    fis_theta = addMF(fis_theta, 'e_theta', 'trimf', [0 pi 2*pi], 'Name', 'PB');
    
    fis_theta = addInput(fis_theta, [-20 20], 'Name', 'de_theta');
    fis_theta = addMF(fis_theta, 'de_theta', 'trimf', [-20 -10 0], 'Name', 'NB');
    fis_theta = addMF(fis_theta, 'de_theta', 'trimf', [-10 0 10], 'Name', 'Z');
    fis_theta = addMF(fis_theta, 'de_theta', 'trimf', [0 10 20], 'Name', 'PB');
    
    fis_theta = addOutput(fis_theta, [-50 50], 'Name', 'F_theta');
    fis_theta = addMF(fis_theta, 'F_theta', 'trimf', [-50 -25 0], 'Name', 'NL');
    fis_theta = addMF(fis_theta, 'F_theta', 'trimf', [-10 0 10], 'Name', 'Z');
    fis_theta = addMF(fis_theta, 'F_theta', 'trimf', [0 25 50], 'Name', 'PL');

    rules_theta = [
        1 1 3 1 1;
        1 2 3 1 1;
        1 3 2 1 1;
        2 1 3 1 1;
        2 2 2 1 1;
        2 3 1 1 1;
        3 1 2 1 1;
        3 2 1 1 1;
        3 3 1 1 1;
    ];
    fis_theta = addRule(fis_theta, rules_theta);

    %% FIS_x
    fis_x = mamfis('Name','FIS_x');
    fis_x = addInput(fis_x, [-5 5], 'Name', 'e_x');
    fis_x = addMF(fis_x, 'e_x', 'trimf', [-5 -2.5 0], 'Name', 'NB');
    fis_x = addMF(fis_x, 'e_x', 'trimf', [-2 0 2], 'Name', 'Z');
    fis_x = addMF(fis_x, 'e_x', 'trimf', [0 2.5 5], 'Name', 'PB');

    fis_x = addInput(fis_x, [-5 5], 'Name', 'de_x');
    fis_x = addMF(fis_x, 'de_x', 'trimf', [-5 -2.5 0], 'Name', 'NB');
    fis_x = addMF(fis_x, 'de_x', 'trimf', [-2 0 2], 'Name', 'Z');
    fis_x = addMF(fis_x, 'de_x', 'trimf', [0 2.5 5], 'Name', 'PB');

    fis_x = addOutput(fis_x, [-5 5], 'Name', 'F_x');
    fis_x = addMF(fis_x, 'F_x', 'trimf', [-5 -2.5 0], 'Name', 'NL');
    fis_x = addMF(fis_x, 'F_x', 'trimf', [-2.5 0 2.5], 'Name', 'Z');
    fis_x = addMF(fis_x, 'F_x', 'trimf', [0 2.5 5], 'Name', 'PL');

    rules_x = [
        1 1 3 1 1;
        1 2 3 1 1;
        1 3 2 1 1;
        2 1 3 1 1;
        2 2 2 1 1;
        2 3 1 1 1;
        3 1 2 1 1;
        3 2 1 1 1;
        3 3 1 1 1;
    ];

    fis_x = addRule(fis_x, rules_x);
end