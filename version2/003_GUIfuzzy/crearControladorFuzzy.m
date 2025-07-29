function [fis_theta,fis_x] = crearControladorFuzzy()
    % Controlador theta (ángulo en ?)
    fis_theta = newfis('controlador_pi');
    fis_theta = addvar(fis_theta, 'input', 'error', [-2*pi 2*pi]);
    fis_theta = addmf(fis_theta, 'input', 1, 'N', 'trimf', [-2*pi -pi 0]);
    fis_theta = addmf(fis_theta, 'input', 1, 'Z', 'trimf', [-pi 0 pi]);
    fis_theta = addmf(fis_theta, 'input', 1, 'P', 'trimf', [0 pi 2*pi]);

    fis_theta = addvar(fis_theta, 'input', 'derror', [-20 20]);
    fis_theta = addmf(fis_theta, 'input', 2, 'N', 'trimf', [-20 -10 0]);
    fis_theta = addmf(fis_theta, 'input', 2, 'Z', 'trimf', [-10 0 10]);
    fis_theta = addmf(fis_theta, 'input', 2, 'P', 'trimf', [0 10 20]);

    fis_theta = addvar(fis_theta, 'output', 'force', [-50 50]);
    fis_theta = addmf(fis_theta, 'output', 1, 'N', 'trimf', [-50 -25 0]);
    fis_theta = addmf(fis_theta, 'output', 1, 'Z', 'trimf', [-10 0 10]);
    fis_theta = addmf(fis_theta, 'output', 1, 'P', 'trimf', [0 25 50]);

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
    fis_theta = addrule(fis_theta, rules_theta);

    % Controlador X (posición carro)
    fis_x = newfis('controlador_X');
    fis_x = addvar(fis_x, 'input', 'pos_error', [-1 1]);
    fis_x = addmf(fis_x, 'input', 1, 'N', 'trimf', [-1 -1 0]);
    fis_x = addmf(fis_x, 'input', 1, 'Z', 'trimf', [-1 0 1]);
    fis_x = addmf(fis_x, 'input', 1, 'P', 'trimf', [0 1 1]);

    fis_x = addvar(fis_x, 'input', 'vel_error', [-2 2]);
    fis_x = addmf(fis_x, 'input', 2, 'N', 'trimf', [-2 -2 0]);
    fis_x = addmf(fis_x, 'input', 2, 'Z', 'trimf', [-2 0 2]);
    fis_x = addmf(fis_x, 'input', 2, 'P', 'trimf', [0 2 2]);

    fis_x = addvar(fis_x, 'output', 'F_x', [-15 15]);
    fis_x = addmf(fis_x, 'output', 1, 'N', 'trimf', [-15 -7.5 0]);
    fis_x = addmf(fis_x, 'output', 1, 'Z', 'trimf', [-5 0 5]);
    fis_x = addmf(fis_x, 'output', 1, 'P', 'trimf', [0 7.5 15]);

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
    fis_x = addrule(fis_x, rules_x);
end
