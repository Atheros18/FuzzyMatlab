% MATLAB controller for Webots
% File:          my_controller.m
% Date:
% Description:
% Author:
% Modifications:
function my_controller
TIME_STEP = 10;  dt = TIME_STEP/1000;

% Parámetros de tu control difuso
params = [2.41 0.756 18.9  0.298 1.34 23];   % [Ke_pos Kde_pos Umax_pos Ke_th Kde_th Umax_th]
Fmax   = 15;
x_ref  = 0;  th_ref = 0;

% --- Init Webots
wb_robot_init();

% Sensores (nombres según tu Scene Tree)
hip  = wb_robot_get_device('pendulum_sensor');                 % ángulo del péndulo
wb_position_sensor_enable(hip, TIME_STEP);

cart = wb_robot_get_device('cart_sensor');     % posición del carro
wb_position_sensor_enable(cart, TIME_STEP);

% Motor
motor = wb_robot_get_device('cart_motor');
wb_motor_set_position(motor, inf);                 % modo velocidad

% Un step antes de la primera lectura (evita "invalid device tag")
wb_robot_step(TIME_STEP);

% Cargar tus FIS
fis_theta = readfis('fis_theta.fis');
fis_pos   = readfis('fis_pos.fis');

% Derivadas filtradas
a = 0.2;                 % coef. filtro exponencial
dth_f = 0;  dx_f = 0;
prev_th = wb_position_sensor_get_value(hip);
prev_x  = wb_position_sensor_get_value(cart);

% Usa el maxVelocity real del motor para no excederlo
vmax = wb_motor_get_max_velocity(motor);

while wb_robot_step(TIME_STEP) ~= -1
    % Lecturas
    th  = wb_position_sensor_get_value(hip);                 % rad
    dth = (th - prev_th)/dt;   dth_f = (1-a)*dth_f + a*dth;

    x   = wb_position_sensor_get_value(cart);                % m
    dx  = (x - prev_x)/dt;     dx_f  = (1-a)*dx_f  + a*dx;

    % Fuerza difusa (tu misma función)
    F = fuzzy_control_force([x; dx_f; th; dth_f], ...
        fis_theta, fis_pos, params, x_ref, th_ref, Fmax);

    % Aplicación segura al LinearMotor:
    %  - La dirección la da el signo de la velocidad objetivo
    %  - La magnitud de empuje la limita set_force
    wb_motor_set_force(motor, abs(F));                  % límite de |F| (N)
    wb_motor_set_velocity(motor, sign(F) * vmax);       % no excede maxVelocity

    % memoria
    prev_th = th;  prev_x = x;
end

wb_robot_cleanup();
end
