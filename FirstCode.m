%define parameters ideals
% constantes
% Initial state: [x, dx, theta, dtheta]
x0 = [0; 0; pi + 0.1; 0];  % Slight push away from upright
% Time span
tspan = [0 10];
% Input force (zero for open-loop simulation)
F = 0;

% Solve using ODE45
[t, X] = ode45(@(t, x) dinamica_pendulo(t, x, F), tspan, x0);

% Plot
figure;
subplot(2,1,1); plot(t, X(:,1)); ylabel('Cart Position x');
subplot(2,1,2); plot(t, X(:,3)); ylabel('Pendulum Angle \theta'); xlabel('Time (s)');
