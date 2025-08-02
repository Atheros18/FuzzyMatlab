% Double Pendulum Simulation using ODE45

% Parameters
m1 = 1;       % Mass of the first pendulum (kg)
m2 = 1;       % Mass of the second pendulum (kg)
L1 = 1;       % Length of the first pendulum (m)
L2 = 1;       % Length of the second pendulum (m)
g = 9.81;     % Acceleration due to gravity (m/s^2)

% Time span
tspan = [0 350];  % simulate for 20 seconds

% Initial conditions: [theta1, omega1, theta2, omega2]
y0 = [pi/2; 0; pi/2; 0];

% Solve the system using ode45
[t, Y] = ode45(@(t, y) double_pendulum_ode(t, y, m1, m2, L1, L2, g), tspan, y0);

% Extract solutions
theta1 = Y(:,1);
omega1 = Y(:,2);
theta2 = Y(:,3);
omega2 = Y(:,4);

% Plotting results
figure;
subplot(2,1,1);
plot(t, theta1, 'r', t, theta2, 'b');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Theta1', 'Theta2');
title('Double Pendulum Angles');

subplot(2,1,2);
plot(t, omega1, 'r', t, omega2, 'b');
xlabel('Time (s)');
ylabel('Angular velocity (rad/s)');
legend('Omega1', 'Omega2');
title('Double Pendulum Angular Velocities');

