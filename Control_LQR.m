% Parameters
M = 0.5;    % cart mass
m = 0.2;    % pendulum mass
l = 0.3;    % pendulum length (to center of mass)
g = 9.81;   % gravity

% Precompute terms
den = M + m;
A = [ 0      1       0           0;
      0      0   -(m*g)/den      0;
      0      0       0           1;
      0      0  (g*(M + m))/(l*den)   0 ];

B = [ 0;
      1/den;
      0;
     -1/(l*den)];
 Q = diag([10, 1, 100, 1]);  % Penalize position and especially angle
R = 0.01;                   % Penalize control effort (force)
K = lqr(A, B, Q, R);

% Closed-loop dynamics
Acl = A - B * K;

% Initial condition: small deviation
x0 = [0.05; 0; 0.1; 0];  % Slightly displaced

% Time
tspan = 0:0.01:10;

% Simulate
[~, X] = ode45(@(t, x) Acl * x, tspan, x0);

% Plot
figure;
subplot(2,1,1); plot(tspan, X(:,1)); ylabel('Cart Position x');
subplot(2,1,2); plot(tspan, X(:,3)); ylabel('Pendulum Angle \theta'); xlabel('Time (s)');


