function dx = dinamica_pendulo(t, x, K)
    % Physical constants
    M = 0.5;
    m = 0.2;
    l = 0.3;
    g = 9.81;

    % State variables
    theta = x(3);
    dtheta = x(4);
    c = cos(theta);
    s = sin(theta);

    % Control input from LQR
    F = -K * x;

    % Equations of motion
    denom = M + m - m * c^2;

    dx = zeros(4,1);
    dx(1) = x(2);
    dx(3) = x(4);

    dx(2) = (F + m * l * dtheta^2 * s - m * g * s * c) / denom;
    dx(4) = (-F * c - m * l * dtheta^2 * s * c + (M + m) * g * s) / (l * denom);
end