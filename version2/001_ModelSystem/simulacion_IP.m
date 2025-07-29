function simulacion_IP()
    %-----------------------------
    % Parametros generales
    %-----------------------------
    M = 0.5;     % masa del carro(kg)
    m = 0.2;     % masa del pendulo (kg)
    l = 0.3;     % Longitud del pendulo al centro de la masa(m)
    g = -9.81;    % gravedad (m/s^2)
    I = (1/3)*m*l^2;  % momento de inercia del pendulo(rod)
    b1 = 0.1;    % coeficiente de friccion carro
    b2 = 0.05;   % coeficiente de friccion pendulo.
    %simualcion
    tspan = [0 10];  %segundos
    y0 = [0, 0, pi-0.1, 0];  % estado inicial [X, X', theta, theta']

 
    %-----------------------------
    [t, Y] = ode45(@(t,y) dinamica_sistema(t, y, M, m, l, g, I, b1, b2), tspan, y0);

    %-----------------------------
    % Plot results
    %-----------------------------    
    figure;
    subplot(2,1,1);
    plot(t, Y(:,1), 'LineWidth', 1.5);
    ylabel('Posicion del carro X (m)');
    grid on;

    subplot(2,1,2);
    plot(t, Y(:,3)*180/pi, 'LineWidth', 1.5);
    ylabel('Angulo del pendulo \theta ');
    xlabel('Tiempo (s)');
    grid on;
end