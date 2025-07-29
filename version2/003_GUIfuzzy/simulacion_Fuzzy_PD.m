function simulacion_Fuzzy_PD()
    % Parameters
    M = 0.5; m = 0.2; l = 0.3; g = -9.81;
    I = (1/3)*m*l^2; b1 = 0.1; b2 = 0.05;

    % Load both fuzzy controllers
    fis_theta = readfis('myfic_utheta.fis');
    fis_x     = readfis('myfic_ux.fis');

    % Initial conditions
    tspan = [0 50];
    y0 = [2, 0, pi-0.1, 0];

    [t, Y] = ode45(@(t,y) dinamica_sistema_fuzzy(t,y,M,m,l,g,I,b1,b2,fis_theta,fis_x), tspan, y0);
    
% for k=1:length(t)
%     drawpendf(Y(k,:),m,M,l);
% end
    % Plot
    figure;
    subplot(2,1,1); plot(t, Y(:,1)); ylabel('X (m)'); grid on;
    subplot(2,1,2); plot(t, (Y(:,3)) * 180/pi); ylabel('\theta (°)'); xlabel('Tiempo (s)'); grid on;
end
