function dx = pendcart(x,m,M,L,g,d,fis_theta,fis_pos,ref_theta,ref_pos)  
    Sx = sin(x(3));
    Cx = cos(x(3));
    D = m*L*L*(M+m*(1-Cx^2));

    % ----- Fuzzy Controllers -----
    % Angular
    % ----- Fuzzy Controllers -----
    % Angular
    e_theta  = ref_theta-x(3);
    de_theta = -x(4);
    e_theta  = max(min(e_theta/pi, 0.999), -0.999);
    de_theta = max(min(de_theta/5, 0.999), -0.999);
% opt = evalfisOptions('NumSamplePoints', 101, 'OutOfRangeInputValueMessage','none', 'NoRuleFiredMessage','none');
% u_theta = evalfis(fis_theta, [e_theta, de_theta], opt);
    u_theta = evalfis(fis_theta, [e_theta, de_theta]);
    e_pos = ref_pos-x(1);
    de_pos = -x(2);
    e_pos  = max(min(e_pos/3, 0.999), -0.999);
    de_pos = max(min(de_pos/5, 0.999), -0.999);
    u_pos     = evalfis(fis_pos, [e_pos, de_pos]);
    u_pos = max(min(u_pos, 30), -30);  % por ejemplo
%     u_pos = 0;
    alpha = 1;
    beta = 1;  % o empieza con 2 y ve subiendo
    u = alpha * u_theta + beta * u_pos;
    u = max(min(u, 100), -100);
    fprintf('e_pos = %.5f\n', e_pos);
    fprintf('e_theta = %.5f\n', e_theta);
    global error_theta_log error_pos_log U_pos_log U_theta_log
    error_theta_log(end+1) = e_theta;
    error_pos_log(end+1) = e_pos;
    U_pos_log(end+1) = u_pos;
    U_theta_log(end+1) = u_theta;
    
    % Dinámica
    dx(1,1) = x(2);
    dx(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - d*x(2))) + m*L*L*(1/D)*u;
    dx(3,1) = x(4);
    dx(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - d*x(2))) - m*L*Cx*(1/D)*u;
end
