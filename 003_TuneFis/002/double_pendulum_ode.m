function dydt = double_pendulum_ode(~, y, m1, m2, L1, L2, g)
    theta1 = y(1);
    omega1 = y(2);
    theta2 = y(3);
    omega2 = y(4);
    
    delta = theta2 - theta1;
    
    denom1 = (m1 + m2)*L1 - m2*L1*cos(delta)^2;

    domega1_dt = (m2*L1*omega1^2*sin(delta)*cos(delta) + ...
                  m2*g*sin(theta2)*cos(delta) + ...
                  m2*L2*omega2^2*sin(delta) - ...
                  (m1 + m2)*g*sin(theta1)) / denom1;

    denom2 = (L2/L1)*denom1;

    domega2_dt = (-m2*L2*omega2^2*sin(delta)*cos(delta) + ...
                  (m1 + m2)*(g*sin(theta1)*cos(delta) - L1*omega1^2*sin(delta) - g*sin(theta2))) / denom2;
    
    dydt = zeros(4,1);
    dydt(1) = omega1;
    dydt(2) = domega1_dt;
    dydt(3) = omega2;
    dydt(4) = domega2_dt;
end
