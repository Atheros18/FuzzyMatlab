function dydt = pendcart_dyn_2d(t, y, pars, ctrl)
% y = [x; yW; psi; v; r; theta; thetadot]
x   = y(1); %#ok<NASGU>
yW  = y(2); %#ok<NASGU>
psi = y(3);
v   = y(4);
r   = y(5);
th  = y(6);
thd = y(7);

% --- Control (wheel torques) ---
[tauL, tauR] = ctrl(t,y);

% Map torques -> body force & yaw moment
F  = (tauL + tauR)/pars.R;          % forward (body-x) force
Mz = pars.d*(tauR - tauL)/pars.R;   % yaw moment

% Saturations (safety)
F  = max(min(F,  pars.Fmax),  -pars.Fmax);
Mz = max(min(Mz, pars.Mz_max),-pars.Mz_max);

% --- Cart forward accel using 1D cart-pendulum coupling (X_dot = v) ---
M=pars.M; m=pars.m; l=pars.l; g=pars.g; I=pars.I; b1=pars.b_v; b2=0; % (you can add pendulum joint damping in b2)
D = I + m*l^2 - (m^2*l^2*cos(th)^2)/(M + m);

theta_ddot_num = -m*l*cos(th)*(F + m*l*thd^2*sin(th) - b1*v)/(M + m) ...
                 - m*g*l*sin(th) - b2*thd;
thdd = theta_ddot_num / D;

vdot = (F - m*l*cos(th)*thdd + m*l*thd^2*sin(th) - b1*v)/(M + m);

% --- Yaw dynamics ---
rdot = (Mz - pars.b_r*r) / pars.Iz;

% --- Kinematics to world ---
xdot  = v*cos(psi);
ydot  = v*sin(psi);
psidot= r;

dydt = [xdot; ydot; psidot; vdot; rdot; thd; thdd];
end