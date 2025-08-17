function [tauL, tauR, F_cmd, Mz_cmd] = fuzzy_control_drive(t, y, fis_pos, fis_theta, pars, ref_fun)
% y = [x;yW;psi; v; r; theta; thetadot]
x = y(1); psi = y(3); v = y(4); r = y(5); th = y(6); thd = y(7);

ref = ref_fun(t);   % struct('x',..,'psi',..,'th',..)

% --- Errors (θ = 0 upright convention) ---
ex   = ref.x  - x;     dex  = -v;      % longitudinal
eth  = ref.th - th;    deth = -thd;    % angle
epsi = ref.psi- psi;   dr   = -r;      % yaw

% --- Normalize into [-1,1] with physical bounds ---
exn   = max(min(pars.Ke_pos  * ex / pars.EX_MAX,  0.999), -0.999);
dexn  = max(min(pars.Kde_pos * dex/ pars.DX_MAX,  0.999), -0.999);
ethn  = max(min(pars.Ke_th   * eth/ pi,           0.999), -0.999);
dethn = max(min(pars.Kde_th  * deth/pars.DTH_MAX, 0.999), -0.999);

% --- DC-bias cancellation on both FIS outputs ---
Upos_raw   = evalfis(fis_pos,   [exn,  dexn]);
persistent U0_pos; if isempty(U0_pos), U0_pos = evalfis(fis_pos,   [0 0]); end
Utheta_raw = evalfis(fis_theta, [ethn, dethn]);
persistent U0_th;  if isempty(U0_th),  U0_th  = evalfis(fis_theta, [0 0]); end

Upos   = pars.Umax_pos * (Upos_raw   - U0_pos);
Utheta = pars.Umax_th  * (Utheta_raw - U0_th);

F_cmd  = Upos + Utheta;
F_cmd  = max(min(F_cmd, pars.Fmax), -pars.Fmax);

% --- Yaw control (PD for now; swap to fuzzy later if you want) ---
Mz_cmd = pars.Kyaw_e*epsi + pars.Kyaw_d*dr;
Mz_cmd = max(min(Mz_cmd, pars.Mz_max), -pars.Mz_max);

% --- Map to wheel torques ---
tau_sum  = F_cmd * pars.R;
tau_diff = (Mz_cmd * pars.R)/pars.d;

tauL = 0.5*(tau_sum - tau_diff);
tauR = 0.5*(tau_sum + tau_diff);

% torque clamp
tauL = max(min(tauL, pars.tau_max), -pars.tau_max);
tauR = max(min(tauR, pars.tau_max), -pars.tau_max);
end
