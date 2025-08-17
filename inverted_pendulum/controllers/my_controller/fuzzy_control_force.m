function F = fuzzy_control_force(y, fis_theta, fis_pos, params, x_ref, th_ref, Fmax)
% params = [Ke_pos, Kde_pos, Umax_pos, Ke_theta, Kde_theta, Umax_theta]
X = y(1);  Xd = y(2);  th = y(3);  thd = y(4);

Ke_pos    = params(1);  Kde_pos    = params(2);  Umax_pos   = params(3);
Ke_theta  = params(4);  Kde_theta  = params(5);  Umax_theta = params(6);

% References
% ref_x = 0; ref_th = 0;  % upright, at origin

% Errors
ex   = x_ref  - X;
dex  = -Xd;
eth  = th_ref - th;    % θ=0 is upright
deth = -thd;

% Normalize to [-1,1] then scale with Ke/Kde (mGA will adapt)
exn   = max(min(Ke_pos   * ex/3,   1), -1);
dexn  = max(min(Kde_pos  * dex/5,  1), -1);
ethn  = max(min(Ke_theta * eth/pi, 1), -1);
dethn = max(min(Kde_theta*deth/8,  1), -1);

% FIS outputs (assume both FIS return roughly in [-1,1])
% Upos   = Umax_pos   * evalfis(fis_pos,   [exn,  dexn]);
% Position force
Upos_raw = evalfis(fis_pos, [exn, dexn]);
persistent U0_pos; if isempty(U0_pos), U0_pos = evalfis(fis_pos, [0 0]); end
Upos     = Umax_pos * (Upos_raw - U0_pos);

% Angle force (you already did this)
Utheta_raw = evalfis(fis_theta, [ethn, dethn]);
persistent U0_th;  if isempty(U0_th),  U0_th  = evalfis(fis_theta, [0 0]); end
Utheta  = Umax_theta * (Utheta_raw - U0_th);


% Utheta = Umax_theta * evalfis(fis_theta, [ethn, dethn]);
% 
F = Upos + Utheta;
% F = 0;
F = max(min(F, Fmax), -Fmax);
end
