function F = fuzzy_control_force(y, fis_theta, fis_pos, params, ref_pos, ref_theta,Fmax)
% y = [X; Xdot; theta; thetadot]; ref_x=0, ref_theta=0

    X = y(1);  Xd = y(2);  th = y(3);  thd = y(4);
Ke_pos    = params(1);  Kde_pos    = params(2);  Umax_pos   = params(3);
Ke_th  = params(4);  Kde_th  = params(5);  Umax_th = params(6);
    % Errors
    ex  = ref_pos-X;          dex  = -Xd;
    eth = ref_theta-th;         deth = -thd;

    % Normalize into [-1,1] with stronger leverage (EDIT #3, path B)
    exn   = max(min(Ke_pos * (ex/1.0),  1), -1);
    dexn  = max(min(Kde_pos* (dex/2.0), 1), -1);
    ethn  = max(min(Ke_th  * (eth/pi),  1), -1);   % angle scale by pi
    dethn = max(min(Kde_th * (deth/5.0), 1), -1);

    % Bias removal (EDIT #2)
    persistent u0p u0t
    if isempty(u0p), u0p = evalfis(fis_pos,   [0 0]); end
    if isempty(u0t), u0t = evalfis(fis_theta, [0 0]); end

    u_pos   = evalfis(fis_pos,   [exn,  dexn]);
    u_theta = evalfis(fis_theta, [ethn, dethn]);

    Upos   = Umax_pos   * (u_pos   - u0p);
    Utheta = Umax_th    * (u_theta - u0t);

    % Combine and saturate (keep your actuator limits)
    F = Upos + Utheta;
    F = max(min(F, Fmax), -Fmax);
end
