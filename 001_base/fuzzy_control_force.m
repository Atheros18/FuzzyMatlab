function F = fuzzy_control_force(y, fis_pos, fis_theta, params, ref_pos, ref_theta,Fmax)
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
    
    
    
  

% after up, ut are computed (bias already removed)
    theta_c = 10*pi/180;           % 5°
    speed_c = 60*pi/180;          % 25°/s
    alpha = exp( - (abs(eth)/theta_c).^2 - (abs(deth)/speed_c).^2 );  % 0..1
    
    
      
    
    
    F = (1-alpha)*Utheta + alpha*(Upos); % position fades in as |θ|→0
    F = max(min(F, Fmax), -Fmax);
    
    
    
%     % DEBUG (remove later)
% if X==0.2 && abs(th-0.1)<1e-9 && thd==0 && Xd==0
%     fprintf('t0: up=%.3f ut=%.3f gate=%.2f  Fpos=%+.3f Fth=%+.3f F=%+.3f\n', ...
%             (u_pos   - u0p), (u_theta - u0t), gate, gate*Upos, Utheta, F);
% end
end
