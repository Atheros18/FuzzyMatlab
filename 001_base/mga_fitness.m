function J = mga_fitness(params0)
% params = [Ke_pos, Kde_pos, Umax_pos, Ke_th, Kde_th, Umax_th]
% θ = 0 is upright. States: [X, Xdot, θ, θdot]

    persistent fis_pos fis_theta
    if isempty(fis_pos),   fis_pos   = readfis('files_created/fis_pos.fis');   end
    if isempty(fis_theta), fis_theta = readfis('files_created/fis_theta.fis'); end

    % --- GA variables unpack ---
%     Ke_pos   = params(1);  Kde_pos  = params(2);  Umax_pos   = params(3);
%     Ke_th    = params(4);  Kde_th   = params(5);  Umax_th    = params(6);

    % --- Plant constants (adjust if needed) ---
    M=0.5; m=0.2; l=0.3; g=9.81; I=(1/3)*m*l^2; b1=0.1; b2=0.05;
    ref_theta = 0;
    ref_pos = 0;
    Fmax = 30;
    
    %%
    % --- Feasibility windows + penalty ---
    Xwin  = 0.05;           % 5 cm
    Vwin  = 0.10;           % 0.10 m/s
    Thwin = 3*pi/180;       % 3 deg
    Wwin  = 5*pi/180;       % 5 deg/s
    HARD  = 1e4;
    %%
    % --- Horizon & weights (EDIT #1) ---
    Tmax = 8.0;
    wX   = 2.0;      % position integral weight
    rho  = 3.0;      % angle integral weight
    wXT  = 2.0;      % terminal position
    wVT  = 0.5;      % terminal velocity
    % --- Initial conditions set (try a small batch) ---
    ICs = [
        0,    0,   0.15, 0;     % small angle
        0.2,  0,   0.10, 0;     % position offset + small angle
        -0.2, 0,  -0.10, 0;
    ];

    J = 0;
    try
        for k = 1:size(ICs,1)
            y0 = ICs(k,:).';
            t0 = 0; tf = Tmax;

            dyn = @(t,y) pendcart_dyn(t,y, M,m,l,g,I,b1,b2, @(yy) fuzzy_control_force(yy, fis_pos, fis_theta, params0, ref_pos, ref_theta,Fmax));

            ev   = @(t,y) stop_events(t,y);  % you already have stop_events.m
            opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-2,'Events',ev);
            [t,Y,te] = ode45(dyn, [t0 tf], y0, opts);

            X  = Y(:,1);   Xd = Y(:,2);   th = Y(:,3);   thd = Y(:,4);
            ex = ref_pos - X;             eth = ref_theta - th;

            L  = wX*ex.^2 + rho*eth.^2;
            Jk = trapz(t, L) + wXT*X(end)^2 + wVT*Xd(end)^2;

            % penalize early exit (hit event)
            if ~isempty(te), Jk = Jk + HARD/2; end

            % hard terminal box
            if abs(X(end))  > Xwin  || abs(Xd(end)) > Vwin || ...
               abs(th(end)) > Thwin || abs(thd(end))> Wwin
                Jk = Jk + HARD;
            end


            if ~isfinite(Jk), J = J + 1e6; else, J = J + Jk; end
        end
        J = J / size(ICs,1);

    catch
        J = 1e7;  % penalize failures
    end
end
