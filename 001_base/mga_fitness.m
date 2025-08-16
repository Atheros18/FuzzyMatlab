function J = mga_fitness(params)
% params = [Ke_pos, Kde_pos, Umax_pos, Ke_th, Kde_th, Umax_th]
% θ = 0 is upright. States: [X, Xdot, θ, θdot]
    %disp('Iniciando')
    tic
    % ---- Plant constants (keep consistent with your repo) ----
    M=0.5; m=0.2; l=0.3; g=9.81; I=(1/3)*m*l^2; b1=0.1; b2=0.05;
    Fmax = 60;                 % actuator saturation used in control & penalty

    % ---- References ----
    ref_pos   = 0;
    ref_theta = 0;

    % ---- Load FIS once ----
    persistent fis_pos fis_theta
    if isempty(fis_pos),   fis_pos   = readfis('files_created/fis_pos.fis');   end
    if isempty(fis_theta), fis_theta = readfis('files_created/fis_theta.fis'); end

    % ---- Robustness IC set ----
    ICs = [
        0.20   0.5     0.10   0
       -0.20   -0.5    -0.10   0
    ];

    % ---- Horizons & weights ----
    Tshort = [0 1.5];           % screening horizon
    Tlong  = [0 5.0];           % full horizon
    short_thresh = 5;           % if J_short < thisd -> run long pass

    wX   = 2.0;                 % ∫ wX*ex^2
    rho  = 3.0;                 % ∫ rho*eth^2
    wXT  = 2.0;                 % terminal |X|^2
    wVT  = 0.5;                 % terminal |Xdot|^2
    lambdaF = 1e-3;             % ∫ λ (F/Fmax)^2

    % ---- ODE opts with early-stop (use your stop_events.m) ----
    opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',1e-3,'Events',@stop_events);

    % ---- Controller (wrap your fuzzy_control_force with current params) ----
    ctrl = @(y) fuzzy_control_force(y, params, fis_pos, fis_theta, ...
                                    ref_pos, ref_theta, Fmax);

    % ================== 1) QUICK PASS ==================
    try
        J_short = 0;
        for k = 1:size(ICs,1)
            y0  = ICs(k,:).';
            dyn = @(t,y) pendcart_dyn(t, y, M,m,l,g,I,b1,b2, ctrl(y));
            [t, Y, te] = ode45(dyn, Tshort, y0, opts);

            ex  = -Y(:,1);          % ref_x = 0
            eth = -Y(:,3);          % ref_theta = 0
            L   = wX*ex.^2 + rho*eth.^2;
            Jk  = trapz(t, L);

            % mild penalty if it tripped an event early
            if ~isempty(te) || t(end) < Tshort(2)-1e-9, Jk = Jk + 100; end

            J_short = J_short + (isfinite(Jk) * Jk + ~isfinite(Jk)*1e6);
        end
    catch
        J = 1e7;   % numerical failure
        return
    end

    % ================== 2) LONG PASS (only if promising) ==================
    if J_short < short_thresh
        try
            J_long = 0;
            for k = 1:size(ICs,1)
                y0  = ICs(k,:).';
                dyn = @(t,y) pendcart_dyn(t, y, M,m,l,g,I,b1,b2, ctrl(y));
                [t, Y, te] = ode45(dyn, Tlong, y0, opts);

                X  = Y(:,1);  Xd = Y(:,2);  th = Y(:,3);
                ex = -X;      eth = -th;

                L   = wX*ex.^2 + rho*eth.^2;
                Jk  = trapz(t, L) + wXT*X(end)^2 + wVT*Xd(end)^2;

                % control-effort penalty: rebuild F on solver grid
                Fv = zeros(numel(t),1);
                for i = 1:numel(t)
                    Fv(i) = ctrl(Y(i,:).');
                end
                Jk = Jk + lambdaF * trapz(t, (Fv/Fmax).^2);

                if ~isempty(te) || t(end) < Tlong(2)-1e-9, Jk = Jk + 200; end
                if ~isfinite(Jk), Jk = 1e6; end

                J_long = J_long + Jk;
            end
            J = 0.3*J_short + 0.7*J_long;
        catch
            J = J_short + 1e5;
        end
    else
        J = J_short + 1e3;   % not promising → don’t spend time
    end
    disp("Tiempo de evaluación: " + toc + " s")
end
