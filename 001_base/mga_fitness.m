function J = mga_fitness(params)
% ===== Physical constants (keep consistent) =====
M=0.5; m=0.2; l=0.3; g=9.81; I=(1/3)*m*l^2; b1=0.1; b2=0.05;
Fmax = 30;
% lb = [0 0  0   0 0  0];
% ub = [2 2  8   3 3 40];
% params = max(min(params, ub), lb);
ref_pos = 0 ;
ref_theta = 0;
% ===== Load your FIS =====
persistent fis_theta fis_pos
if isempty(fis_theta)
    fis_theta = readfis('files_created/fis_theta.fis');   % Mamdani 2-in/1-out [-1..1]
    fis_pos   = readfis('files_created/fis_pos.fis');
end

% ===== Simulation config =====
Tmax = 3.0;                        % seconds
rho  = 4.0;                        % angle weight
opts = odeset('RelTol',1e-6,'AbsTol',1e-8, ...
              'Events',@(t,y) stop_events(t,y), ...
              'MaxStep', 1e-2);

% Multiple ICs improves robustness (small grid)
ICs = [
    0.15  0     0.10  0;    % mild offsets
    0     0     0.15  0;    % angle only
    0.25  0     0.00  0;    % position only
    -0.2  0    -0.10  0;    % opposite
];

J = 0;
try
    for k=1:size(ICs,1)
        y0 = ICs(k,:).';
        t0 = 0; tf = Tmax;

        % Sim loop with control-in-the-loop
        dyn = @(t,y) pendcart_dyn(t, y, M,m,l,g,I,b1,b2, fuzzy_control_force(y,params, fis_pos, fis_theta, ref_pos, ref_theta,Fmax));

        [t,Y,te,ye,ie] = ode45(dyn, [t0 tf], y0, opts);

        X=Y(:,1); th=Y(:,3);
        ex = -X; eth = -th;

        % Trapezoidal integral of cost
        L = ex.^2 + rho*eth.^2;
        Jk = trapz(t, L);

        % Penalties
        hit_bounds = ~isempty(te);
        if hit_bounds
            Jk = Jk + 1e4 + 1e4*length(te);  % big penalty
        end

        % Smooth penalties for large control activity (optional)
        % (You can log F inside dyn via nested function if you want.)

        J = J + Jk;
    end

catch
    % If solver fails or NaN appears, punish hard
    J = 1e8;
end

% Small regularization to discourage extreme params
if any(~isfinite(params)) || any(abs(params)>100)
    J = J + 1e6;
end
end
