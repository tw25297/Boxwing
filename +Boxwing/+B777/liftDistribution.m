%% B777.LiftDistribution
% Schrenk approximation for spanwise lift distribution
% Covers: Cruise (n=1) and Take-off (high CL, flaps)
%
% Schrenk method: l(y) = 0.5 * [c(y) + c_elliptic(y)] * CL_total
% where c_elliptic(y) = (4S/pi*b) * sqrt(1 - (2y/b)^2)
%
% REF: Schrenk (1940), NACA TM-948
%      Raymer Ch12 for load conditions
%
% OUTPUT:
%   dist struct with spanwise CL, lift/span, bending moment
%
% USAGE:
%   dist = B777.LiftDistribution(ADP)           % TubeWing
%   dist = B777.LiftDistribution(ADP_BW)        % BoxWing

function dist = LiftDistribution(obj)

    % =========================================================
    % GEOMETRY
    % =========================================================
    b     = obj.Span;
    S_ref = obj.WingArea;
    AR    = b^2 / S_ref;

    % spanwise stations -- half span, 200 points
    N  = 200;
    y  = linspace(0, b/2, N);   % [m] from root to tip

    % =========================================================
    % CHORD DISTRIBUTION c(y)
    % =========================================================
    if isa(obj, 'B777.ADP_BW') || isa(obj, 'BoxWing.B777.ADP')
        % BoxWing -- front wing chord distribution (trapezoidal)
        % uses same tr and geometry as boxwingmass.m
        tr    = 0.30;
        alpha = obj.alphaArea;
        Sf    = alpha * S_ref;              % front wing area
        c_rf  = 2*Sf / (b*(1+tr));          % root chord front wing
        c_tf  = tr * c_rf;                  % tip chord front wing

        % linear taper root to tip
        c_y = c_rf + (c_tf - c_rf) .* (y./(b/2));

        % rear wing chord distribution
        Sr   = (1-alpha) * S_ref;
        c_rr = 2*Sr / (b*(1+tr));
        c_tr = tr * c_rr;
        c_y_rear = c_rr + (c_tr - c_rr) .* (y./(b/2));

        config_name = 'BOXWING';
    else
        % TubeWing -- trapezoidal approximation
        % matches wing.m: tr=0.30, kink at KinkPos
        tr   = 0.30;
        c_r  = 2*S_ref / (b*(1+tr));        % root chord
        c_t  = tr * c_r;                    % tip chord
        c_y  = c_r + (c_t - c_r) .* (y./(b/2));

        config_name = 'TUBEWING';
    end

    % =========================================================
    % ELLIPTIC CHORD DISTRIBUTION
    % c_elliptic(y) = (4S/pi*b) * sqrt(1-(2y/b)^2)
    % =========================================================
    c_elliptic = (4*S_ref / (pi*b)) .* sqrt(1 - (2*y/b).^2);

    % =========================================================
    % SCHRENK DISTRIBUTION
    % l(y) = 0.5 * [c(y) + c_elliptic(y)]  (normalised)
    % then scale to match total lift = W
    % =========================================================
    function ld = schrenk(c_chord, CL_total, W_total)
        % Schrenk: average of actual and elliptic chord
        l_raw = 0.5 * (c_chord + c_elliptic);

        % normalise so integral over full span = total lift
        % integral of l(y) dy from -b/2 to b/2 = 2 * integral from 0 to b/2
        L_raw = 2 * trapz(y, l_raw);
        scale = (W_total) / L_raw;
        ld    = l_raw * scale;   % [N/m] lift per unit span
    end

    % =========================================================
    % FLIGHT CONDITIONS
    % =========================================================
    [rho_SL, a_SL, ~,~,~] = BoxWing.cast.atmos(0);
    [rho_cr, a_cr, ~,~,~] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);

    M_cr  = obj.TLAR.M_c;
    M_TO  = 0.25;   % ***ASSUMPTION*** typical TO Mach

    V_cr  = M_cr * a_cr;
    V_TO  = M_TO * a_SL;

    q_cr  = 0.5 * rho_cr * V_cr^2;
    q_TO  = 0.5 * rho_SL * V_TO^2;

    % weights
    W_cruise = obj.MTOM * 9.81 * obj.Mf_TOC;   % TOC weight
    W_TO     = obj.MTOM * 9.81;                 % MTOM at take-off

    % CL per phase
    CL_cruise = W_cruise / (q_cr * S_ref);
    CL_TO     = obj.CL_max + obj.Delta_Cl_to;   % max CL with flaps

    % =========================================================
    % COMPUTE DISTRIBUTIONS
    % =========================================================
    % --- Cruise ---
    l_cruise    = schrenk(c_y, CL_cruise, W_cruise);
    cl_cruise   = l_cruise ./ (q_cr .* c_y);    % local cl(y)

    % --- Take-off ---
    % at TO, CL is fixed at CL_max_TO, lift = Weight
    l_TO        = schrenk(c_y, CL_TO, W_TO);
    cl_TO       = l_TO ./ (q_TO .* c_y);        % local cl(y)

    % --- Elliptic reference (cruise weight) ---
    l_elliptic  = schrenk(c_elliptic, CL_cruise, W_cruise);

    % --- BoxWing: rear wing distribution ---
    if isa(obj, 'B777.ADP_BW') || isa(obj, 'BoxWing.B777.ADP')
        eta       = obj.etaLift;               % lift split
        W_front_cr = eta     * W_cruise;
        W_rear_cr  = (1-eta) * W_cruise;
        W_front_TO = eta     * W_TO;
        W_rear_TO  = (1-eta) * W_TO;

        l_front_cr  = schrenk(c_y,      CL_cruise, W_front_cr);
        l_rear_cr   = schrenk(c_y_rear, CL_cruise, W_rear_cr);
        l_front_TO  = schrenk(c_y,      CL_TO,     W_front_TO);
        l_rear_TO   = schrenk(c_y_rear, CL_TO,     W_rear_TO);

        % combined total
        l_cruise_total = l_front_cr + l_rear_cr;
        l_TO_total     = l_front_TO + l_rear_TO;
    else
        l_cruise_total = l_cruise;
        l_TO_total     = l_TO;
    end

    

    % =========================================================
    % BENDING MOMENT (root bending moment from lift distribution)
    % M_root = integral from 0 to b/2 of l(y)*y dy
    % =========================================================
    BM_cruise = trapz(y, l_cruise_total .* y);   % [N·m]
    BM_TO     = trapz(y, l_TO_total     .* y);   % [N·m]

    % Spanwise centroid of lift (where resultant acts)
    y_centroid_cr = BM_cruise / (W_cruise / 2);   % [m] from root
    y_centroid_TO = BM_TO     / (W_TO    / 2);
    
    % =========================================================
    % PRINT SUMMARY
    % =========================================================
    fprintf('\n==============================================\n');
    fprintf('  LIFT DISTRIBUTION -- %s\n', config_name);
    fprintf('==============================================\n');
    fprintf('  Span          = %.1f m\n',    b);
    fprintf('  S_ref         = %.1f m^2\n',  S_ref);
    fprintf('  AR            = %.2f\n',       AR);
    fprintf('----------------------------------------------\n');
    fprintf('  CRUISE\n');
    fprintf('  W_cruise      = %.0f N\n',    W_cruise);
    fprintf('  CL_cruise     = %.3f\n',       CL_cruise);
    fprintf('  Root BM       = %.3e N·m\n',  BM_cruise);
    fprintf('  Lift centroid (cruise): y = %.2f m (%.1f%% semi-span)\n', y_centroid_cr, y_centroid_cr/(b/2)*100);
    fprintf('----------------------------------------------\n');
    fprintf('  TAKE-OFF\n');
    fprintf('  W_TO          = %.0f N\n',    W_TO);
    fprintf('  CL_TO         = %.3f\n',       CL_TO);
    fprintf('  Root BM       = %.3e N·m\n',  BM_TO);
    fprintf('  Lift centroid (TO):     y = %.2f m (%.1f%% semi-span)\n', y_centroid_TO, y_centroid_TO/(b/2)*100);
    fprintf('==============================================\n\n');


    % =========================================================
    % PLOT
    % =========================================================
    if isa(obj, 'B777.ADP_BW') || isa(obj, 'BoxWing.B777.ADP')
        fig_num = 401;
    else
        fig_num = 400;
    end

    fh = figure(fig_num);
    set(fh, 'WindowStyle', 'docked');
    set(fh, 'Name', sprintf('Lift Distribution -- %s', config_name));
    clf;

    % full span (mirror)
    y_full       = [-fliplr(y), y(2:end)];
    l_cr_full    = [fliplr(l_cruise_total), l_cruise_total(2:end)];
    l_TO_full    = [fliplr(l_TO_total),     l_TO_total(2:end)];
    l_ell_full   = [fliplr(l_elliptic),     l_elliptic(2:end)];

    % --- Plot 1: lift per unit span l(y) ---
    subplot(1,2,1); hold on; grid on; box on;
    plot(y_full, l_cr_full/1000,  'b-',  'LineWidth', 2, 'DisplayName', 'Cruise');
    plot(y_full, l_TO_full/1000,  'r--', 'LineWidth', 2, 'DisplayName', 'Take-off');
    plot(y_full, l_ell_full/1000, 'k:',  'LineWidth', 1.5, 'DisplayName', 'Elliptic (ref)');

    if isa(obj, 'B777.ADP_BW') || isa(obj, 'BoxWing.B777.ADP')
        l_fr_full = [fliplr(l_front_cr), l_front_cr(2:end)];
        l_rr_full = [fliplr(l_rear_cr),  l_rear_cr(2:end)];
        plot(y_full, l_fr_full/1000, 'b-.', 'LineWidth', 1.2, 'DisplayName', 'Front wing (cruise)');
        plot(y_full, l_rr_full/1000, 'g-.', 'LineWidth', 1.2, 'DisplayName', 'Rear wing (cruise)');
    end

    xlabel('Spanwise position y  [m]');
    ylabel('Lift per unit span  l(y)  [kN/m]');
    title(sprintf('Schrenk Lift Distribution  –  %s', config_name));
    legend('Location', 'north', 'FontSize', 8);
    xline(0, 'k--', 'Alpha', 0.3, 'HandleVisibility', 'off');

    % --- Plot 2: local cl(y) ---
    subplot(1,2,2); hold on; grid on; box on;

    cl_cr_full = [fliplr(cl_cruise), cl_cruise(2:end)];
    cl_TO_full = [fliplr(cl_TO),     cl_TO(2:end)];

    % elliptic cl reference
    cl_ell = l_elliptic ./ (q_cr .* c_elliptic);
    cl_ell(end) = 0;   % tip goes to zero
    cl_ell_full = [fliplr(cl_ell), cl_ell(2:end)];

    plot(y_full, cl_cr_full, 'b-',  'LineWidth', 2, 'DisplayName', 'Cruise');
    plot(y_full, cl_TO_full, 'r--', 'LineWidth', 2, 'DisplayName', 'Take-off');
    plot(y_full, cl_ell_full,'k:',  'LineWidth', 1.5, 'DisplayName', 'Elliptic (ref)');
    yline(obj.CL_max, 'k-.',  'LineWidth', 1, 'DisplayName', sprintf('CL_{max} clean = %.2f', obj.CL_max));
    yline(obj.CL_max + obj.Delta_Cl_to, 'm-.', 'LineWidth', 1, ...
          'DisplayName', sprintf('CL_{max} TO = %.2f', obj.CL_max + obj.Delta_Cl_to));

    xlabel('Spanwise position y  [m]');
    ylabel('Local lift coefficient  c_l(y)  [-]');
    title(sprintf('Local c_l Distribution  –  %s', config_name));
    legend('Location', 'north', 'FontSize', 8);
    xline(0, 'k--', 'Alpha', 0.3, 'HandleVisibility', 'off');
    ylim([0 3.5]);

    sgtitle(sprintf('%s  –  Schrenk Lift Distribution', config_name), ...
            'FontSize', 12, 'FontWeight', 'bold');

    % =========================================================
    % OUTPUT STRUCT
    % =========================================================
    dist.y            = y;
    dist.y_full       = y_full;
    dist.l_cruise     = l_cruise_total;
    dist.l_TO         = l_TO_total;
    dist.l_elliptic   = l_elliptic;
    dist.cl_cruise    = cl_cruise;
    dist.cl_TO        = cl_TO;
    dist.CL_cruise    = CL_cruise;
    dist.CL_TO        = CL_TO;
    dist.BM_cruise    = BM_cruise;
    dist.BM_TO        = BM_TO;
    dist.config       = config_name;
    dist.y_centroid_cr = y_centroid_cr;
    dist.y_centroid_TO = y_centroid_TO;

    if isa(obj, 'B777.ADP_BW') || isa(obj, 'BoxWing.B777.ADP')
        dist.l_front_cr = l_front_cr;
        dist.l_rear_cr  = l_rear_cr;
        dist.l_front_TO = l_front_TO;
        dist.l_rear_TO  = l_rear_TO;
    end

end