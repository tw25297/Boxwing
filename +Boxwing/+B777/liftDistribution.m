%% Boxwing.B777.liftDistribution
% Schrenk approximation for spanwise lift distribution.
% Covers: Cruise (n=1) and Take-off (high CL, flaps).
%
% FIX (2025-01): BoxWing.cast.atmos → Boxwing.cast.atmos (lowercase 'w').
%   The namespace package is +Boxwing (lowercase), so all calls must use
%   Boxwing.cast.atmos, not BoxWing.cast.atmos.
%
% REF: Schrenk (1940), NACA TM-948; Raymer Ch12 for load conditions.

function dist = liftDistribution(obj)

    % =========================================================
    % GEOMETRY
    % =========================================================
    b     = obj.Span;
    S_ref = obj.WingArea;
    AR    = b^2 / S_ref;

    N  = 200;
    y  = linspace(0, b/2, N);

    % =========================================================
    % CHORD DISTRIBUTION c(y)
    % =========================================================
    if isa(obj, 'Boxwing.B777.ADP')
        tr    = 0.30;
        alpha = obj.alphaArea;
        Sf    = alpha * S_ref;
        c_rf  = 2*Sf / (b*(1+tr));
        c_tf  = tr * c_rf;
        c_y   = c_rf + (c_tf - c_rf) .* (y./(b/2));

        Sr      = (1-alpha) * S_ref;
        c_rr    = 2*Sr / (b*(1+tr));
        c_tr    = tr * c_rr;
        c_y_rear = c_rr + (c_tr - c_rr) .* (y./(b/2));

        config_name = 'BOXWING';
    else
        tr   = 0.30;
        c_r  = 2*S_ref / (b*(1+tr));
        c_t  = tr * c_r;
        c_y  = c_r + (c_t - c_r) .* (y./(b/2));
        config_name = 'TUBEWING';
    end

    % =========================================================
    % ELLIPTIC CHORD DISTRIBUTION
    % =========================================================
    c_elliptic = (4*S_ref / (pi*b)) .* sqrt(max(1 - (2*y/b).^2, 0));

    % =========================================================
    % SCHRENK DISTRIBUTION (nested function)
    % =========================================================
    function ld = schrenk(c_chord, W_total)
        l_raw = 0.5 * (c_chord + c_elliptic);
        L_raw = 2 * trapz(y, l_raw);
        if L_raw < 1e-6; ld = zeros(size(y)); return; end
        ld = l_raw * (W_total / L_raw);
    end

    % =========================================================
    % FLIGHT CONDITIONS
    % FIX: use Boxwing.cast.atmos (full path, lowercase 'w')
    % =========================================================
    [rho_SL, a_SL, ~,~,~] = Boxwing.cast.atmos(0);
    [rho_cr, a_cr, ~,~,~] = Boxwing.cast.atmos(obj.TLAR.Alt_cruise);

    M_cr = obj.TLAR.M_c;
    M_TO = 0.25;

    q_cr = 0.5 * rho_cr * (M_cr * a_cr)^2;
    q_TO = 0.5 * rho_SL * (M_TO * a_SL)^2;

    W_cruise = obj.MTOM * 9.81 * obj.Mf_TOC;   % actual cruise weight [N]
    W_TO     = obj.MTOM * 9.81;

    % CL_cruise from wing loading — reliable, avoids stale WingArea issues.
    % WingLoading = MTOM*g/WingArea, so CL = WingLoading * Mf_TOC / q_cr
    if isprop(obj,'WingLoading') && isfinite(obj.WingLoading) && obj.WingLoading > 1000
        CL_cruise = (obj.WingLoading * obj.Mf_TOC) / q_cr;
    else
        CL_cruise = W_cruise / (q_cr * S_ref);
    end
    CL_cruise = min(max(CL_cruise, 0.3), 1.2);   % physical clamp

    CL_TO = obj.CL_max + obj.Delta_Cl_to;

    % =========================================================
    % COMPUTE DISTRIBUTIONS
    % =========================================================
    l_cruise  = schrenk(c_y, W_cruise);
    % Zero out cl where chord < 15% of root chord — tip cl is meaningless
    % (Schrenk l→0 but c→0 faster, so ratio diverges artificially)
    c_root    = c_y(1);
    tip_mask  = c_y < 0.15 * c_root;
    cl_cruise = l_cruise ./ max(q_cr .* c_y, 1e-6);
    cl_cruise(tip_mask) = 0;
    cl_cruise = min(cl_cruise, 3.5);

    l_TO  = schrenk(c_y, W_TO);
    cl_TO = l_TO ./ max(q_TO .* c_y, 1e-6);
    cl_TO(tip_mask) = 0;
    cl_TO = min(cl_TO, 3.5);

    l_elliptic = schrenk(c_elliptic, W_cruise);

    if isa(obj, 'Boxwing.B777.ADP')
        eta          = obj.etaLift;
        l_front_cr   = schrenk(c_y,      eta     * W_cruise);
        l_rear_cr    = schrenk(c_y_rear, (1-eta) * W_cruise);
        l_front_TO   = schrenk(c_y,      eta     * W_TO);
        l_rear_TO    = schrenk(c_y_rear, (1-eta) * W_TO);
        l_cruise_total = l_front_cr + l_rear_cr;
        l_TO_total     = l_front_TO + l_rear_TO;
    else
        l_cruise_total = l_cruise;
        l_TO_total     = l_TO;
    end

    % =========================================================
    % BENDING MOMENT
    % =========================================================
    BM_cruise = trapz(y, l_cruise_total .* y);
    BM_TO     = trapz(y, l_TO_total     .* y);

    y_centroid_cr = BM_cruise / max(W_cruise/2, 1);
    y_centroid_TO = BM_TO     / max(W_TO   /2, 1);

    % =========================================================
    % PRINT SUMMARY
    % =========================================================
    fprintf('\n==============================================\n');
    fprintf('  LIFT DISTRIBUTION -- %s\n', config_name);
    fprintf('==============================================\n');
    fprintf('  Span          = %.1f m\n',   b);
    fprintf('  S_ref         = %.1f m^2\n', S_ref);
    fprintf('  AR            = %.2f\n',      AR);
    fprintf('----------------------------------------------\n');
    fprintf('  CRUISE\n');
    fprintf('  CL_cruise     = %.3f\n',      CL_cruise);
    fprintf('  Root BM       = %.3e N·m\n', BM_cruise);
    fprintf('  Lift centroid : y = %.2f m (%.1f%% semi-span)\n', ...
            y_centroid_cr, y_centroid_cr/(b/2)*100);
    fprintf('----------------------------------------------\n');
    fprintf('  TAKE-OFF\n');
    fprintf('  CL_TO         = %.3f\n',      CL_TO);
    fprintf('  Root BM       = %.3e N·m\n', BM_TO);
    fprintf('  Lift centroid : y = %.2f m (%.1f%% semi-span)\n', ...
            y_centroid_TO, y_centroid_TO/(b/2)*100);
    fprintf('==============================================\n\n');

    % =========================================================
    % PLOT
    % =========================================================
    fh = figure(6);
    clf(fh);
    set(fh, 'Color', 'w', 'Name', sprintf('Lift Distribution -- %s', config_name));

    y_full     = [-fliplr(y),          y(2:end)];
    l_cr_full  = [fliplr(l_cruise_total), l_cruise_total(2:end)];
    l_TO_full  = [fliplr(l_TO_total),     l_TO_total(2:end)];
    l_ell_full = [fliplr(l_elliptic),     l_elliptic(2:end)];

    subplot(1,2,1); hold on; grid on; box on;
    plot(y_full, l_cr_full/1000,  'b-',  'LineWidth', 2, 'DisplayName', 'Cruise');
    plot(y_full, l_TO_full/1000,  'r--', 'LineWidth', 2, 'DisplayName', 'Take-off');
    plot(y_full, l_ell_full/1000, 'k:',  'LineWidth', 1.5, 'DisplayName', 'Elliptic ref');

    if isa(obj, 'Boxwing.B777.ADP')
        plot(y_full, [fliplr(l_front_cr), l_front_cr(2:end)]/1000, ...
             'b-.', 'LineWidth', 1.2, 'DisplayName', 'Front (cruise)');
        plot(y_full, [fliplr(l_rear_cr),  l_rear_cr(2:end)]/1000, ...
             'g-.', 'LineWidth', 1.2, 'DisplayName', 'Rear (cruise)');
    end
    xlabel('Spanwise position y  [m]');
    ylabel('Lift per unit span  [kN/m]');
    title(sprintf('Schrenk Distribution -- %s', config_name));
    legend('Location','north','FontSize',8);

    subplot(1,2,2); hold on; grid on; box on;
    cl_cr_full  = [fliplr(cl_cruise), cl_cruise(2:end)];
    cl_ell      = l_elliptic ./ max(q_cr .* c_elliptic, 1e-6);
    cl_ell(end) = 0;
    cl_ell_full = [fliplr(cl_ell), cl_ell(2:end)];

    % Only plot cruise cl — TO cl is dominated by low q_TO at M=0.25
    % and is not physically informative on this scale.
    plot(y_full, cl_cr_full,  'b-',  'LineWidth', 2.5, 'DisplayName', 'Cruise');
    plot(y_full, cl_ell_full, 'k:',  'LineWidth', 1.5, 'DisplayName', 'Elliptic ref');
    yline(obj.CL_max, 'k-.', 'LineWidth', 1.2, ...
          'DisplayName', sprintf('CL_{max} clean = %.2f', obj.CL_max));
    yline(CL_cruise, 'b:', 'LineWidth', 1.0, ...
          'DisplayName', sprintf('CL cruise = %.3f', CL_cruise));

    xlabel('Spanwise position y  [m]');
    ylabel('Local lift coefficient  c_l(y)  [-]');
    title(sprintf('Local c_l Distribution -- %s  (cruise)', config_name));
    legend('Location','northeast','FontSize',8);
    % Ylim: just above CL_max clean so the stall margin is visible
    ylim([0, obj.CL_max * 1.25]);

    sgtitle(sprintf('%s -- Schrenk Lift Distribution', config_name), ...
            'FontSize', 12, 'FontWeight', 'bold');

    % Save inside the function while the handle is guaranteed valid
    saveas(fh, 'Boxwing_LiftDistribution.png');

    % =========================================================
    % OUTPUT STRUCT
    % =========================================================
    dist.y             = y;
    dist.y_full        = y_full;
    dist.l_cruise      = l_cruise_total;
    dist.l_TO          = l_TO_total;
    dist.l_elliptic    = l_elliptic;
    dist.cl_cruise     = cl_cruise;
    dist.cl_TO         = cl_TO;
    dist.CL_cruise     = CL_cruise;
    dist.CL_TO         = CL_TO;
    dist.BM_cruise     = BM_cruise;
    dist.BM_TO         = BM_TO;
    dist.config        = config_name;
    dist.y_centroid_cr = y_centroid_cr;
    dist.y_centroid_TO = y_centroid_TO;

    if isa(obj, 'Boxwing.B777.ADP')
        dist.l_front_cr = l_front_cr;
        dist.l_rear_cr  = l_rear_cr;
        dist.l_front_TO = l_front_TO;
        dist.l_rear_TO  = l_rear_TO;
    end
end