function [TW_SLS, WS_design] = ConstraintAnalysis(obj, doPlot)
%CONSTRAINTANALYSIS  Wing loading and SLS thrust-to-weight for BoxWing.
%
%  Builds all constraint curves, plots the constraint diagram, and picks
%  the design point from the feasible envelope.
%
%  USAGE:
%    [TW, WS] = BoxWing.B777.ConstraintAnalysis(ADP)          % with plot
%    [TW, WS] = BoxWing.B777.ConstraintAnalysis(ADP, false)   % silent
%
%  CONSTRAINTS:
%    TOL   — take-off field length      (T/W vs W/S)
%    ROC   — rate of climb at cruise    (T/W vs W/S)
%    TOCG  — take-off climb gradient OEI(T/W vs W/S)
%    MACS  — cruise drag match          (T/W vs W/S)
%    LFL   — landing field length       (max W/S vertical line)
%    Approach — stall speed approach    (max W/S vertical line)
%    Ceiling  — cruise ceiling          (max W/S vertical line)

if nargin < 2, doPlot = true; end

%% ── Wing-loading sweep ────────────────────────────────────────────────────
WS = linspace(3000, 10000, 300);   % [N/m^2]  realistic range for wide-body

%% ── T/W constraint curves (each returns 1×N) ─────────────────────────────
TW_to   = BoxWing.B777.geom.subconstrains.TOL(obj,  WS);
TW_roc  = BoxWing.B777.geom.subconstrains.ROC(obj,  WS);
TW_tocg = BoxWing.B777.geom.subconstrains.TOCG(obj, WS);
TW_macs = BoxWing.B777.geom.subconstrains.MACS(obj, WS);

% Envelope = maximum T/W requirement at each W/S
TW_env = max([TW_to; TW_roc; TW_tocg; TW_macs], [], 1);

%% ── Vertical (max W/S) constraints ───────────────────────────────────────
WSmax_lfl      = BoxWing.B777.geom.subconstrains.LFL(obj);       % landing field
WSmax_approach = BoxWing.B777.geom.subconstrains.Approach(obj);  % stall speed
WSmax_ceiling  = BoxWing.B777.geom.subconstrains.Ceiling(obj);   % cruise ceiling

% Convert from lb/ft² back to N/m² (subconstraints return lb/ft²)
lbft_to_SI = 1 / 0.020885;   % 1 lb/ft² = 47.88 N/m²  →  1/0.020885
WSmax_lfl_SI      = WSmax_lfl      * lbft_to_SI;
WSmax_approach_SI = WSmax_approach * lbft_to_SI;
WSmax_ceiling_SI  = WSmax_ceiling  * lbft_to_SI;
WSmax_all_SI      = min([WSmax_lfl_SI, WSmax_approach_SI, WSmax_ceiling_SI]);

%% ── Feasible mask ─────────────────────────────────────────────────────────
feasible     = WS <= WSmax_all_SI;
TW_env_feas  = TW_env;
TW_env_feas(~feasible) = NaN;

if ~any(isfinite(TW_env_feas))
    warning('ConstraintAnalysis: feasible region empty — using full envelope.');
    TW_env_feas = TW_env;
    WSmax_all_SI = max(WS);
end

%% ── Design point: 90% of WSmax ───────────────────────────────────────────
WS_design = 0.90 * WSmax_all_SI;
WS_design = max(min(WS_design, max(WS)), min(WS));

mask = isfinite(TW_env_feas);
if sum(mask) >= 2
    TW_design_cruise = interp1(WS(mask), TW_env_feas(mask), WS_design, ...
                               'linear', 'extrap');
else
    mask = isfinite(TW_env);
    TW_design_cruise = interp1(WS(mask), TW_env(mask), WS_design, ...
                               'linear', 'extrap');
end

% Convert cruise T/W → SLS using turbofan lapse  T_SLS ~ T_alt / (rho/rho_sl)^0.75
[rho_c, ~] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
rho_sl      = 1.225;
lapse       = (rho_c / rho_sl)^0.75;
TW_SLS      = TW_design_cruise / lapse;
TW_SLS      = max(0.20, min(TW_SLS, 0.45));   % physical clamp

obj.WingLoading         = WS_design;
obj.ThrustToWeightRatio = TW_SLS;

%% ── Constraint diagram ────────────────────────────────────────────────────
if doPlot
    figure(101); clf;
    hold on; grid on; box on;
    set(gcf, 'Color', 'w', 'Position', [100 100 900 600]);

    % Colour palette
    c_to   = [0.20 0.45 0.80];
    c_roc  = [0.85 0.40 0.10];
    c_tocg = [0.20 0.65 0.30];
    c_macs = [0.60 0.20 0.70];
    c_env  = [0.10 0.10 0.10];

    % ── Shade feasible region ────────────────────────────────────────────
    idx  = feasible & isfinite(TW_env_feas) & isfinite(TW_to);
    xF   = WS(idx);
    yTop = TW_env_feas(idx);
    yBot = TW_to(idx);
    good = yTop >= yBot;
    xF = xF(good); yTop = yTop(good); yBot = yBot(good);

    if numel(xF) >= 2
        px = patch([xF, fliplr(xF)], [yBot, fliplr(yTop)], ...
                   [0.80 0.95 0.80], 'EdgeColor', 'none', 'FaceAlpha', 0.30);
        set(px, 'DisplayName', 'Feasible region');
        uistack(px, 'bottom');
    end

    % ── Constraint curves ────────────────────────────────────────────────
    plot(WS, TW_to,   '-',  'Color', c_to,   'LineWidth', 2, 'DisplayName', 'Take-off (TOL)');
    plot(WS, TW_roc,  '--', 'Color', c_roc,  'LineWidth', 2, 'DisplayName', 'Rate of climb (ROC)');
    plot(WS, TW_tocg, '-.', 'Color', c_tocg, 'LineWidth', 2, 'DisplayName', 'Take-off climb OEI (TOCG)');
    plot(WS, TW_macs, ':',  'Color', c_macs, 'LineWidth', 2, 'DisplayName', 'Cruise drag (MACS)');

    % ── Envelope ────────────────────────────────────────────────────────
    plot(WS, TW_env,      'k--', 'LineWidth', 1.5, 'DisplayName', 'Envelope (all)');
    if any(isfinite(TW_env_feas))
        plot(WS, TW_env_feas, 'k-',  'LineWidth', 2.5, 'DisplayName', 'Envelope (feasible)');
    end

    % ── Vertical constraints ─────────────────────────────────────────────
    ymax_plot = max(TW_env(isfinite(TW_env))) * 1.15;
    ymax_plot = max(ymax_plot, 0.50);

    xline(WSmax_lfl_SI,      ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, ...
          'DisplayName', sprintf('Landing field (%.0f N/m²)', WSmax_lfl_SI));
    xline(WSmax_approach_SI, ':', 'Color', [0.6 0.3 0.0], 'LineWidth', 1.5, ...
          'DisplayName', sprintf('Approach stall (%.0f N/m²)', WSmax_approach_SI));
    xline(WSmax_ceiling_SI,  ':', 'Color', [0.0 0.4 0.6], 'LineWidth', 1.5, ...
          'DisplayName', sprintf('Ceiling (%.0f N/m²)', WSmax_ceiling_SI));
    xline(WSmax_all_SI, '-', 'Color', [0.7 0.0 0.0], 'LineWidth', 2.0, ...
          'DisplayName', sprintf('W/S limit (%.0f N/m²)', WSmax_all_SI));

    % ── Design point ─────────────────────────────────────────────────────
    plot(WS_design, TW_SLS, 'ko', ...
         'MarkerSize', 12, 'MarkerFaceColor', 'k', ...
         'DisplayName', sprintf('Design point (W/S=%.0f N/m², T/W=%.3f)', ...
                                WS_design, TW_SLS));

    % Annotate
    text(WS_design + 80, TW_SLS + 0.005, ...
         sprintf('  W/S = %.0f N/m²\n  T/W_{SLS} = %.3f', WS_design, TW_SLS), ...
         'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold');

    % ── Formatting ───────────────────────────────────────────────────────
    xlabel('Wing Loading  W/S  [N/m²]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Thrust-to-Weight Ratio  T/W  [-]', 'FontSize', 12, 'FontWeight', 'bold');
    title('BoxWing Freighter — Constraint Diagram', 'FontSize', 14, 'FontWeight', 'bold');

    xlim([min(WS), max(WS)]);
    ylim([0, ymax_plot]);

    legend('Location', 'northwest', 'FontSize', 9);
    drawnow;

    fprintf('\n╔══════════════════════════════════════════╗\n');
    fprintf('║      CONSTRAINT ANALYSIS RESULTS         ║\n');
    fprintf('╠══════════════════════════════════════════╣\n');
    fprintf('║  W/S  design  : %7.0f  N/m²           ║\n', WS_design);
    fprintf('║  T/W  SLS     : %7.3f                  ║\n', TW_SLS);
    fprintf('╠══════════════════════════════════════════╣\n');
    fprintf('║  WSmax  LFL      : %6.0f  N/m²         ║\n', WSmax_lfl_SI);
    fprintf('║  WSmax  Approach : %6.0f  N/m²         ║\n', WSmax_approach_SI);
    fprintf('║  WSmax  Ceiling  : %6.0f  N/m²         ║\n', WSmax_ceiling_SI);
    fprintf('║  WSmax  (binding): %6.0f  N/m²         ║\n', WSmax_all_SI);
    fprintf('╚══════════════════════════════════════════╝\n\n');
end

end