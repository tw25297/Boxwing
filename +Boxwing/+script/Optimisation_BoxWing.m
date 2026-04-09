clear; clc; close all;

%% BOUNDS, INITIAL POINT, SHARED OPTIONS
%          N     M_c   Span   Alt   SAF   Range_km
lb = [     3,   0.80,   45,  10.5,   0,    5000 ];
ub = [    10,   0.92,   65,  12.5,   1,   16000 ];
x0 = [     5,   0.85,   60,  11.5,   1.0,  8000 ];  % nominal design


n_flights = 23; % N_FLIGHTS: THIS HAS TO BE LINKED TO MISSION ANALYSIS PM
 
% fmincon options — SQP is best for smooth nonlinear problems
opts_verbose = optimoptions('fmincon', ...
    'Algorithm',              'sqp', ...
    'Display',                'iter', ...
    'MaxFunctionEvaluations', 10, ...
    'MaxIterations',          10, ...   
    'OptimalityTolerance',    1e-4, ...
    'StepTolerance',          1e-5);

opts_quiet = optimoptions('fmincon', ...
    'Algorithm',              'sqp', ...
    'Display',                'off', ...
    'MaxFunctionEvaluations', 10, ...
    'MaxIterations',          10, ...   
    'OptimalityTolerance',    1e-4, ...
    'StepTolerance',          1e-5);


%%  WRAPPER FUNCTION (black box)


%%  MINIMISE DOC   (SAF locked at 100%)
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║           RUN 1 — MINIMISE DOC  (SAF = 100%%)               ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

% Lock SAF = 1.0 by tightening its bounds
lb1      = lb;  ub1 = ub;
lb1(5)   = 1.0; ub1(5) = 1.0;   % SAF fixed at 100%
x0_1     = x0;  x0_1(5) = 1.0;

obj_DOC = @(x) mdo_wrapper(x, 'DOC');

[x_opt1, DOC_opt1, flag1] = fmincon(obj_DOC, x0_1, ...
    [], [], [], [], lb1, ub1, [], opts_verbose);

% Evaluate both objectives at the Run 1 optimum for later normalisation
[DOC_at1, ATR_at1] = eval_both(x_opt1, n_flights);  

fprintf('\n── Run 1 Results ──────────────────────────────────────────\n');
fprintf('  Fleet size      : %d aircraft\n',  round(x_opt1(1)));
fprintf('  Cruise Mach     : %.3f\n',          x_opt1(2));
fprintf('  Wing span       : %.1f m\n',        x_opt1(3));
fprintf('  Cruise altitude : %.1f km\n',       x_opt1(4));
fprintf('  SAF ratio       : %.0f%%\n',        x_opt1(5)*100);
fprintf('  Design range    : %.0f km\n',       x_opt1(6));
fprintf('  → DOC           : $%.3f M/season\n', DOC_opt1/1e6);
fprintf('  → ATR100        : %.4e K\n',        ATR_at1);
fprintf('  Exit flag       : %d\n',            flag1);


%%  MINIMISE ATR100
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║           RUN 2 — MINIMISE ATR100                          ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

obj_ATR = @(x) mdo_wrapper(x, 'ATR');

[x_opt2, ATR_opt2, flag2] = fmincon(obj_ATR, x0, ...
    [], [], [], [], lb, ub, [], opts_verbose);

[DOC_at2, ATR_at2] = eval_both(x_opt2, n_flights);

fprintf('\n── Run 2 Results ──────────────────────────────────────────\n');
fprintf('  Fleet size      : %d aircraft\n',  round(x_opt2(1)));
fprintf('  Cruise Mach     : %.3f\n',          x_opt2(2));
fprintf('  Wing span       : %.1f m\n',        x_opt2(3));
fprintf('  Cruise altitude : %.1f km\n',       x_opt2(4));
fprintf('  SAF ratio       : %.0f%%\n',        x_opt2(5)*100);
fprintf('  Design range    : %.0f km\n',       x_opt2(6));
fprintf('  → ATR100        : %.4e K\n',        ATR_opt2);
fprintf('  → DOC           : $%.3f M/season\n', DOC_at2/1e6);
fprintf('  Exit flag       : %d\n',            flag2);


%%  PARETO FRONT: WEIGHTED SUM
%  Scalarised objective:  J = w * DOC_norm + (1-w) * ATR_norm
%  where DOC_norm = DOC / DOC_at1  and  ATR_norm = ATR / ATR_at2
%
%  w=1  →  pure DOC minimisation
%  w=0  →  pure ATR minimisation
%
%  We use the single-objective optima as normalisation anchors so that
%  both terms are ~O(1) and the weighting is physically meaningful.

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║      RUN 3a — PARETO FRONT: WEIGHTED SUM                   ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

DOC_ref = DOC_at1;    % best achievable DOC  (normalisation anchor)
ATR_ref = ATR_at2;    % best achievable ATR  (normalisation anchor)
fprintf('  Normalisation: DOC_ref = $%.2f M,  ATR_ref = %.4e K\n\n', ...
        DOC_ref/1e6, ATR_ref);

n_ws      = 5;
w_vals    = linspace(0, 1, n_ws);   % sweep from pure ATR to pure DOC

pareto_DOC_ws = nan(1, n_ws);
pareto_ATR_ws = nan(1, n_ws);
pareto_x_ws   = nan(n_ws, 6);

for i = 1:n_ws
    w = w_vals(i);
    fprintf('  [WS %2d/%d] w=%.2f ... ', i, n_ws, w);

    % Warm-start: interpolate between the two single-objective optima
    % This gives the optimiser a sensible starting point for each weight
    x0_i = (1-w) * x_opt2 + w * x_opt1;

    obj_ws = @(x) w * (mdo_wrapper(x,'DOC',n_flights) / DOC_ref) + ...
                  (1-w) * (mdo_wrapper(x,'ATR',n_flights) / ATR_ref);

    try
        [x_i, ~] = fmincon(obj_ws, x0_i, [], [], [], [], ...
                            lb, ub, [], opts_quiet);
        [pareto_DOC_ws(i), pareto_ATR_ws(i)] = eval_both(x_i, n_flights);
        pareto_x_ws(i,:) = x_i;
        fprintf('DOC=$%.2fM  ATR=%.3e K\n', ...
                pareto_DOC_ws(i)/1e6, pareto_ATR_ws(i));
    catch ME
        fprintf('FAILED: %s\n', ME.message);
    end
end


%% PARETO FRONT: ε-CONSTRAINT
%  Fix DOC ≤ ε (a hard constraint) and minimise ATR100.
%  Sweep ε from DOC_at1 (best DOC) to DOC_at2 (DOC at min-ATR point).
%  This sweeps the Pareto front from one end to the other.
%
%  The ε-constraint method is more rigorous than weighted sum because:
%  - It can find non-convex parts of the Pareto front
%  - The trade-off interpretation is clearer (every $ of DOC buys X K ATR)

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║      RUN 3b — PARETO FRONT: ε-CONSTRAINT                   ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

% ε values: evenly spaced from tightest (min-DOC) to loosest (min-ATR DOC)
eps_vals  = linspace(DOC_at1, DOC_at2, n_ws);

pareto_DOC_ec = nan(1, n_ws);
pareto_ATR_ec = nan(1, n_ws);
pareto_x_ec   = nan(n_ws, 6);

for i = 1:n_ws
    eps_i = eps_vals(i);
    fprintf('  [EC %2d/%d] ε=$%.2fM ... ', i, n_ws, eps_i/1e6);

    % Nonlinear inequality constraint: DOC(x) - ε ≤ 0
    con_eps = @(x) deal(mdo_wrapper(x,'DOC',n_flights) - eps_i, []);

    % Warm-start from the weighted-sum point at same index if available
    if ~any(isnan(pareto_x_ws(i,:)))
        x0_i = pareto_x_ws(i,:);
    else
        x0_i = x0;
    end

    try
        [x_i, ATR_i] = fmincon(@(x) mdo_wrapper(x,'ATR',n_flights), ...
                                x0_i, [], [], [], [], lb, ub, ...
                                con_eps, opts_quiet);
        [pareto_DOC_ec(i), pareto_ATR_ec(i)] = eval_both(x_i, n_flights);
        pareto_x_ec(i,:) = x_i;
        fprintf('DOC=$%.2fM  ATR=%.3e K\n', ...
                pareto_DOC_ec(i)/1e6, pareto_ATR_ec(i));
    catch ME
        fprintf('FAILED: %s\n', ME.message);
    end
end


%% PLOTS

%% ── Figure 1: Pareto Front ───────────────────────────────────────────
fig1 = figure('Name','Pareto Front','Color','w','Position',[100 100 800 550]);
hold on; grid on; box on;

% Plot both methods
valid_ws = ~isnan(pareto_DOC_ws);
valid_ec = ~isnan(pareto_DOC_ec);

plot(pareto_DOC_ws(valid_ws)/1e6, pareto_ATR_ws(valid_ws), ...
     'b-o','LineWidth',1.8,'MarkerSize',7,'DisplayName','Weighted sum');
plot(pareto_DOC_ec(valid_ec)/1e6, pareto_ATR_ec(valid_ec), ...
     'r-s','LineWidth',1.8,'MarkerSize',7,'DisplayName','\epsilon-constraint');

% Mark single-objective optima
scatter(DOC_at1/1e6, ATR_at1, 120, 'b', 'filled', 'p', ...
        'DisplayName','Min DOC optimum');
scatter(DOC_at2/1e6, ATR_at2, 120, 'r', 'filled', 'p', ...
        'DisplayName','Min ATR optimum');

xlabel('DOC [M\$/season]',  'FontSize',13);
ylabel('ATR_{100} [K]',     'FontSize',13);
title('Pareto Front: DOC vs Climate Impact (ATR_{100})', ...
      'FontSize',14,'FontWeight','bold');
legend('Location','northeast','FontSize',11);

saveas(fig1, 'Pareto_Front.png');
fprintf('\nFigure 1 saved: Pareto_Front.png\n');


%% ── Figure 2: Hyperparameters along Pareto (weighted-sum) ────────────
var_names  = {'Fleet size N', 'Cruise Mach', 'Span [m]', ...
              'Alt [km]', 'SAF ratio', 'Range [km]'};
transforms = {@(x)round(x), @(x)x, @(x)x, @(x)x, @(x)x*100, @(x)x};
ylabels    = {'N [-]','M_c [-]','b [m]','h [km]','SAF [%]','R [km]'};

fig2 = figure('Name','Hyperparameters along Pareto','Color','w', ...
              'Position',[100 100 1200 700]);
tt   = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

for k = 1:6
    nexttile;
    vals = arrayfun(transforms{k}, pareto_x_ws(valid_ws, k));
    plot(w_vals(valid_ws), vals, '-s','LineWidth',1.6,'MarkerSize',7);
    xlabel('Weight w  (0=ATR, 1=DOC)','FontSize',10);
    ylabel(ylabels{k},'FontSize',10);
    title(var_names{k},'FontSize',11,'FontWeight','bold');
    grid on;
end

title(tt,'Design Variables along Weighted-Sum Pareto Front', ...
      'FontSize',13,'FontWeight','bold');

saveas(fig2, 'Pareto_Hyperparameters.png');
fprintf('Figure 2 saved: Pareto_Hyperparameters.png\n');


%% ── Figure 3: DOC and ATR breakdown at three key points ──────────────
% Evaluate at: min-DOC point, knee point (w=0.5), min-ATR point
key_points = {x_opt1, pareto_x_ws(round(n_ws/2),:), x_opt2};
key_labels = {'Min DOC', 'Knee (w=0.5)', 'Min ATR'};
n_key      = 3;

DOC_items  = nan(n_key, 9);   % 9 DOC components
ATR_items  = nan(n_key, 6);   % 6 ATR species

for k = 1:n_key
    xk = key_points{k};
    [~, ~, doc_bd, atr_bd] = eval_full(xk, n_flights);
    DOC_items(k,:) = [doc_bd.crew, doc_bd.fuel, doc_bd.landing, ...
                      doc_bd.parking, doc_bd.navigation, ...
                      doc_bd.maintenance, doc_bd.depreciation, ...
                      doc_bd.interest, doc_bd.insurance];
    ATR_items(k,:) = [atr_bd.ATR_CO2, atr_bd.ATR_H2O, atr_bd.ATR_NOx, atr_bd.ATR_AIC];
end

fig3 = figure('Name','Cost & Climate Breakdown','Color','w', ...
              'Position',[100 100 1200 500]);
tl3  = tiledlayout(1,2,'TileSpacing','compact');

% DOC stacked bar
nexttile;
b1 = bar(DOC_items/1e6,'stacked');
set(gca,'XTickLabel',key_labels,'FontSize',11);
ylabel('DOC [M\$/season]','FontSize',12);
title('DOC Breakdown at Key Points','FontSize',13,'FontWeight','bold');
legend({'Crew','Fuel','Landing','Parking','Navigation', ...
        'Maintenance','Depreciation','Interest','Insurance'}, ...
       'Location','northeastoutside','FontSize',9);
grid on;

% ATR stacked bar
nexttile;
bar(ATR_items,'stacked');
set(gca,'XTickLabel',key_labels,'FontSize',11);
ylabel('ATR_{100} [K]','FontSize',12);
title('Climate Breakdown at Key Points','FontSize',13,'FontWeight','bold');
legend({'CO_2','H_2O','SO_4','Soot','NO_x','AIC'}, ...
       'Location','northeastoutside','FontSize',9);
grid on;

saveas(fig3, 'Breakdown_KeyPoints.png');
fprintf('Figure 3 saved: Breakdown_KeyPoints.png\n');


%%  SAVE RESULTS
results = struct();
results.Run1.x_opt      = x_opt1;
results.Run1.DOC        = DOC_at1;
results.Run1.ATR        = ATR_at1;
results.Run2.x_opt      = x_opt2;
results.Run2.DOC        = DOC_at2;
results.Run2.ATR        = ATR_at2;
results.Pareto_WS.w     = w_vals;
results.Pareto_WS.DOC   = pareto_DOC_ws;
results.Pareto_WS.ATR   = pareto_ATR_ws;
results.Pareto_WS.x     = pareto_x_ws;
results.Pareto_EC.eps   = eps_vals;
results.Pareto_EC.DOC   = pareto_DOC_ec;
results.Pareto_EC.ATR   = pareto_ATR_ec;
results.Pareto_EC.x     = pareto_x_ec;

save('OptimisationResults_BoxWing.mat', 'results');
fprintf('\nAll results saved to OptimisationResults_BoxWing.mat\n');

fprintf('\n╔════════════════════════════════════════════════╗\n');
fprintf('║         OPTIMISATION COMPLETE                  ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');


%%  LOCAL FUNCTIONS

function J = mdo_wrapper(x, objective)
% MDO_WRAPPER  Build ADP, run sizing, return requested objective.

PENALTY = 1e12;   % returned on any failure — huge so optimiser avoids it

% Unpack and clamp design variables
N_fleet   = max(1, round(x(1)));
M_c       = x(2);
Span_m    = x(3);
Alt_m     = x(4) * 1e3;                     % km → m
SAF_ratio = min(max(x(5), 0), 1);
Range_m   = x(6) * 1e3;                     % km → m

% Build a fresh ADP (never copy handles — always construct anew)
ADP = Boxwing.B777.ADP();
ADP.TLAR              = Boxwing.cast.TLAR.Boxwing();
ADP.TLAR.M_c          = M_c;
ADP.TLAR.Alt_cruise   = Alt_m;
ADP.TLAR.Alt_max      = max(Alt_m + 500, ADP.TLAR.Alt_max);
ADP.TLAR.Range        = Range_m;
ADP.TLAR.Payload      = 736e3 / N_fleet;    % fixed total payload / fleet

ADP.Engine = Boxwing.cast.eng.TurboFan.GE90(1.0, Alt_m, M_c);

ADP.CockpitLength  = 6.5;
ADP.CabinRadius    = 2.93;
ADP.CabinLength    = 70.0 - 6.5 - 2.93*2*1.48;
ADP.V_HT           = 0;
ADP.V_VT           = 0.05;

ADP.FrontWingSpan  = Span_m;
ADP.RearWingSpan   = Span_m;
ADP.ConnectorHeight = 8;
ADP.updateDerivedProps();

% Class-I seed
ADP.MTOM    = 3.0 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.28;
ADP.Mf_res  = 0.04;
ADP.Mf_Ldg  = 0.75;
ADP.Mf_TOC  = 0.98;

% [~, ~, total_fuel_burn] = Engine_code(MTOM, OEW, s_ref, ar, range_km,  mach)
[MissionAnalysis] = Boxwing.script.MissionAnalysisPM(N_fleet, M_c, ADP.TLAR.Alt_cruise, ADP.MTOM, ADP.TLAR.Range/1000);
n_flights = MissionAnalysis.num_landings;
% Run MDO inner loop
try
    [ADP, sizing_out] = Boxwing.B777.Size(ADP, false);
    % [BoxGeom, BoxMass] = BoxWing.B777.BuildGeometry(ADP);
catch
    J = PENALTY;
    return;
end

% Sanity check — reject physically implausible results
if ~isfinite(ADP.MTOM) || ADP.MTOM > 5e6 || ADP.MTOM < 1e4
    J = PENALTY;
    return;
end

% Evaluate objective
T_max_kN = ADP.Engine.T_Static / 1000;

switch upper(objective)
    case 'DOC'
        try
            J = Boxwing.script.DOC(ADP.MTOM/1e3, ADP.OEM, sizing_out.BlockFuel, ...
                    N_fleet, SAF_ratio, M_c, T_max_kN);
        catch
            J = PENALTY;
        end

    case 'ATR'
        try
            J = Boxwing.script.ClimateImpact(ADP, sizing_out.BlockFuel, N_fleet, n_flights, SAF_ratio);
        catch
            J = PENALTY;
        end

    otherwise
        error('Unknown objective: %s', objective);
end

% Final guard — if result is non-finite, return penalty
if ~isfinite(J)
    J = PENALTY;
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [DOC, ATR] = eval_both(x, n_flights)
% EVAL_BOTH  Evaluate both objectives at a design point.
    DOC = mdo_wrapper(x, 'DOC');
    ATR = mdo_wrapper(x, 'ATR');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [DOC, ATR, doc_bd, atr_bd] = eval_full(x, n_flights)
% EVAL_FULL  Evaluate both objectives AND return full breakdown structs.
%  Used only for the breakdown figure — not called during optimisation.

PENALTY = 1e12;

N_fleet   = max(1, round(x(1)));
M_c       = x(2);
Span_m    = x(3);
Alt_m     = x(4) * 1e3;
SAF_ratio = min(max(x(5), 0), 1);
Range_m   = x(6) * 1e3;

ADP = BoxWing.B777.ADP();
ADP.TLAR              = BoxWing.cast.TLAR.Boxwing();
ADP.TLAR.M_c          = M_c;
ADP.TLAR.Alt_cruise   = Alt_m;
ADP.TLAR.Alt_max      = max(Alt_m + 500, ADP.TLAR.Alt_max);
ADP.TLAR.Range        = Range_m;
ADP.TLAR.Payload      = 736e3 / N_fleet;
ADP.Engine            = BoxWing.cast.eng.TurboFan.GE90(1.0, Alt_m, M_c);
ADP.CockpitLength     = 6.5;
ADP.CabinRadius       = 2.93;
ADP.CabinLength       = 70.0 - 6.5 - 2.93*2*1.48;
ADP.V_HT = 0; ADP.V_VT = 0.05;
ADP.FrontWingSpan     = Span_m;
ADP.RearWingSpan      = Span_m;
ADP.ConnectorHeight   = 8;
ADP.updateDerivedProps();
ADP.MTOM    = 3.0 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.28; ADP.Mf_res = 0.04;
ADP.Mf_Ldg  = 0.75; ADP.Mf_TOC = 0.98;

[ADP, sizing_out] = BoxWing.B777.Size(ADP, false);

T_max_kN = ADP.Engine.T_Static / 1000;

[DOC, doc_bd] = BoxWing.script.DOC( ...
    ADP.MTOM/1e3, ADP.OEM, sizing_out.BlockFuel, ...
    N_fleet, SAF_ratio, M_c, T_max_kN);

s_ref = 422.5;
[ATR, atr_bd] = BoxWing.cast.eng.Engine_code(ADP.MTOM, ADP.OEM, s_ref, ADP.AR_target, ADP.TLAR.Range, ADP.TLAR.M_c);

end