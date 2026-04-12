%  STRUCTURE:
%   Phase 1 — Latin Hypercube Sampling (design space exploration)
%   Phase 2 — SQP (fmincon)            (single-objective: DOC, ATR100)
%   Phase 3 — NSGA-II                  (multi-objective Pareto front)
%   Phase 4 — epsilon-constraint SQP   (refinement of NSGA-II front)

%  BEFORE RUNNING: comment out ALL figure/disp calls inside sizing loop:
%    MissionAnalysis  -> disp(cruise_FL) and figure(11)
%    ConstraintAnalysis -> the plot block at the bottom
%    Size.m -> all fprintf inside the while loop
clear; clc; close all;
%% PROBLEM DEFINITION
%  DESIGN VECTOR  x = [N, M_c, Span, Alt_km, SAF, Range_km]
%   x(1) N_fleet    8-20       fleet size (rounded inside wrapper)
%   x(2) M_c        0.80-0.92  cruise Mach
%   x(3) Span_m     50-65      front & rear wing span [m]
%   x(4) Alt_km     10.5-12.5  cruise altitude [km]
%   x(5) SAF_ratio  0-1        SAF blend fraction
%   x(6) Range_km   7000-12000 design range [km]

%    payload [kg] M_c   Span   Alt   SAF   Range_km  MTOM_t
lb = [ 100000,    0.80,   40,  10.0,   0,    10000  ]; % lower boundary
ub = [250000,    0.92,   65,  15.5,   1,   17000  ]; % upper boundary
n_vars = numel(lb);

x_nom = [123000,   0.85,   60,  11.0,   1.0,  8000 ];  % nominal design
MTOM = 450;
TOTAL_PAYLOAD_KG = 736e3;   % fixed mission total payload [kg]
fleet_nom = round(TOTAL_PAYLOAD_KG / x_nom(1));  % implied fleet size at nominal payload per ac
[results] = Boxwing.script.MissionAnalysis.MissionAnalysisPM(fleet_nom, x_nom(2), x_nom(4), MTOM,  x_nom(6));
N_FLIGHTS        = results.num_landings;      % flights/aircraft/season (F1: 18 legs + 5 refuels)
PENALTY          = 1e12;    % returned on any failure

var_names = {'Payload/ac [kg]','Mach','Span [m]','Alt [km]','SAF','Range [km]'};

%% PHASE 1 — LATIN HYPERCUBE SAMPLING
%
%  WHY LHS OVER RANDOM MONTE CARLO?
%  With 6 dimensions, pure random sampling leaves large voids by chance.
%  LHS stratifies each axis into n_lhs equal intervals and places exactly
%  one sample per interval, guaranteeing uniform coverage with far fewer
%  points. 100 LHS points covers the space as well as ~600 random points.
%
%  WHAT WE USE THE RESULTS FOR:
%  1. Feasibility map: which regions of the design space converge?
%  2. Sensitivity: Pearson correlation shows which variable drives DOC/ATR.
%  3. Warm-starts: best LHS points seed SQP and NSGA-II initial population.

n_lhs = 100;
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  PHASE 1 — Latin Hypercube Sampling  (%3d points)        ║\n', n_lhs);
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

rng(42,'twister');   % reproducible results
lhs_unit    = lhsdesign(n_lhs, n_vars);
lhs_samples = lb + lhs_unit .* (ub - lb);
% lhs_DOC = zeros(n_lhs,1);
% lhs_ATR = zeros(n_lhs,1);

% Use parfor if Parallel Computing Toolbox is available; falls back to for
% useParallel = license('test','Distrib_Computing_Toolbox');
% if useParallel
%     try; parpool('local'); catch; end
%     parfor i = 1:n_lhs
%         xi         = lhs_samples(i,:);
%         lhs_DOC(i) = mdo_wrapper(xi,'DOC',N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);
%         lhs_ATR(i) = mdo_wrapper(xi,'ATR',N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);
%     end
% else
    for i = 1:n_lhs
        xi         = lhs_samples(i,:);
        lhs_DOC(i) = mdo_wrapper(xi,'DOC',N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);
        lhs_ATR(i) = mdo_wrapper(xi,'ATR',N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);
        fprintf('  LHS [%3d/%d]  DOC=$%.1fM  ATR=%.3e\n', i, n_lhs, lhs_DOC(i)/1e6, lhs_ATR(i));
    end
% end

feasible = (lhs_DOC < PENALTY*0.9) & (lhs_ATR < PENALTY*0.9);
n_feas   = sum(feasible);
fprintf('\n  %d / %d points feasible (%.0f%%)\n\n', n_feas, n_lhs, 100*n_feas/n_lhs);

% Best LHS seeds for SQP warm-starts
[~, i_doc] = min(lhs_DOC);   x_seed_doc = lhs_samples(i_doc,:);
[~, i_atr] = min(lhs_ATR);   x_seed_atr = lhs_samples(i_atr,:);

%% LHS figures ──────────────────────────────────────────────────────
fig1 = figure('Name','LHS Exploration','Color','w','Position',[50 50 1400 480]);
tl1  = tiledlayout(1,2,'TileSpacing','compact');
title(tl1,'Phase 1 — LHS Design Space Exploration', 'FontSize',13,'FontWeight','bold');

nexttile; hold on; grid on;
scatter(lhs_DOC(feasible)/1e6,  lhs_ATR(feasible),  50,'b','filled', 'DisplayName','Feasible');
% scatter(lhs_DOC(~feasible)/1e6, lhs_ATR(~feasible), 30,'r','x', 'LineWidth',1.5,'DisplayName','Infeasible/Penalty');
xlabel('DOC [M$/season]','FontSize',11);
ylabel('ATR_{100} [K]','FontSize',11);
title('DOC vs ATR100 — LHS cloud');

legend('Location','best');

nexttile; hold on; grid on;
if n_feas > 5
    X_f       = lhs_samples(feasible,:);
    corr_doc  = corr(X_f, lhs_DOC(feasible)');
    corr_atr  = corr(X_f, lhs_ATR(feasible)');
    b = barh([corr_doc, corr_atr]);
    b(1).FaceColor = [0.2 0.5 0.8];
    b(2).FaceColor = [0.9 0.4 0.1];
    set(gca,'YTickLabel',var_names,'FontSize',10);
    xlabel('Pearson correlation','FontSize',11);
    title('Sensitivity: correlation with objectives');
    legend({'DOC','ATR100'},'Location','best');
    xline(0,'k--');
end

saveas(fig1,'LHS_DesignSpace.png');
fprintf('Figure saved: LHS_DesignSpace.png\n\n');

%% PHASE 2 — SQP (fmincon) — Single-Objective Optima
%  WHY SQP?
%  Sequential Quadratic Programming approximates the problem locally with a
%  quadratic model and solves it exactly at each step. For smooth problems
%  like ours (continuous outputs from the sizing loop), it converges in
%  50-150 function evaluations — roughly 30x faster than NSGA-II for a
%  single objective. The SQP optima serve as anchors that validate the
%  endpoints of the NSGA-II Pareto front.

opts_sqp = optimoptions('fmincon', ...
    'Algorithm',              'sqp', ...
    'Display',                'iter', ...
    'MaxFunctionEvaluations', 500, ...
    'MaxIterations',          200, ...
    'OptimalityTolerance',    1e-4, ...
    'StepTolerance',          1e-5, ...
    'UseParallel',            false);

% ── Run 1: Minimise DOC  (SAF = 100%) ─────────────────────────────────
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  PHASE 2a — SQP: Minimise DOC  (SAF locked = 100%%)      ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

lb1 = lb; ub1 = ub;
lb1(5) = 1.0; ub1(5) = 1.0;   % lock SAF at 100%
x0_1   = x_seed_doc; x0_1(5) = 1.0;

obj_DOC = @(x) mdo_wrapper(x,'DOC',N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);

[x_opt1, ~, flag1] = fmincon(obj_DOC, x0_1, ...
    [],[],[],[], lb1, ub1, [], opts_sqp);
[DOC_at1, ATR_at1] = eval_both(x_opt1,N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);

fprintf('\n── SQP Run 1 (min DOC) ─────────────────────────────────────\n');
print_result(x_opt1, DOC_at1, ATR_at1, flag1);

% ── Run 2: Minimise ATR100 ────────────────────────────────────────────
fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  PHASE 2b — SQP: Minimise ATR100                        ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');
obj_ATR = @(x) mdo_wrapper(x,'ATR',N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);

% Seed
x_seeds_atr = [x_seed_atr; x_opt1];   % try both seeds
best_ATR2 = inf;  best_x2 = x_seed_atr;
% for s = 1:size(x_seeds_atr, 1)
%    try
%        [xi, fi] = fmincon(obj_ATR, x_seeds_atr(s,:), [], [], [], [], ...
%                           lb, ub, [], opts_sqp);
%        if fi < best_ATR2
%            best_ATR2 = fi;  best_x2 = xi;
%        end
%    catch; end
%end
%x_opt2 = best_x2;
%flag2 = best_ATR2;

 [x_opt2, ~, flag2] = fmincon(obj_ATR, x_seed_atr, ...
     [],[],[],[], lb, ub, [], opts_sqp);
[DOC_at2, ATR_at2] = eval_both(x_opt2,N_FLIGHTS,TOTAL_PAYLOAD_KG,PENALTY);

fprintf('\n── SQP Run 2 (min ATR100) ──────────────────────────────────\n');
print_result(x_opt2, DOC_at2, ATR_at2, flag2);

%% PHASE 3 — NSGA-II (gamultiobj) — Full Multi-Objective Pareto Front
%
%  WHY NSGA-II OVER WEIGHTED-SUM?
%  NSGA-II (Non-dominated Sorting Genetic Algorithm II) solves the full
%  multi-objective problem in ONE run and recovers the complete Pareto front
%  including non-convex regions that weighted-sum cannot find.
%
%  NSGA-II maintains a population and evolves it over generations, sorting
%  solutions by Pareto rank and crowding distance. Solutions on rank-1
%  (the Pareto front) are kept; dominated solutions are discarded. The
%  result is a set of ~50 diverse Pareto-optimal designs in one pass.
%
%  KEY SETTINGS:
%   PopulationSize 80: enough diversity for 6 variables; bigger = better
%                      coverage but slower (each generation = 80 MDO calls)
%   MaxGenerations 150: 150 x 80 = 12000 evaluations total; with parfor
%                       and ~0.2s per call this is ~40 min
%   ParetoFraction 0.5: half the population maintained on the Pareto front
%   InitialPopulation: seeded with LHS feasible points + SQP optima so
%                      NSGA-II starts near good solutions (faster convergence)

fprintf('\n╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  PHASE 3 — NSGA-II: Multi-Objective Pareto Front        ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

opts_nsga = optimoptions('gamultiobj', ...
    'PopulationSize',       80, ...
    'MaxGenerations',       150, ...
    'ParetoFraction',       0.5, ...
    'CrossoverFraction',    0.8, ...
    'FunctionTolerance',    1e-4, ...
    'UseParallel',          true, ...
    'Display',              'iter', ...
    'PlotFcn',              []);

% Seed initial population with LHS feasible points + SQP optima
if n_feas >= 5
    seed_rows = lhs_samples(feasible,:);
    seed_pop  = [seed_rows; x_opt1; x_opt2];
    pop_size  = opts_nsga.PopulationSize;
    if size(seed_pop,1) > pop_size
        seed_pop = seed_pop(1:pop_size,:);
    end
    opts_nsga.InitialPopulationMatrix = seed_pop;
end

obj_multi = @(x) multi_obj(x, N_FLIGHTS, TOTAL_PAYLOAD_KG, PENALTY);

[x_pareto, fval, flag3] = gamultiobj(obj_multi, n_vars, ...
    [],[],[],[], lb, ub, [], opts_nsga);

% Filter out penalty points
valid_p   = (fval(:,1) < PENALTY*0.9) & (fval(:,2) < PENALTY*0.9);
DOC_nsga  = fval(valid_p,1);
ATR_nsga  = fval(valid_p,2);
x_nsga    = x_pareto(valid_p,:);

fprintf('\n  NSGA-II: %d valid Pareto points found (flag=%d)\n\n', ...
        sum(valid_p), flag3);

%% PHASE 4 — epsilon-constraint SQP  (refinement)
%  NSGA-II finds the shape of the Pareto front well but individual points
%  can be imprecise (genetic algorithms don't converge to machine precision).
%  We take 15 evenly-spaced points from the NSGA-II front as warm-starts
%  for SQP with a hard DOC <= epsilon constraint, which sharpens each point
%  to the SQP tolerance level. Comparing NSGA-II and epsilon-constraint
%  curves validates the result: if they agree, the front is correct.

fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  PHASE 4 — epsilon-constraint SQP refinement (15 pts)   ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

n_refine = 10;
opts_ec  = optimoptions('fmincon','Algorithm','sqp','Display','off', ...
                        'MaxFunctionEvaluations',300,'MaxIterations',100);

% Select 15 evenly spaced points across the NSGA-II front (by DOC)
[DOC_sorted, s_idx] = sort(DOC_nsga);
pick = round(linspace(1, numel(DOC_sorted), n_refine));

DOC_ec = nan(n_refine,1);
ATR_ec = nan(n_refine,1);
x_ec   = nan(n_refine, n_vars);

for i = 1:n_refine
    k     = s_idx(pick(i));
    eps_i = DOC_sorted(pick(i)) * 1.02;   % 2% slack to ensure feasibility
    x0_i  = x_nsga(k,:);

    fprintf('  [ep %2d/%d]  eps=$%.2fM ... ', i, n_refine, eps_i/1e6);

    con_ep = @(x) deal(mdo_wrapper(x,'DOC',N_FLIGHTS, ...
                       TOTAL_PAYLOAD_KG,PENALTY) - eps_i, []);
    try
        [xi, ~] = fmincon(obj_ATR, x0_i,[],[],[],[], lb, ub, con_ep, opts_ec);
        [DOC_ec(i), ATR_ec(i)] = eval_both(xi,N_FLIGHTS, ...
                                            TOTAL_PAYLOAD_KG,PENALTY);
        x_ec(i,:) = xi;
        fprintf('DOC=$%.2fM  ATR=%.3e\n', DOC_ec(i)/1e6, ATR_ec(i));
    catch
        fprintf('failed\n');
    end
end

valid_ec = ~isnan(DOC_ec) & (DOC_ec < PENALTY*0.9);

%% OUTPUT FIGURES

%% ── Figure 2: Pareto front (NSGA-II + epsilon-constraint) ────────────
fig2 = figure('Name','Pareto Front','Color','w','Position',[100 100 900 600]);
hold on; grid on; box on;

scatter(DOC_nsga/1e6, ATR_nsga, 60, [0.2 0.5 0.85], 'filled', ...
        'DisplayName','NSGA-II Pareto front');
if any(valid_ec)
    scatter(DOC_ec(valid_ec)/1e6, ATR_ec(valid_ec), 80, ...
            [0.9 0.4 0.1], 'filled','^', ...
            'DisplayName','\epsilon-constraint (SQP refined)');
end
scatter(DOC_at1/1e6, ATR_at1, 160,'b','p','filled', ...
        'DisplayName','Min DOC  (SQP, SAF=100%)');
scatter(DOC_at2/1e6, ATR_at2, 160,'r','p','filled', ...
        'DisplayName','Min ATR100 (SQP)');

xlabel('DOC [M$/season]','FontSize',13);
ylabel('ATR_{100} [K]','FontSize',13);
title('Pareto Front: DOC vs Climate Impact (ATR_{100})', ...
      'FontSize',14,'FontWeight','bold');
legend('Location','northeast','FontSize',11);
grid on;

saveas(fig2,'Pareto_Front.png');
fprintf('\nFigure saved: Pareto_Front.png\n');

%% ── Figure 3: Algorithm comparison ──────────────────────────────────
fig3 = figure('Name','Algorithm Comparison','Color','w', ...
              'Position',[150 150 800 550]);
hold on; grid on; box on;
scatter(DOC_nsga/1e6, ATR_nsga, 50, [0.2 0.5 0.85], 'filled', ...
        'DisplayName','NSGA-II');
if any(valid_ec)
    scatter(DOC_ec(valid_ec)/1e6, ATR_ec(valid_ec), 70, ...
            [0.9 0.4 0.1],'^','filled', ...
            'DisplayName','\epsilon-constraint SQP');
end
xlabel('DOC [M$/season]','FontSize',12);
ylabel('ATR_{100} [K]','FontSize',12);
title({'Algorithm comparison: NSGA-II vs \epsilon-constraint', ...
       'If curves agree, the Pareto front is validated'}, ...
      'FontSize',12,'FontWeight','bold');
legend('Location','northeast','FontSize',11);

saveas(fig3,'Algorithm_Comparison.png');
fprintf('Figure saved: Algorithm_Comparison.png\n');

%% ── Figure 4: Hyperparameters along Pareto front ─────────────────────
if size(x_nsga,1) > 3
    [~, sp] = sort(DOC_nsga);
    xs = x_nsga(sp,:);  ds = DOC_nsga(sp)/1e6;

    transforms = {@(v)v/1e3, @(v)v, @(v)v, @(v)v, @(v)v*100, @(v)v};
    ylabs      = {'Payload/ac [t]','M_c [-]','Span [m]','Alt [km]','SAF [%]','Range [km]'};
    fig4 = figure('Name','Pareto Hyperparameters','Color','w', ...
                  'Position',[100 100 1300 700]);
    tl4 = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
    title(tl4,'Design Variables along NSGA-II Pareto Front', ...
          'FontSize',13,'FontWeight','bold');

    for k = 1:6
        nexttile;
        vals = arrayfun(transforms{k}, xs(:,k));
        scatter(ds, vals, 40, linspace(0,1,size(xs,1)), 'filled');
        colormap(gca, parula);
        xlabel('DOC [M$/season]','FontSize',10);
        ylabel(ylabs{k},'FontSize',10);
        title(var_names{k},'FontSize',11,'FontWeight','bold');
        grid on;
    end
    nexttile;
    fleet_implied = round(736e3 ./ xs(:,1));
    scatter(ds, fleet_implied, 40, linspace(0,1,size(xs,1)), 'filled');
    colormap(gca, parula);
    xlabel('DOC [M$/season]','FontSize',10);
    ylabel('Fleet size N [-]','FontSize',10);
    title('Implied Fleet Size','FontSize',11,'FontWeight','bold');
    grid on;
    
    saveas(fig4,'Pareto_Hyperparameters.png');
    fprintf('Figure saved: Pareto_Hyperparameters.png\n');
end

%% ── Figure 5: DOC and ATR breakdown at three key points ──────────────
% Evaluate at: min-DOC point, knee point (w=0.5), min-ATR point
% key_points = {x_opt1, pareto_x_ws(round(n_ws/2),:), x_opt2};
% key_labels = {'Min DOC', 'Knee (w=0.5)', 'Min ATR'};
% n_key      = 3;
% 
% DOC_items  = nan(n_key, 9);   % 9 DOC components
% ATR_items  = nan(n_key, 4);   % 4 ATR species
% 
% for k = 1:n_key
%     xk = key_points{k};
%     [~, ~, doc_bd, atr_bd] = eval_full(xk, N_FLIGHTS);
%     DOC_items(k,:) = [doc_bd.crew, doc_bd.fuel, doc_bd.landing, ...
%                       doc_bd.parking, doc_bd.navigation, ...
%                       doc_bd.maintenance, doc_bd.depreciation, ...
%                       doc_bd.interest, doc_bd.insurance];
%     ATR_items(k,:) = [atr_bd.ATR_CO2, atr_bd.ATR_H2O, ...
%                       atr_bd.ATR_NOx, atr_bd.ATR_AIC];
% end
% 
% fig5 = figure('Name','Cost & Climate Breakdown','Color','w', ...
%               'Position',[100 100 1200 500]);
% tl5  = tiledlayout(1,2,'TileSpacing','compact');
% 
% % DOC stacked bar
% nexttile;
% b1 = bar(DOC_items/1e6,'stacked');
% set(gca,'XTickLabel',key_labels,'FontSize',11);
% ylabel('DOC [M\$/season]','FontSize',12);
% title('DOC Breakdown at Key Points','FontSize',13,'FontWeight','bold');
% legend({'Crew','Fuel','Landing','Parking','Navigation', ...
%         'Maintenance','Depreciation','Interest','Insurance'}, ...
%        'Location','northeastoutside','FontSize',9);
% grid on;
% 
% % ATR stacked bar
% nexttile;
% bar(ATR_items,'stacked');
% set(gca,'XTickLabel',key_labels,'FontSize',11);
% ylabel('ATR_{100} [K]','FontSize',12);
% title('Climate Breakdown at Key Points','FontSize',13,'FontWeight','bold');
% legend({'CO_2','H_2O','SO_4','Soot','NO_x','AIC'}, ...
%        'Location','northeastoutside','FontSize',9);
% grid on;
% 
% saveas(fig5, 'Breakdown_KeyPoints.png');
% fprintf('Figure 5 saved: Breakdown_KeyPoints.png\n');

%% SAVE ALL RESULTS
results = struct();
results.LHS.samples  = lhs_samples;
results.LHS.DOC      = lhs_DOC;
results.LHS.ATR      = lhs_ATR;
results.LHS.feasible = feasible;

results.SQP.minDOC.x   = x_opt1;
results.SQP.minDOC.DOC = DOC_at1;
results.SQP.minDOC.ATR = ATR_at1;
results.SQP.minATR.x   = x_opt2;
results.SQP.minATR.DOC = DOC_at2;
results.SQP.minATR.ATR = ATR_at2;

results.NSGA.x   = x_nsga;
results.NSGA.DOC = DOC_nsga;
results.NSGA.ATR = ATR_nsga;

results.EC.x       = x_ec;
results.EC.DOC     = DOC_ec;
results.EC.ATR     = ATR_ec;
results.EC.valid   = valid_ec;

save('OptimisationResults_BoxWing.mat','results');
fprintf('\nAll results saved -> OptimisationResults_BoxWing.mat\n');
fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║            OPTIMISATION COMPLETE                     ║\n');
fprintf('╚══════════════════════════════════════════════════════╝\n\n');


%% LOCAL FUNCTIONS

function J = mdo_wrapper(x, objective, n_flights, total_payload, penalty)
%MDO_WRAPPER  Single MDO evaluation: build ADP -> size -> objective.

    payload_per_ac = x(1);                          % kg, continuous
    N_fleet        = max(1, round(total_payload / payload_per_ac));
    M_c       = x(2);
    Span_m    = x(3);
    Alt_m     = x(4) * 1e3;
    SAF_ratio = min(max(x(5),0),1);
    Range_m   = x(6) * 1e3;
    % MTOM_t = x(7);

    % try
        ADP = Boxwing.B777.ADP();                       
        ADP.TLAR            = Boxwing.cast.TLAR.Boxwing();     
        ADP.TLAR.M_c        = M_c;
        ADP.TLAR.Alt_cruise = Alt_m;
        ADP.TLAR.Alt_max    = max(Alt_m + 500, ADP.TLAR.Alt_max);
        ADP.TLAR.Range      = Range_m;
        ADP.TLAR.Payload    = payload_per_ac;
        ADP.Engine = Boxwing.cast.eng.TurboFan.GE90(1.0, Alt_m, M_c);  
        ADP.CockpitLength   = 6.5;
        ADP.CabinRadius     = 2.93;
        ADP.CabinLength     = 70.0 - 6.5 - 2.93*2*1.48;
        ADP.V_HT = 0;  ADP.V_VT = 0.05;
        ADP.FrontWingSpan   = Span_m;
        ADP.RearWingSpan    = Span_m;
        ADP.ConnectorHeight = 8;
        ADP.updateDerivedProps();
       
        if ADP.EffectiveSpan < 45 || ADP.EffectiveSpan > 70
            J = penalty; return;
        end
        % Check AR makes sense
        AR_check = ADP.EffectiveSpan^2 / ADP.WingArea;
        if AR_check < 6 || AR_check > 18
            J = penalty; return;
        end
        
        ADP.MTOM    = 3*ADP.TLAR.Payload;
        ADP.Mf_Fuel = 0.28;  ADP.Mf_res = 0.04;
        ADP.Mf_Ldg  = 0.75;  ADP.Mf_TOC = 0.98;
    % catch
        % J = penalty; return;
    % end

    try
        [ADP, out] = Boxwing.B777.Size(ADP, false);     
    catch
        J = penalty; return;
    end

    if ~isfinite(ADP.MTOM) || ADP.MTOM > 5e6 || ADP.MTOM < 1e4
        J = penalty; return;
    end

    % Payload per aircraft must be achievable
    payload_per_ac = total_payload / N_fleet;
    if payload_per_ac > 200000   % kg — above A350F max payload, penalise
        J = penalty; return;
    end
    if payload_per_ac < 90000    % too small to be a freighter, penalise  
        J = penalty; return;
    end
    if N_fleet < 3 || N_fleet > 20
        J = penalty; return;
    end
    
    % MTOM/payload ratio sanity check (should be 2.5–4.5 for freighters)
    % mtom_ratio = ADP.MTOM / payload_per_ac;
    % if mtom_ratio < 2.0 || mtom_ratio > 5.5
    %     J = penalty; return;
    % end
    % Range sanity — if mission analysis gives implausible fuel burn, reject
    if out.BlockFuel / ADP.MTOM > 0.55   % >55% fuel fraction is unrealistic
        J = penalty; return;
    end
    if out.BlockFuel / ADP.MTOM < 0.05   % <5% implies range wasn't used
        J = penalty; return;
    end

    T_max_kN = ADP.Engine.T_Static / 1000;

    try
        switch upper(objective)
            case 'DOC'
                J = Boxwing.script.DOC( ...                
                        ADP.MTOM/1e3, ADP.OEM, out.BlockFuel, ...
                        N_fleet, SAF_ratio, M_c, T_max_kN);
            case 'ATR'
                % J = Boxwing.script.ClimateImpact( ...      % [ADAPT]
                %         ADP, out.BlockFuel, N_fleet, n_flights, SAF_ratio);
                [ATR, ~] = Boxwing.cast.eng.Engine_code(ADP.MTOM, ADP.OEM, ADP.WingArea, ADP.AR_target, ADP.TLAR.Range, ADP.TLAR.M_c);
                J = ATR;
            otherwise
                error('Unknown objective: %s', objective);
        end
    catch
        J = penalty; return;
    end

    if ~isfinite(J) || J <= 0;  J = penalty;  end
end


function f = multi_obj(x, n_flights, total_payload, penalty)
%MULTI_OBJ  [DOC, ATR100] vector for gamultiobj / NSGA-II.
    f(1) = mdo_wrapper(x,'DOC',n_flights,total_payload,penalty);
    f(2) = mdo_wrapper(x,'ATR',n_flights,total_payload,penalty);
end


function [DOC, ATR] = eval_both(x, n_flights, total_payload, penalty)
    DOC = mdo_wrapper(x,'DOC',n_flights,total_payload,penalty);
    ATR = mdo_wrapper(x,'ATR',n_flights,total_payload,penalty);
end


function [DOC, ATR, doc_bd, atr_bd] = eval_full(x, n_flights)
% EVAL_FULL  Evaluate both objectives AND return full breakdown structs.
%  Used only for the breakdown figure — not called during optimisation.

PENALTY = 1e12;

payload_per_ac   = x(1);
N_fleet          = max(1, round(736e3 / payload_per_ac));
M_c       = x(2);
Span_m    = x(3);
Alt_m     = x(4) * 1e3;
SAF_ratio = min(max(x(5), 0), 1);
Range_m   = x(6) * 1e3;
% MTOM_t    = x(7);

ADP = Boxwing.B777.ADP();
ADP.TLAR              = Boxwing.cast.TLAR.Boxwing();
ADP.TLAR.M_c          = M_c;
ADP.TLAR.Alt_cruise   = Alt_m;
ADP.TLAR.Alt_max      = max(Alt_m + 500, ADP.TLAR.Alt_max);
ADP.TLAR.Range        = Range_m;
ADP.TLAR.Payload      = payload_per_ac;
ADP.Engine            = Boxwing.cast.eng.TurboFan.GE90(1.0, Alt_m, M_c);
ADP.CockpitLength     = 6.5;
ADP.CabinRadius       = 2.93;
ADP.CabinLength       = 70.0 - 6.5 - 2.93*2*1.48;
ADP.V_HT = 0; ADP.V_VT = 0.05;
ADP.FrontWingSpan     = Span_m;
ADP.RearWingSpan      = Span_m;
ADP.ConnectorHeight   = 8;
ADP.updateDerivedProps();
ADP.MTOM    = 3*ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.28; ADP.Mf_res = 0.04;
ADP.Mf_Ldg  = 0.75; ADP.Mf_TOC = 0.98;

[ADP, sizing_out] = Boxwing.B777.Size(ADP, false);

T_max_kN = ADP.Engine.T_Static / 1000;

[DOC, doc_bd] = Boxwing.script.DOC( ...
    ADP.MTOM/1e3, ADP.OEM, sizing_out.BlockFuel, ...
    N_fleet, SAF_ratio, M_c, T_max_kN);

s_ref = 422.5;
[ATR, atr_bd] = Boxwing.cast.eng.Engine_code(ADP.MTOM, ADP.OEM, s_ref, ADP.AR_target, ADP.TLAR.Range, ADP.TLAR.M_c);

end


function print_result(x, DOC, ATR, flag)
    labs = {'Payload/ac [kg]','Cruise Mach','Span [m]','Altitude [km]','SAF [%]','Range [km]'};
    vals = [x(1), x(2), x(3), x(4), x(5)*100, x(6)];
    fmts = {'%.0f','%.3f','%.1f','%.1f','%.0f','%.0f'};for k = 1:6
        fprintf(['  %-16s : ' fmts{k} '\n'], labs{k}, vals(k));
    end
    fprintf('  -> Fleet size  : %d aircraft\n', max(1, round(736e3 / x(1))));
    fprintf('  -> DOC    : $%.3f M/season\n', DOC/1e6);
    fprintf('  -> ATR100 : %.4e K\n', ATR);
    fprintf('  Exit flag : %d\n', flag);
end
