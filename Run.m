%% BOXWING FREIGHTER — COMPLETE SIZING AND ANALYSIS
%  Fixes applied:
%    1. Boxwing.ADP()           -> Boxwing.B777.ADP()
%    2. Boxwing.Size()          -> Boxwing.B777.Size()
%    3. Boxwing.BuildGeometry() -> Boxwing.B777.BuildGeometry()
%    4. Boxwing.UpdateAero()    -> Boxwing.B777.UpdateAero()
%    5. Boxwing.MissionAnalysis()-> Boxwing.B777.MissionAnalysis()
%    6. cast.TLAR.Boxwing()     -> Boxwing.cast.TLAR.Boxwing()
%    7. cast.draw()             -> Boxwing.cast.draw()
%    8. ADP (handle class) — trade loop uses proper deep-copy via freshADP()
%    9. Engine object with T_ref/D_ref/L_ref/M_ref baselines + TSFC
%       (Rubberise now handled inline in engine.m — no method needed)
%   10. Boxwing.cast.eng.Fuel.JA1.Density replaced with inline constant

clear; clc; close all;

%% 0. Setup
fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   BOXWING FREIGHTER — COMPREHENSIVE SIZING ANALYSIS        ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

projectRoot = pwd;
addpath(projectRoot);
clear classes;

% SI conversion factors (stored in base workspace for use by TLAR and geom files)
% Convention: dividing imperial value BY factor gives SI (metres, m/s, etc.)
% Multiplying stored SI value BY factor converts back for display.
SI.ft    = 3.28084;      % ft/m
SI.Nmile = 1 / 1852;    % NM/m   (Range*SI.Nmile gives NM)
SI.knt   = 1 / 0.5144;  % kts/(m/s)
SI.min   = 1 / 60;      % min/s
SI.lb    = 2.20462;     % lb/kg  (Raymer mass formulas)
SI.litre = 1000;        % L/m^3
SI.km    = 1e-3;
SI.hr    = 1 / 3600;
assignin('base', 'SI', SI);


%% ═══════════════════════════════════════════════════════════════════════
%  PART 1 — BASELINE SIZING
%% ═══════════════════════════════════════════════════════════════════════

%% Instantiate Boxwing ADP and set TLAR
ADP      = Boxwing.B777.ADP();
ADP.TLAR = Boxwing.cast.TLAR.Boxwing();
ADP.Engine = GE90Engine();   % engine baseline data + TSFC (scaling done in engine.m)

%% Set boxwing-specific parameters
% Fuselage geometry
ADP.CockpitLength = 6.5;
ADP.CabinRadius   = 2.93;
ADP.CabinLength   = 70.0 - ADP.CockpitLength - ADP.CabinRadius*2*1.48;

L_f = ADP.CockpitLength + ADP.CabinLength + ADP.CabinRadius*1.48;
ADP.FrontWingPos = 0.40 * L_f;
ADP.RearWingPos  = 0.75 * L_f;

% Boxwing configuration (no conventional horizontal tail)
ADP.V_HT = 0;
ADP.V_VT = 0.05;

%% Set hyperparameters (design variables)
ADP.FrontWingSpan = 60;   % [m]
ADP.RearWingSpan  = 50;   % [m]
ADP.ConnectorHeight = 3;  % [m] vertical gap between wings

ADP.updateDerivedProps();

%% Class-I estimates (initial guesses)
ADP.MTOM    = 3.0 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.28;
ADP.Mf_res  = 0.04;
ADP.Mf_Ldg  = 0.75;
ADP.Mf_TOC  = 0.98;

%% Initialize aerodynamic polar
Boxwing.B777.UpdateAero(ADP);

%% CD0 drag breakdown using DragMeta
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   CD0 DRAG BREAKDOWN (DragMeta)\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

try
    [CD0_val, cd0_breakdown] = Boxwing.B777.CD0(ADP);
    componentNames = {'Wing', 'Fuselage', 'Nacelles', 'HTP', 'VTP', 'Tail', 'Misc'};
    componentVals  = [cd0_breakdown.CD0_wing, cd0_breakdown.CD0_fuse, cd0_breakdown.CD0_nac, ...
                      cd0_breakdown.CD0_HTP,  cd0_breakdown.CD0_VTP,  cd0_breakdown.CD0_tail, ...
                      cd0_breakdown.CD0_misc];
    dragItems = Boxwing.cast.DragMeta(componentNames, componentVals);
    fprintf('  %-12s  CD0 (counts x10^4)\n', 'Component');
    fprintf('  %s\n', repmat('-',1,35));
    for k = 1:length(dragItems)
        fprintf('  %-12s  %.2f\n', dragItems(k).Name, dragItems(k).CD0 * 1e4);
    end
    fprintf('  %s\n', repmat('-',1,35));
    fprintf('  %-12s  %.2f\n\n', 'TOTAL', CD0_val * 1e4);
catch ME
    fprintf('  [SKIP] DragMeta breakdown: %s\n\n', ME.message);
end

%% Sizing loop
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   BASELINE SIZING (Span = %.0f m)\n', ADP.FrontWingSpan);
fprintf('═══════════════════════════════════════════════════════════\n\n');

[ADP, sizing_out] = Boxwing.B777.Size(ADP);

%% Build final geometry
[BoxGeom, BoxMass] = Boxwing.B777.BuildGeometry(ADP);

%% Print key results
fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║              BASELINE SIZING RESULTS                       ║\n');
fprintf('╠════════════════════════════════════════════════════════════╣\n');
fprintf('║  MTOM          : %7.1f t                                ║\n', ADP.MTOM/1e3);
fprintf('║  OEM           : %7.1f t  (%4.1f%%)                    ║\n', ADP.OEM/1e3, ADP.OEM/ADP.MTOM*100);
fprintf('║  Block Fuel    : %7.1f t  (%4.1f%%)                    ║\n', sizing_out.BlockFuel/1e3, ADP.Mf_Fuel*100);
fprintf('║  Payload       : %7.1f t                                ║\n', ADP.TLAR.Payload/1e3);
fprintf('║  Wing Area     : %7.1f m2                               ║\n', ADP.WingArea);
fprintf('║  Eff. Span     : %7.1f m                                ║\n', ADP.EffectiveSpan);
fprintf('║  Aspect Ratio  : %7.2f                                  ║\n', ADP.AR());
fprintf('║  T/W           : %7.3f                                  ║\n', ADP.ThrustToWeightRatio);
fprintf('║  W/S           : %7.0f N/m2                             ║\n', ADP.WingLoading);
fprintf('╠════════════════════════════════════════════════════════════╣\n');
fprintf('║  CD0           : %7.4f                                  ║\n', ADP.AeroPolar.CD(0));
fprintf('║  CD (CL=0.5)   : %7.4f                                  ║\n', ADP.AeroPolar.CD(0.5));
fprintf('║  L/D cruise    : %7.1f                                  ║\n', ADP.LD_c);
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 2 — GEOMETRY VISUALIZATION
%% ═══════════════════════════════════════════════════════════════════════

cgX  = @(m) sum([m.m] .* cellfun(@(x)x(1),{m.X})) / sum([m.m]);
x_cg = cgX(BoxMass);

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   GEOMETRY PLOT (Figure 1)\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

figure(1); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1200 600]);

Boxwing.cast.draw(BoxGeom, BoxMass);
hold on;
plot(x_cg, 0, 'rx', 'MarkerSize', 20, 'LineWidth', 3);
text(x_cg, -2, sprintf('CG: x=%.1f m', x_cg), ...
     'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');

axis equal; grid on;
xlabel('X — Fuselage axis (m)', 'FontSize', 12);
ylabel('Y — Span (m)', 'FontSize', 12);
title(sprintf('Boxwing Freighter — Top View  |  MTOM=%.0f t  |  Fuel=%.0f t  |  Span=%.0f m', ...
      ADP.MTOM/1e3, sizing_out.BlockFuel/1e3, ADP.EffectiveSpan), 'FontSize', 14);
ylim([-0.55 0.55] * ADP.EffectiveSpan);


%% ═══════════════════════════════════════════════════════════════════════
%  PART 3 — MASS BREAKDOWN
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   MASS BREAKDOWN (Figure 2)\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

figure(2); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 900 700]);

allNames = cellfun(@(x) x, {BoxMass.Name}, 'UniformOutput', false);
allMass  = [BoxMass.m] / 1e3;

colors = repmat([0.3 0.5 0.8], length(allNames), 1);
for i = 1:length(allNames)
    name = allNames{i};
    if contains(name, 'Fuel', 'IgnoreCase', true)
        colors(i,:) = [0.9 0.6 0.1];
    elseif strcmp(name, 'Payload')
        colors(i,:) = [0.9 0.3 0.3];
    elseif contains(name, {'Systems','Avionics','Hydraulics','Electrical','APU', ...
                            'Cargo','Ice','Fire','Tank','Operator','Wiring'})
        colors(i,:) = [0.3 0.7 0.4];
    end
end

bh = barh(allMass, 0.7);
set(bh, 'FaceColor', 'flat');
bh.CData = colors;
set(gca, 'YTick', 1:length(allNames), 'YTickLabel', allNames, 'FontSize', 9);
xlabel('Mass (tonnes)', 'FontSize', 12);
title(sprintf('Boxwing — Component Mass Breakdown  |  OEM=%.0ft  Fuel=%.0ft  Payload=%.0ft', ...
      ADP.OEM/1e3, sizing_out.BlockFuel/1e3, ADP.TLAR.Payload/1e3), 'FontSize', 12);
grid on;
xlim([0 max(allMass)*1.15]);
legend({'Structure','Systems','Fuel','Payload'}, 'Location', 'southeast');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 4 — MISSION ANALYSIS
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   MISSION ANALYSIS\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

[BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, cruise_FL] = ...
    Boxwing.B777.MissionAnalysis(ADP, ADP.TLAR.Range, ADP.MTOM);

fprintf('Design Range:    %.0f NM\n',  ADP.TLAR.Range * SI.Nmile);
fprintf('Trip Fuel:       %.1f t\n',   TripFuel/1e3);
fprintf('Reserve Fuel:    %.1f t\n',   ResFuel/1e3);
fprintf('Block Fuel:      %.1f t\n',   BlockFuel/1e3);
fprintf('Mission Time:    %.1f hr\n',  MissionTime/3600);
fprintf('Cruise FL:       FL%.0f\n',   cruise_FL);
fprintf('\n');


%% =======================================================================
%  PART 5 - CONSTRAINT ANALYSIS (T/W vs W/S diagram)
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   CONSTRAINT ANALYSIS\n');
fprintf('===========================================================\n\n');

figure(4); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 900 650]);
[TW_design, WS_design] = Boxwing.B777.ConstraintAnalysis(ADP, true);
fprintf('Constraint design point:\n');
fprintf('  T/W (SLS) = %.4f\n', TW_design);
fprintf('  W/S       = %.0f N/m2\n\n', WS_design);
saveas(figure(4), 'Boxwing_ConstraintDiagram.png');


%% =======================================================================
%  PART 6 - MULTI-PHASE DRAG POLAR
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   MULTI-PHASE DRAG POLAR\n');
fprintf('===========================================================\n\n');

try
    polars = Boxwing.B777.multiPhasePolar(ADP, BoxGeom);
    fprintf('  Drag polars computed.\n');
    phases = fieldnames(polars);
    for k = 1:length(phases)
        p = polars.(phases{k});
        if isstruct(p) && isfield(p,'CD0')
            fprintf('  %-12s  CD0 = %.4f\n', phases{k}, p.CD0);
        end
    end
    figure(5); clf;
    set(gcf, 'Color', 'w', 'Position', [100 100 800 600]);
    CL_vec = linspace(0, 1.8, 200);
    hold on; grid on;
    phase_colors = lines(length(phases));
    for k = 1:length(phases)
        p = polars.(phases{k});
        if isstruct(p) && isfield(p,'CD') && isa(p.CD,'function_handle')
            CD_vec = arrayfun(p.CD, CL_vec);
            plot(CD_vec, CL_vec, 'LineWidth', 2, 'Color', phase_colors(k,:), ...
                 'DisplayName', phases{k});
        end
    end
    xlabel('C_D','FontSize',12); ylabel('C_L','FontSize',12);
    title('Boxwing -- Multi-Phase Drag Polars','FontSize',14,'FontWeight','bold');
    legend('Location','northwest','FontSize',9);
    saveas(figure(5), 'Boxwing_DragPolars.png');
    fprintf('  Saved: Boxwing_DragPolars.png\n\n');
catch ME
    fprintf('  [SKIP] multiPhasePolar: %s\n\n', ME.message);
    polars = [];
end


%% =======================================================================
%  PART 7 - LIFT DISTRIBUTION
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   LIFT DISTRIBUTION\n');
fprintf('===========================================================\n\n');

try
    figure(6); clf;
    set(gcf, 'Color', 'w', 'Position', [100 100 900 600]);
    dist = Boxwing.B777.liftDistribution(ADP);
    fprintf('  Spanwise lift distribution computed and plotted.\n\n');
    saveas(figure(6), 'Boxwing_LiftDistribution.png');
    fprintf('  Saved: Boxwing_LiftDistribution.png\n\n');
catch ME
    fprintf('  [SKIP] LiftDistribution: %s\n\n', ME.message);
    dist = [];
end


%% =======================================================================
%  PART 8 - STRUCTURAL SIZING (Beam Properties + Cap Sizing)
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   STRUCTURAL SIZING -- BEAM PROPERTIES\n');
fprintf('===========================================================\n\n');

try
    E_CFRP   = 70e9;   % [Pa]  CFRP Young's modulus
    % Use atmosT (temperature-only, faster) to get cruise temperature for
    % thermal expansion reference — full atmos() used elsewhere for rho/a
    T_cruise = Boxwing.cast.atmosT(ADP.TLAR.Alt_cruise);
    fprintf('  Cruise temperature (atmosT): %.1f K  (%.1f °C)\n', T_cruise, T_cruise - 273.15);
    I_front  = 0.08;   % [m^4] front wing box second moment of area
    I_rear   = 0.06;   % [m^4] rear wing box second moment of area
    lift_split_front = ADP.etaLift;
    lift_split_rear  = 1 - ADP.etaLift;

    front_beam = Boxwing.B777.beamproperties(ADP, 'Front Wing', lift_split_front, E_CFRP, I_front);
    rear_beam  = Boxwing.B777.beamproperties(ADP, 'Rear Wing',  lift_split_rear,  E_CFRP, I_rear);

    n_limit    = 2.5;  % limit load factor
    front_beam = front_beam.calcTriangularLoad(n_limit);
    rear_beam  = rear_beam.calcTriangularLoad(n_limit);
    front_beam = front_beam.reactionLoads();
    rear_beam  = rear_beam.reactionLoads();

    fprintf('  Front wing:\n'); front_beam.printSummary();
    fprintf('\n  Rear wing:\n');  rear_beam.printSummary();

    % Cap sizing via structures_calculations
    sigma_allow = 400e6;
    h_box = 0.12 * ADP.TotalLiftingArea / ADP.Span;
    b_cap = 0.40 * h_box;
    npts  = 50;
    [x_f, ~, M_f] = front_beam.diagrams(npts);
    [x_r, ~, M_r] = rear_beam.diagrams(npts);
    [A_cap_front, t_cap_front] = Boxwing.B777.structures_calculations([], M_f, h_box, b_cap, sigma_allow);
    [A_cap_rear,  t_cap_rear ] = Boxwing.B777.structures_calculations([], M_r, h_box, b_cap, sigma_allow);

    fprintf('\n  Cap sizing (CFRP, sigma_allow=%.0f MPa):\n', sigma_allow/1e6);
    fprintf('    Front wing:  max A_cap=%.4f m2   max t_cap=%.4f m\n', max(A_cap_front), max(t_cap_front));
    fprintf('    Rear  wing:  max A_cap=%.4f m2   max t_cap=%.4f m\n\n', max(A_cap_rear), max(t_cap_rear));

    figure(7); clf;
    set(gcf,'Color','w','Position',[100 100 1000 500]);
    tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    nexttile;
    plot(x_f, M_f/1e6,'b-','LineWidth',2); hold on;
    plot(x_r, M_r/1e6,'r-','LineWidth',2);
    grid on; xlabel('Span [m]'); ylabel('Bending Moment [MN.m]');
    title('Wing Bending Moment','FontWeight','bold');
    legend({'Front','Rear'},'Location','northeast');
    nexttile;
    plot(x_f, A_cap_front*1e4,'b-','LineWidth',2); hold on;
    plot(x_r, A_cap_rear*1e4, 'r-','LineWidth',2);
    grid on; xlabel('Span [m]'); ylabel('Cap Area [cm^2]');
    title('Spar Cap Area','FontWeight','bold');
    legend({'Front','Rear'},'Location','northeast');
    saveas(figure(7), 'Boxwing_StructuralSizing.png');
    fprintf('  Saved: Boxwing_StructuralSizing.png\n\n');
catch ME
    fprintf('  [SKIP] Structural sizing: %s\n\n', ME.message);
    front_beam = []; rear_beam = [];
end


%% =======================================================================
%  PART 9 - DIRECT OPERATING COST (DOC)
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   DIRECT OPERATING COST (DOC)\n');
fprintf('===========================================================\n\n');

fleet_size = 10;  SAF_ratio = 0.0;  T_max_K = 1850;
[DOC_total, docBreakdown, ~, total_init, ~, ~] = Boxwing.script.DOC( ...
    ADP.MTOM/1e3, ADP.OEM, sizing_out.BlockFuel, ...
    fleet_size, SAF_ratio, ADP.TLAR.M_c, T_max_K);
fprintf('  Fleet=%d  SAF=%.0f%%\n\n  DOC Breakdown:\n', fleet_size, SAF_ratio*100);
dfields = fieldnames(docBreakdown);
for k = 1:length(dfields)
    if ~strcmp(dfields{k},'total')
        fprintf('    %-18s  $ %14.0f\n', dfields{k}, docBreakdown.(dfields{k}));
    end
end
fprintf('    ------------------------------------------\n');
fprintf('    %-18s  $ %14.0f\n', 'TOTAL DOC', docBreakdown.total);
fprintf('\n  Total programme cost : $ %.2f B\n\n', total_init/1e9);



%% ═══════════════════════════════════════════════════════════════════════
%  PART 10 — PAYLOAD-RANGE: UltraFan vs GE90 COMPARISON
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   PAYLOAD-RANGE: UltraFan vs GE90-115B\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

%  Breguet-based payload-range sweep for both engine options.
%  Aircraft geometry fixed (MTOM, OEM, WingArea, LD all from sizing).
%  Fuel capacity capped at 165t (5-tank geometry limit from analysis).
%
%  Convention:
%    Point A — max payload (140t),  fuel fills remaining MTOM margin
%    Point B — payload reduced, full fuel tanks (165t cap)
%    Point C — ferry range, zero payload, full tanks

g_pa        = 9.81;
V_cr_pa     = ADP.TLAR.M_c * 295.0;      % cruise TAS [m/s] at FL390
LD_cr_pa    = ADP.LD_c;                   % L/D from sizing
fuel_cap_pa = 165000;                     % [kg] total tank capacity
OEM_pa      = ADP.OEM;
MTOM_pa     = ADP.MTOM;
Payload_max = ADP.TLAR.Payload;

% ── TSFC functions ───────────────────────────────────────────────────
alt_cr_pa = ADP.TLAR.Alt_cruise;

tsfc_ge90_fn = @(M, h) max(0.0158e-3 ...
    * ((288.15-0.0065*min(h,11000))/288.15)^(-0.5) ...
    * (0.45 + 0.54*M), 1e-5);

tsfc_uf_fn = @(M, h) ultraFanTSFC(M, h);

function tsfc = ultraFanTSFC(Mach, alt_m)
    BPR    = 15;
    SFC_TO = 18 * exp(-0.12*BPR) * 1e-6;
    SFC_cr = 22 * exp(-0.05*BPR) * 1e-6;
    T      = max(288.15 - 0.0065*min(alt_m,11000), 216.65);
    sr     = sqrt(T/288.15);
    SFC_B  = (SFC_cr/sr - SFC_TO) / 0.82;
    tsfc   = max((SFC_TO + SFC_B*Mach)*sr, 1e-6);
end

% ── Breguet range ────────────────────────────────────────────────────
breguet_R = @(W_start, trip_fuel, tsfc) ...
    (V_cr_pa * LD_cr_pa) / (g_pa * tsfc) ...
    * log(max(W_start ./ max(W_start - trip_fuel, 1), 1+1e-9));

% ── Payload sweep ────────────────────────────────────────────────────
payloads_pa = linspace(Payload_max, 0, 60);   % [kg]

range_ge90 = zeros(size(payloads_pa));
range_uf   = zeros(size(payloads_pa));
fuels_pa   = zeros(size(payloads_pa));

TSFC_ge = tsfc_ge90_fn(ADP.TLAR.M_c, alt_cr_pa);
TSFC_uf = tsfc_uf_fn(ADP.TLAR.M_c,   alt_cr_pa);

for k = 1:length(payloads_pa)
    pld = payloads_pa(k);
    fuel_avail  = min(MTOM_pa - OEM_pa - pld, fuel_cap_pa);
    fuel_avail  = max(fuel_avail, 0);
    trip_fuel   = fuel_avail * 0.94;   % reserve = 6% of block
    W_start     = OEM_pa + pld + fuel_avail;
    fuels_pa(k) = fuel_avail;

    range_ge90(k) = breguet_R(W_start, trip_fuel, TSFC_ge) / 1852;   % NM
    range_uf(k)   = breguet_R(W_start, trip_fuel, TSFC_uf) / 1852;   % NM
end

% ── Key design points ────────────────────────────────────────────────
[~, idx_maxpld] = min(abs(payloads_pa - Payload_max));
[~, idx_halfpld]= min(abs(payloads_pa - Payload_max/2));
[~, idx_ferry]  = min(abs(payloads_pa));

fprintf('  Payload-Range comparison (Breguet, LD=%.1f):\n', LD_cr_pa);
fprintf('  %-12s  %10s  %10s  %8s\n', 'Payload', 'GE90 [NM]', 'UF [NM]', 'Gain');
fprintf('  %s\n', repmat('-',1,48));
for pld_t = [140, 100, 70, 0]
    [~,ki] = min(abs(payloads_pa - pld_t*1000));
    fprintf('  %-12s  %10.0f  %10.0f  %+7.0f NM (+%.0f%%)\n', ...
        sprintf('%dt',pld_t), range_ge90(ki), range_uf(ki), ...
        range_uf(ki)-range_ge90(ki), ...
        (range_uf(ki)-range_ge90(ki))/max(range_ge90(ki),1)*100);
end
fprintf('\n');

% ── Figure 14 — Payload-Range ────────────────────────────────────────
figure(14); clf;
set(gcf, 'Color','w', 'Position',[100 100 1000 620]);

% Shaded region between curves (UltraFan advantage)
x_fill = [range_ge90, fliplr(range_uf)];
y_fill = [payloads_pa/1e3, fliplr(payloads_pa/1e3)];
fill(x_fill, y_fill, [0.85 0.95 0.85], 'EdgeColor','none', 'FaceAlpha',0.5, ...
     'DisplayName','UltraFan range gain');
hold on; grid on; box on;

% Main curves
plot(range_ge90, payloads_pa/1e3, 'b-',  'LineWidth', 2.5, ...
     'DisplayName', 'GE90-115B  (BPR 8.7, 1995 tech)');
plot(range_uf,   payloads_pa/1e3, 'r-',  'LineWidth', 2.5, ...
     'DisplayName', 'UltraFan   (BPR 15,  ~2030 tech)');

% Design point markers
plot(range_ge90(idx_maxpld), payloads_pa(idx_maxpld)/1e3, 'bs', ...
     'MarkerSize',10, 'MarkerFaceColor','b', 'HandleVisibility','off');
plot(range_uf(idx_maxpld),   payloads_pa(idx_maxpld)/1e3, 'rs', ...
     'MarkerSize',10, 'MarkerFaceColor','r', 'HandleVisibility','off');

% Annotations at max payload
text(range_ge90(idx_maxpld)+80, payloads_pa(idx_maxpld)/1e3+1, ...
     sprintf('%.0f NM', range_ge90(idx_maxpld)), ...
     'Color','b', 'FontSize',9, 'FontWeight','bold');
text(range_uf(idx_maxpld)+80,   payloads_pa(idx_maxpld)/1e3-3, ...
     sprintf('%.0f NM', range_uf(idx_maxpld)), ...
     'Color','r', 'FontSize',9, 'FontWeight','bold');

% Design range reference line
xline(ADP.TLAR.Range * SI.Nmile, 'k--', 'LineWidth', 1.5, ...
      'DisplayName', sprintf('Design range (%.0f NM)', ADP.TLAR.Range*SI.Nmile));

% Gain annotation box
mid_idx = round(length(payloads_pa)/2);
x_mid   = (range_ge90(mid_idx) + range_uf(mid_idx)) / 2;
y_mid   = payloads_pa(mid_idx)/1e3;
gain_pct = (range_uf(mid_idx) - range_ge90(mid_idx)) / range_ge90(mid_idx) * 100;
text(x_mid, y_mid, sprintf('+%.0f%% range\nfrom UltraFan', gain_pct), ...
     'HorizontalAlignment','center', 'FontSize',10, ...
     'Color',[0.1 0.5 0.1], 'FontWeight','bold', 'BackgroundColor','w');

% Fuel capacity line — max range at zero payload
plot([range_ge90(end) range_ge90(end)], [0 5], 'b:', 'LineWidth',1.2, 'HandleVisibility','off');
plot([range_uf(end)   range_uf(end)],   [0 5], 'r:', 'LineWidth',1.2, 'HandleVisibility','off');
text(range_ge90(end), 7, sprintf('Ferry\n%.0f NM', range_ge90(end)), ...
     'Color','b', 'FontSize',8, 'HorizontalAlignment','center');
text(range_uf(end),   7, sprintf('Ferry\n%.0f NM', range_uf(end)), ...
     'Color','r', 'FontSize',8, 'HorizontalAlignment','center');

xlabel('Range  [NM]',           'FontSize',13, 'FontWeight','bold');
ylabel('Payload  [tonnes]',     'FontSize',13, 'FontWeight','bold');
title(sprintf(['Boxwing Freighter — Payload-Range Diagram\n' ...
               'MTOM=%.0ft  |  OEM=%.0ft  |  Max Fuel=%.0ft  |  L/D=%.1f'], ...
    MTOM_pa/1e3, OEM_pa/1e3, fuel_cap_pa/1e3, LD_cr_pa), ...
    'FontSize',12, 'FontWeight','bold');

legend('Location','northeast', 'FontSize',10);
xlim([0, max(range_uf)*1.05]);
ylim([0, Payload_max/1e3 * 1.15]);
xticks(0:1000:ceil(max(range_uf)/1000)*1000);

saveas(figure(14), 'Boxwing_PayloadRange.png');
fprintf('  Saved: Boxwing_PayloadRange.png\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 11 — TRADE STUDY: SPAN vs MTOM & FUEL
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   TRADE STUDY: Wing Span Sweep\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

Spans = 40:2:60;   % [m] front-wing span range

mtoms = zeros(size(Spans));
fuels = zeros(size(Spans));
oems  = zeros(size(Spans));
areas = zeros(size(Spans));
ARs   = zeros(size(Spans));

fprintf('Testing %d span configurations from %.0f m to %.0f m...\n', ...
        length(Spans), min(Spans), max(Spans));

for i = 1:length(Spans)
    fprintf('  [%2d/%2d] Span = %.0f m ... ', i, length(Spans), Spans(i));

    % Deep-copy baseline ADP for each iteration (ADP is a handle class)
    ADPi = freshADP(Spans(i), SI);

    try
        ADPi = Boxwing.B777.Size(ADPi, false);   % silent sizing

        mtoms(i) = ADPi.MTOM;
        fuels(i) = ADPi.Mf_Fuel * ADPi.MTOM;
        oems(i)  = ADPi.OEM;
        areas(i) = ADPi.WingArea;
        ARs(i)   = ADPi.AR();

        fprintf('MTOM=%.0f t, Fuel=%.0f t, AR=%.2f\n', ...
                mtoms(i)/1e3, fuels(i)/1e3, ARs(i));
    catch ME
        fprintf('FAILED: %s\n', ME.message);
        mtoms(i) = NaN; fuels(i) = NaN; oems(i) = NaN;
        areas(i) = NaN; ARs(i)   = NaN;
    end
end

fprintf('\nTrade study complete.\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  Plot trade study results
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   TRADE STUDY PLOTS (Figure 3)\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

figure(3); clf;
set(gcf, 'Color', 'w', 'Position', [100 100 1200 900]);
tt = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(1);
plot(Spans, mtoms/1e3, '-s', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.2 0.4 0.8], 'MarkerFaceColor', [0.4 0.6 1.0]);
grid on; xlabel('Span [m]', 'FontSize', 11); ylabel('MTOM [t]', 'FontSize', 11);
title('MTOM vs Span', 'FontSize', 12, 'FontWeight', 'bold');

nexttile(2);
plot(Spans, fuels/1e3, '-o', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.9 0.5 0.1], 'MarkerFaceColor', [1.0 0.7 0.3]);
grid on; xlabel('Span [m]', 'FontSize', 11); ylabel('Block Fuel [t]', 'FontSize', 11);
title('Block Fuel vs Span', 'FontSize', 12, 'FontWeight', 'bold');

nexttile(3);
plot(Spans, oems/1e3, '-d', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.3 0.7 0.4], 'MarkerFaceColor', [0.5 0.9 0.6]);
grid on; xlabel('Span [m]', 'FontSize', 11); ylabel('OEM [t]', 'FontSize', 11);
title('OEM vs Span', 'FontSize', 12, 'FontWeight', 'bold');

nexttile(4);
plot(Spans, areas, '-^', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.7 0.3 0.7], 'MarkerFaceColor', [0.9 0.5 0.9]);
grid on; xlabel('Span [m]', 'FontSize', 11); ylabel('Wing Area [m²]', 'FontSize', 11);
title('Wing Area vs Span', 'FontSize', 12, 'FontWeight', 'bold');

nexttile(5);
plot(Spans, ARs, '-v', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.8 0.2 0.2], 'MarkerFaceColor', [1.0 0.4 0.4]);
grid on; xlabel('Span [m]', 'FontSize', 11); ylabel('Aspect Ratio [-]', 'FontSize', 11);
title('Aspect Ratio vs Span', 'FontSize', 12, 'FontWeight', 'bold');

nexttile(6);
plot(Spans, (fuels./mtoms)*100, '-p', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.5 0.5 0.5], 'MarkerFaceColor', [0.7 0.7 0.7]);
grid on; xlabel('Span [m]', 'FontSize', 11); ylabel('Fuel Fraction [% MTOM]', 'FontSize', 11);
title('Fuel Fraction vs Span', 'FontSize', 12, 'FontWeight', 'bold');

title(tt, 'Boxwing Freighter — Wing Span Trade Study', ...
      'FontSize', 14, 'FontWeight', 'bold');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 12 — OPTIMUM SPAN IDENTIFICATION
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   OPTIMUM SPAN ANALYSIS\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

[mtom_min, idx_mtom] = min(mtoms);
span_opt_mtom = Spans(idx_mtom);

[fuel_min, idx_fuel] = min(fuels);
span_opt_fuel = Spans(idx_fuel);

fprintf('Minimum MTOM:       %.1f t  at  span = %.0f m\n', mtom_min/1e3, span_opt_mtom);
fprintf('Minimum Block Fuel: %.1f t  at  span = %.0f m\n', fuel_min/1e3, span_opt_fuel);
fprintf('\n');

figure(3);
nexttile(1); hold on;
plot(span_opt_mtom, mtom_min/1e3, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
text(span_opt_mtom, mtom_min/1e3, sprintf(' <- Min MTOM\n   (%.0f m)', span_opt_mtom), ...
     'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');

nexttile(2); hold on;
plot(span_opt_fuel, fuel_min/1e3, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
text(span_opt_fuel, fuel_min/1e3, sprintf(' <- Min Fuel\n   (%.0f m)', span_opt_fuel), ...
     'FontSize', 9, 'Color', 'r', 'FontWeight', 'bold');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 13 — SUMMARY TABLE
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   SUMMARY TABLE\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

fprintf('%-25s | %-12s | %-12s | %-12s\n', 'Parameter', 'Baseline', 'Min MTOM', 'Min Fuel');
fprintf('%s\n', repmat('-', 1, 70));
fprintf('%-25s | %10.1f m | %10.1f m | %10.1f m\n', 'Span',       ADP.EffectiveSpan,        span_opt_mtom,          span_opt_fuel);
fprintf('%-25s | %10.1f t | %10.1f t | %10.1f t\n', 'MTOM',       ADP.MTOM/1e3,             mtom_min/1e3,           mtoms(idx_fuel)/1e3);
fprintf('%-25s | %10.1f t | %10.1f t | %10.1f t\n', 'Block Fuel', sizing_out.BlockFuel/1e3, fuels(idx_mtom)/1e3,    fuel_min/1e3);
fprintf('%-25s | %10.1f t | %10.1f t | %10.1f t\n', 'OEM',        ADP.OEM/1e3,              oems(idx_mtom)/1e3,     oems(idx_fuel)/1e3);
fprintf('%-25s | %10.1f m2| %10.1f m2| %10.1f m2\n','Wing Area',  ADP.WingArea,             areas(idx_mtom),        areas(idx_fuel));
fprintf('%-25s | %10.2f   | %10.2f   | %10.2f\n',   'Aspect Ratio',ADP.AR(),                ARs(idx_mtom),          ARs(idx_fuel));
fprintf('%s\n\n', repmat('-', 1, 70));


%% ═══════════════════════════════════════════════════════════════════════
%  PART 14 — EXPORT RESULTS
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   EXPORTING RESULTS\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

saveas(figure(1), 'Boxwing_Geometry.png');
saveas(figure(2), 'Boxwing_MassBreakdown.png');
saveas(figure(3), 'Boxwing_TradeStudy.png');

fprintf('Figures saved:\n');
fprintf('  Boxwing_Geometry.png\n');
fprintf('  Boxwing_MassBreakdown.png\n');
fprintf('  Boxwing_TradeStudy.png\n\n');

T = table(Spans', mtoms'/1e3, fuels'/1e3, oems'/1e3, areas', ARs', ...
          'VariableNames', {'Span_m','MTOM_t','BlockFuel_t','OEM_t','WingArea_m2','AspectRatio'});
writetable(T, 'Boxwing_TradeStudy.csv');
fprintf('Trade study data saved: Boxwing_TradeStudy.csv\n\n');


%% DONE

fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                  ANALYSIS COMPLETE                         ║\n');
fprintf('╠════════════════════════════════════════════════════════════╣\n');
fprintf('║  All results displayed in Figures 1-3 and saved to disk.   ║\n');
fprintf('║  Trade study data exported to Boxwing_TradeStudy.csv       ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL HELPERS
%% ═══════════════════════════════════════════════════════════════════════

function ADPi = freshADP(frontSpan, SI)
%FRESHADP  Create a new ADP object configured for a given front-wing span.
%  ADP is a handle class, so assignment copies the reference, not the data.
%  This function always constructs a brand-new object to avoid aliasing.
    ADPi           = Boxwing.B777.ADP();
    ADPi.TLAR      = Boxwing.cast.TLAR.Boxwing();
    ADPi.Engine    = GE90Engine();   % engine baseline (scaling done in engine.m)

    ADPi.CockpitLength = 6.5;
    ADPi.CabinRadius   = 2.93;
    ADPi.CabinLength   = 70.0 - ADPi.CockpitLength - ADPi.CabinRadius*2*1.48;

    L_f = ADPi.CockpitLength + ADPi.CabinLength + ADPi.CabinRadius*1.48;
    ADPi.FrontWingPos  = 0.40 * L_f;
    ADPi.RearWingPos   = 0.90 * L_f;

    ADPi.V_HT = 0;
    ADPi.V_VT = 0.05;

    rearSpan = frontSpan - 10;   % keep rear 10 m shorter than front
    ADPi.FrontWingSpan  = frontSpan;
    ADPi.RearWingSpan   = rearSpan;
    ADPi.ConnectorHeight = 8;
    ADPi.updateDerivedProps();

    ADPi.MTOM    = 3.0 * ADPi.TLAR.Payload;
    ADPi.Mf_Fuel = 0.28;
    ADPi.Mf_res  = 0.04;
    ADPi.Mf_Ldg  = 0.75;
    ADPi.Mf_TOC  = 0.98;

    Boxwing.B777.UpdateAero(ADPi);
end


function eng = GE90Engine()
%GE90ENGINE  Engine data object based on GE90-115B.
%  Baseline geometry and TSFC model.
%  Rubber scaling (Rubberise) is now handled inline inside engine.m,
%  so this object only needs to carry the reference values + TSFC.
%
%  Fields used by the codebase:
%    .T_ref / .D_ref / .L_ref / .M_ref  — baseline for inline scaling
%    .Diameter / .Length / .Mass        — current (updated by engine.m each iter)
%    .TSFC(Mach, alt_m)                 — used by MissionAnalysis.m

    eng.T_ref    = 513e3;   % [N]  GE90-115B SLS thrust per engine
    eng.D_ref    = 3.124;   % [m]  fan diameter
    eng.L_ref    = 7.29;    % [m]  overall length
    eng.M_ref    = 8618;    % [kg] dry mass per engine

    % Initial geometry = baseline (engine.m will update each sizing iteration)
    eng.Diameter = eng.D_ref;
    eng.Length   = eng.L_ref;
    eng.Mass     = eng.M_ref;

    eng.TSFC = @(Mach, alt_m) rubberTSFC(Mach, alt_m);
end

function tsfc = rubberTSFC(Mach, alt_m)
%RUBBERTSFC  Mattingly correlation, high-BPR turbofan (BPR~9, GE90/GEnX).
    TSFC_SLS = 0.0158e-3;   % [1/s]
    theta    = (288.15 - 0.0065*min(alt_m, 11000)) / 288.15;
    tsfc     = max(TSFC_SLS * theta^(-0.5) * (0.45 + 0.54*Mach), 1e-5);
end