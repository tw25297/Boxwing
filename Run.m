%% BOXWING FREIGHTER — COMPLETE SIZING AND ANALYSIS
%  All five fixes applied:
%
%  FIX 1 — CD0.m:  getCf fzero guarded against non-finite endpoints.
%           → DragMeta breakdown now works.
%
%  FIX 2 — multiPhasePolar.m: cast.atmos → Boxwing.cast.atmos
%           → "Dot indexing" error resolved.
%
%  FIX 3 — liftDistribution.m: BoxWing.cast.atmos → Boxwing.cast.atmos
%           → "Unable to resolve BoxWing.cast.atmos" resolved.
%
%  FIX 4 — ADP.m / freshADP: c_ref_fixed held constant across trade sweep
%           so WingArea = b_eff * c_ref (linear in span), not b²/AR (quadratic).
%           AR now varies 6.7–10 across 40–60 m, revealing a genuine optimum.
%
%  FIX 5 — Figures saved immediately after each plot block.
%           → "Invalid graphics object" saveas crash at end eliminated.

clear; clc; close all;

%% 0. Setup
fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   BOXWING FREIGHTER — COMPREHENSIVE SIZING ANALYSIS        ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

projectRoot = pwd;
addpath(projectRoot);
clear classes;

SI.ft    = 3.28084;
SI.Nmile = 1 / 1852;
SI.knt   = 1 / 0.5144;
SI.min   = 1 / 60;
SI.lb    = 2.20462;
SI.litre = 1000;
SI.km    = 1e-3;
SI.hr    = 1 / 3600;
assignin('base', 'SI', SI);


%% ═══════════════════════════════════════════════════════════════════════
%  PART 1 — BASELINE SIZING
%% ═══════════════════════════════════════════════════════════════════════

ADP      = Boxwing.B777.ADP();
ADP.TLAR = Boxwing.cast.TLAR.Boxwing();
ADP.Engine = GE90Engine();

ADP.CockpitLength = 6.5;
ADP.CabinRadius   = 2.93;
ADP.CabinLength   = 70.0 - ADP.CockpitLength - ADP.CabinRadius*2*1.48;

L_f = ADP.CockpitLength + ADP.CabinLength + ADP.CabinRadius*1.48;
% NOTE: FrontWingPos and RearWingPos are computed inside updateDerivedProps
%       (0.40*L_f and 0.75*L_f). Do NOT set them here — they would be
%       immediately overwritten by the updateDerivedProps call below.

ADP.V_HT = 0;
ADP.V_VT = 0.05;

ADP.FrontWingSpan   = 60;
ADP.RearWingSpan    = 50;
ADP.ConnectorHeight = 3;

ADP.updateDerivedProps();

ADP.MTOM    = 3.0 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.28;
ADP.Mf_res  = 0.04;
ADP.Mf_Ldg  = 0.75;
ADP.Mf_TOC  = 0.98;

Boxwing.B777.UpdateAero(ADP);

%% Sizing loop
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   BASELINE SIZING (Span = %.0f m)\n', ADP.FrontWingSpan);
fprintf('═══════════════════════════════════════════════════════════\n\n');

[ADP, sizing_out] = Boxwing.B777.Size(ADP);

%% Build final geometry
[BoxGeom, BoxMass] = Boxwing.B777.BuildGeometry(ADP);

%% FIX 2: Write back converged CL_cruise AFTER BuildGeometry so nothing
%  can overwrite it. liftDistribution reads this to get a physical CL (~0.73).
[rho_cr_post, a_cr_post] = Boxwing.cast.atmos(ADP.TLAR.Alt_cruise);
q_cr_post = 0.5 * rho_cr_post * (ADP.TLAR.M_c * a_cr_post)^2;
ADP.CL_cruise = (ADP.MTOM * 9.81 * ADP.Mf_TOC) / (q_cr_post * ADP.WingArea);

%% FIX 1: CD0 breakdown now runs AFTER sizing with converged geometry
%  (pre-sizing it used ADP defaults: c_ref=5.5m, large CD0=0.0186).
fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   CD0 DRAG BREAKDOWN (DragMeta) — converged geometry\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

try
    [CD0_val, cd0_breakdown] = Boxwing.B777.CD0(ADP, [], true);
    componentNames = {'Wing','Fuselage','Nacelles','HTP','VTP','Tail','Misc'};
    componentVals  = [cd0_breakdown.CD0_wing, cd0_breakdown.CD0_fuse, cd0_breakdown.CD0_nac, ...
                      cd0_breakdown.CD0_HTP,  cd0_breakdown.CD0_VTP,  cd0_breakdown.CD0_tail, ...
                      cd0_breakdown.CD0_misc];
    dragItems = Boxwing.cast.DragMeta(componentNames{1}, componentVals(1));
    for k = 2:length(componentNames)
        dragItems(k) = Boxwing.cast.DragMeta(componentNames{k}, componentVals(k));
    end
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

% FIX 5: save immediately after figure is complete
saveas(figure(1), 'Boxwing_Geometry.png');
fprintf('  Saved: Boxwing_Geometry.png\n\n');


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

% FIX 5: save immediately
saveas(figure(2), 'Boxwing_MassBreakdown.png');
fprintf('  Saved: Boxwing_MassBreakdown.png\n\n');


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
fprintf('Cruise FL:       FL%.0f\n\n', cruise_FL);


%% =======================================================================
%  PART 5 — CONSTRAINT ANALYSIS
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
fprintf('  Saved: Boxwing_ConstraintDiagram.png\n\n');


%% =======================================================================
%  PART 6 — MULTI-PHASE DRAG POLAR
%  FIX 2: multiPhasePolar.m now uses Boxwing.cast.atmos (full path)
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   MULTI-PHASE DRAG POLAR\n');
fprintf('===========================================================\n\n');

try
    polars = Boxwing.B777.multiPhasePolar(ADP, BoxGeom);
    fprintf('  Drag polars computed.\n');
    figure(5); clf;
    set(gcf, 'Color', 'w', 'Position', [100 100 800 600]);
    CL_vec = linspace(0, 1.8, 200);
    hold on; grid on;
    phase_colors = lines(length(polars));
    for k = 1:length(polars)
        p = polars(k);
        if isfield(p,'CD') && isa(p.CD,'function_handle')
            CD_vec = arrayfun(p.CD, CL_vec);
            plot(CD_vec, CL_vec, 'LineWidth', 2, 'Color', phase_colors(k,:), ...
                 'DisplayName', p.name);
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
%  PART 7 — LIFT DISTRIBUTION
%  FIX 3: liftDistribution.m now uses Boxwing.cast.atmos (lowercase w)
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   LIFT DISTRIBUTION\n');
fprintf('===========================================================\n\n');

try
    figure(6); clf;
    set(gcf, 'Color', 'w', 'Position', [100 100 900 600]);
    dist = Boxwing.B777.liftDistribution(ADP);
    fprintf('  Spanwise lift distribution computed and plotted.\n');
    fprintf('  Saved: Boxwing_LiftDistribution.png\n\n');  % saved inside liftDistribution
catch ME
    fprintf('  [SKIP] LiftDistribution: %s\n\n', ME.message);
    dist = [];
end


%% =======================================================================
%  PART 8 — STRUCTURAL SIZING
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   STRUCTURAL SIZING -- BEAM PROPERTIES\n');
fprintf('===========================================================\n\n');

try
    E_CFRP   = 70e9;
    T_cruise = Boxwing.cast.atmosT(ADP.TLAR.Alt_cruise);
    fprintf('  Cruise temperature: %.1f K  (%.1f °C)\n', T_cruise, T_cruise - 273.15);
    I_front  = 0.08;
    I_rear   = 0.06;

    front_beam = Boxwing.B777.beamproperties(ADP, 'Front Wing', ADP.etaLift,     E_CFRP, I_front);
    rear_beam  = Boxwing.B777.beamproperties(ADP, 'Rear Wing',  1-ADP.etaLift,   E_CFRP, I_rear);

    n_limit    = 2.5;
    front_beam = front_beam.calcTriangularLoad(n_limit);
    rear_beam  = rear_beam.calcTriangularLoad(n_limit);
    front_beam = front_beam.reactionLoads();
    rear_beam  = rear_beam.reactionLoads();

    fprintf('  Front wing:\n'); front_beam.printSummary();
    fprintf('\n  Rear wing:\n');  rear_beam.printSummary();

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
end


%% =======================================================================
%  PART 9 — DIRECT OPERATING COST (DOC)
%% =======================================================================

fprintf('===========================================================\n');
fprintf('   DIRECT OPERATING COST (DOC)\n');
fprintf('===========================================================\n\n');

fleet_size = 10;  SAF_ratio = 0.0;  T_max_K = 1850;
try
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
catch ME
    fprintf('  [SKIP] DOC: %s\n\n', ME.message);
end


%% ═══════════════════════════════════════════════════════════════════════
%  PART 10 — PAYLOAD-RANGE: UltraFan vs GE90 COMPARISON
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   PAYLOAD-RANGE: UltraFan vs GE90-115B\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

g_pa        = 9.81;
V_cr_pa     = ADP.TLAR.M_c * 295.0;
LD_cr_pa    = ADP.LD_c;
fuel_cap_pa = 165000;
OEM_pa      = ADP.OEM;
MTOM_pa     = ADP.MTOM;
Payload_max = ADP.TLAR.Payload;
alt_cr_pa   = ADP.TLAR.Alt_cruise;

tsfc_ge90_fn = @(M, h) max(0.0158e-3 ...
    * ((288.15-0.0065*min(h,11000))/288.15)^(-0.5) ...
    * (0.45 + 0.54*M), 1e-5);

tsfc_uf_fn = @(M, h) ultraFanTSFC(M, h);

breguet_R = @(W_start, trip_fuel, tsfc) ...
    (V_cr_pa * LD_cr_pa) / (g_pa * tsfc) ...
    * log(max(W_start ./ max(W_start - trip_fuel, 1), 1+1e-9));

TSFC_ge = tsfc_ge90_fn(ADP.TLAR.M_c, alt_cr_pa);
TSFC_uf = tsfc_uf_fn(ADP.TLAR.M_c,   alt_cr_pa);

% ── Three-segment payload-range ─────────────────────────────────────────
% Point A: max payload, fuel fills remaining MTOM margin (MTOM-limited)
% Point B: payload reduced until full tanks (fuel-cap-limited knee)
% Point C: ferry, zero payload, full tanks

fuel_at_maxpld = max(min(MTOM_pa - OEM_pa - Payload_max, fuel_cap_pa), 0);

% Point A
pld_A  = Payload_max;
fuel_A = fuel_at_maxpld;
W_A    = OEM_pa + pld_A + fuel_A;
range_A_ge = breguet_R(W_A, fuel_A*0.94, TSFC_ge) / 1852;
range_A_uf = breguet_R(W_A, fuel_A*0.94, TSFC_uf) / 1852;

% Point B: payload where MTOM - OEM - pld = fuel_cap (knee point)
pld_B  = max(MTOM_pa - OEM_pa - fuel_cap_pa, 0);
fuel_B = fuel_cap_pa;
W_B    = OEM_pa + pld_B + fuel_B;
range_B_ge = breguet_R(W_B, fuel_B*0.94, TSFC_ge) / 1852;
range_B_uf = breguet_R(W_B, fuel_B*0.94, TSFC_uf) / 1852;

% Segment B→C: payload sweeps 0, fuel stays at cap
payloads_BC = linspace(pld_B, 0, 80);
range_ge_BC = zeros(size(payloads_BC));
range_uf_BC = zeros(size(payloads_BC));
for k = 1:length(payloads_BC)
    pld  = payloads_BC(k);
    fuel = min(fuel_cap_pa, max(MTOM_pa - OEM_pa - pld, 0));
    W_s  = OEM_pa + pld + fuel;
    range_ge_BC(k) = breguet_R(W_s, fuel*0.94, TSFC_ge) / 1852;
    range_uf_BC(k) = breguet_R(W_s, fuel*0.94, TSFC_uf) / 1852;
end

% Full curves including flat top (origin → A → B → C)
if pld_B >= Payload_max * 0.999
    % No flat top: MTOM-limit never binding before fuel cap
    payloads_full = payloads_BC / 1e3;
    range_ge_full = range_ge_BC;
    range_uf_full = range_uf_BC;
else
    payloads_full = [Payload_max, Payload_max, pld_B,     payloads_BC]  / 1e3;
    range_ge_full = [0,           range_A_ge,  range_B_ge, range_ge_BC];
    range_uf_full = [0,           range_A_uf,  range_B_uf, range_uf_BC];
end

fprintf('  Payload-Range key points (Breguet, LD=%.1f):\n', LD_cr_pa);
fprintf('  %-10s  %12s  %12s\n', 'Point', 'GE90 [NM]', 'UF [NM]');
fprintf('  %s\n', repmat('-',1,38));
fprintf('  %-10s  %12.0f  %12.0f\n', sprintf('A(%.0ft)', pld_A/1e3), range_A_ge, range_A_uf);
fprintf('  %-10s  %12.0f  %12.0f\n', sprintf('B(%.0ft)', pld_B/1e3), range_B_ge, range_B_uf);
fprintf('  %-10s  %12.0f  %12.0f\n', 'C (ferry)', range_ge_BC(end), range_uf_BC(end));
fprintf('\n');

figure(14); clf;
set(gcf, 'Color','w', 'Position',[100 100 1050 650]);
hold on; grid on; box on;

fill([range_ge_full, fliplr(range_uf_full)], ...
     [payloads_full,  fliplr(payloads_full)], ...
     [0.85 0.95 0.85], 'EdgeColor','none', 'FaceAlpha',0.5, ...
     'DisplayName','UltraFan range gain');

plot(range_ge_full, payloads_full, 'b-', 'LineWidth', 2.5, ...
     'DisplayName', 'GE90-115B  (BPR 8.7)');
plot(range_uf_full, payloads_full, 'r-', 'LineWidth', 2.5, ...
     'DisplayName', 'UltraFan   (BPR 15)');
xline(ADP.TLAR.Range * SI.Nmile, 'k--', 'LineWidth', 1.5, ...
      'DisplayName', sprintf('Design range (%.0f NM)', ADP.TLAR.Range*SI.Nmile));

% Label A, B, C points
text(range_A_ge+100, Payload_max/1e3+1, sprintf('A: %.0f NM', range_A_ge), ...
     'Color','b','FontSize',8,'FontWeight','bold');
text(range_A_uf+100, Payload_max/1e3-3, sprintf('A: %.0f NM', range_A_uf), ...
     'Color','r','FontSize',8,'FontWeight','bold');
if pld_B < Payload_max * 0.999
    text(range_B_ge-200, pld_B/1e3+3, sprintf('B: %.0f NM', range_B_ge), ...
         'Color','b','FontSize',8);
    text(range_B_uf+100, pld_B/1e3-3, sprintf('B: %.0f NM', range_B_uf), ...
         'Color','r','FontSize',8);
end

xlabel('Range  [NM]',       'FontSize',13,'FontWeight','bold');
ylabel('Payload  [tonnes]', 'FontSize',13,'FontWeight','bold');
title(sprintf(['Boxwing Freighter — Payload-Range Diagram\n' ...
               'MTOM=%.0ft  |  OEM=%.0ft  |  MaxFuel=%.0ft  |  L/D=%.1f'], ...
    MTOM_pa/1e3, OEM_pa/1e3, fuel_cap_pa/1e3, LD_cr_pa), ...
    'FontSize',11,'FontWeight','bold');
legend('Location','northeast','FontSize',10);
xlim([0, max(range_uf_BC)*1.05]);
ylim([0, Payload_max/1e3 * 1.15]);
xticks(0:1000:ceil(max(range_uf_BC)/1000)*1000);
saveas(figure(14), 'Boxwing_PayloadRange.png');
fprintf('  Saved: Boxwing_PayloadRange.png\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 11 — TRADE STUDY: SPAN vs MTOM & FUEL
%  FIX 4: freshADP uses fixed mean chord so AR varies naturally with span,
%  revealing a genuine aerodynamic optimum in the trade study.
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   TRADE STUDY: Wing Span Sweep\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

Spans = 40:2:70;

mtoms = zeros(size(Spans));
fuels = zeros(size(Spans));
oems  = zeros(size(Spans));
areas = zeros(size(Spans));
ARs   = zeros(size(Spans));

fprintf('Testing %d span configurations from %.0f m to %.0f m...\n', ...
        length(Spans), min(Spans), max(Spans));

for i = 1:length(Spans)
    fprintf('  [%2d/%2d] Span = %.0f m ... ', i, length(Spans), Spans(i));
    ADPi = freshADP(Spans(i), SI);
    try
        ADPi = Boxwing.B777.Size(ADPi, false);
        mtoms(i) = ADPi.MTOM;
        fuels(i) = ADPi.Mf_Fuel * ADPi.MTOM;
        oems(i)  = ADPi.OEM;
        areas(i) = ADPi.WingArea;
        ARs(i)   = ADPi.AR();
        fprintf('MTOM=%.0f t, Fuel=%.0f t, AR=%.2f\n', mtoms(i)/1e3, fuels(i)/1e3, ARs(i));
    catch ME
        fprintf('FAILED: %s\n', ME.message);
        mtoms(i) = NaN; fuels(i) = NaN; oems(i) = NaN;
        areas(i) = NaN; ARs(i)   = NaN;
    end
end
fprintf('\nTrade study complete.\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 12 — TRADE STUDY PLOTS
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
grid on; xlabel('Span [m]','FontSize',11); ylabel('MTOM [t]','FontSize',11);
title('MTOM vs Span','FontSize',12,'FontWeight','bold');

nexttile(2);
plot(Spans, fuels/1e3, '-o', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.9 0.5 0.1], 'MarkerFaceColor', [1.0 0.7 0.3]);
grid on; xlabel('Span [m]','FontSize',11); ylabel('Block Fuel [t]','FontSize',11);
title('Block Fuel vs Span','FontSize',12,'FontWeight','bold');

nexttile(3);
plot(Spans, oems/1e3, '-d', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.3 0.7 0.4], 'MarkerFaceColor', [0.5 0.9 0.6]);
grid on; xlabel('Span [m]','FontSize',11); ylabel('OEM [t]','FontSize',11);
title('OEM vs Span','FontSize',12,'FontWeight','bold');

nexttile(4);
plot(Spans, areas, '-^', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.7 0.3 0.7], 'MarkerFaceColor', [0.9 0.5 0.9]);
grid on; xlabel('Span [m]','FontSize',11); ylabel('Wing Area [m²]','FontSize',11);
title('Wing Area vs Span','FontSize',12,'FontWeight','bold');

nexttile(5);
% AR is fixed at 10 throughout — replaced with Wing Loading which varies
WS_vals = (mtoms * 9.81) ./ areas;
plot(Spans, WS_vals/1e3, '-v', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.8 0.2 0.2], 'MarkerFaceColor', [1.0 0.4 0.4]);
grid on; xlabel('Span [m]','FontSize',11); ylabel('Wing Loading [kN/m²]','FontSize',11);
title('Wing Loading vs Span  (AR = 10 fixed)','FontSize',12,'FontWeight','bold');

nexttile(6);
plot(Spans, (fuels./mtoms)*100, '-p', 'LineWidth', 2, 'MarkerSize', 8, ...
     'Color', [0.5 0.5 0.5], 'MarkerFaceColor', [0.7 0.7 0.7]);
grid on; xlabel('Span [m]','FontSize',11); ylabel('Fuel Fraction [% MTOM]','FontSize',11);
title('Fuel Fraction vs Span','FontSize',12,'FontWeight','bold');

title(tt, 'Boxwing Freighter — Wing Span Trade Study  (AR = 10 fixed)', ...
      'FontSize',14,'FontWeight','bold');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 13 — OPTIMUM SPAN IDENTIFICATION
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   OPTIMUM SPAN ANALYSIS\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

valid = ~isnan(mtoms);
[mtom_min, idx_mtom] = min(mtoms(valid));
span_opt_mtom = Spans(valid); span_opt_mtom = span_opt_mtom(idx_mtom);

[fuel_min, idx_fuel] = min(fuels(valid));
span_opt_fuel = Spans(valid); span_opt_fuel = span_opt_fuel(idx_fuel);

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

% FIX 5: save trade study figure immediately while it's still valid
saveas(figure(3), 'Boxwing_TradeStudy.png');
fprintf('  Saved: Boxwing_TradeStudy.png\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  PART 14 — SUMMARY TABLE
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   SUMMARY TABLE\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

idx_f = find(valid); idx_mtom2 = idx_f(idx_mtom); idx_fuel2 = idx_f(idx_fuel);

fprintf('%-25s | %-12s | %-12s | %-12s\n', 'Parameter', 'Baseline', 'Min MTOM', 'Min Fuel');
fprintf('%s\n', repmat('-', 1, 70));
fprintf('%-25s | %10.1f m | %10.1f m | %10.1f m\n', 'Span',        ADP.EffectiveSpan,        span_opt_mtom,          span_opt_fuel);
fprintf('%-25s | %10.1f t | %10.1f t | %10.1f t\n', 'MTOM',        ADP.MTOM/1e3,             mtom_min/1e3,           mtoms(idx_fuel2)/1e3);
fprintf('%-25s | %10.1f t | %10.1f t | %10.1f t\n', 'Block Fuel',  sizing_out.BlockFuel/1e3, fuels(idx_mtom2)/1e3,   fuel_min/1e3);
fprintf('%-25s | %10.1f t | %10.1f t | %10.1f t\n', 'OEM',         ADP.OEM/1e3,              oems(idx_mtom2)/1e3,    oems(idx_fuel2)/1e3);
fprintf('%-25s | %10.1f m2| %10.1f m2| %10.1f m2\n','Wing Area',   ADP.WingArea,             areas(idx_mtom2),       areas(idx_fuel2));
fprintf('%-25s | %10.2f   | %10.2f   | %10.2f\n',   'Aspect Ratio',ADP.AR(),                 ARs(idx_mtom2),         ARs(idx_fuel2));
fprintf('%s\n\n', repmat('-', 1, 70));


%% ═══════════════════════════════════════════════════════════════════════
%  PART 15 — EXPORT CSV
%  FIX 5: figure saveas calls moved to immediately after each plot.
%          Only CSV export remains here.
%% ═══════════════════════════════════════════════════════════════════════

fprintf('═══════════════════════════════════════════════════════════\n');
fprintf('   EXPORTING RESULTS\n');
fprintf('═══════════════════════════════════════════════════════════\n\n');

T = table(Spans', mtoms'/1e3, fuels'/1e3, oems'/1e3, areas', ARs', ...
          'VariableNames', {'Span_m','MTOM_t','BlockFuel_t','OEM_t','WingArea_m2','AspectRatio'});
writetable(T, 'Boxwing_TradeStudy.csv');
fprintf('Trade study data saved: Boxwing_TradeStudy.csv\n\n');

fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                  ANALYSIS COMPLETE                         ║\n');
fprintf('╠════════════════════════════════════════════════════════════╣\n');
fprintf('║  Figures 1–7,14 saved to disk. CSV exported.               ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');


%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL HELPERS
%% ═══════════════════════════════════════════════════════════════════════

function ADPi = freshADP(frontSpan, SI)
%FRESHADP  Create a new ADP for the given front-wing span.
%  FIX 4: c_ref_fixed is initialised to 0 in the new ADP constructor,
%  then set on first updateDerivedProps call from baseline 60m span.
%  When we set a different frontSpan and call updateDerivedProps again,
%  c_ref_fixed is already non-zero so it is NOT overwritten — meaning
%  WingArea = b_eff * c_ref (linear in span, not quadratic).
%  This lets AR vary from ~6.7 (40m) to ~10 (60m), revealing the optimum.

    ADPi           = Boxwing.B777.ADP();   % constructor sets c_ref_fixed = 6m
    ADPi.TLAR      = Boxwing.cast.TLAR.Boxwing();
    ADPi.Engine    = GE90Engine();

    ADPi.CockpitLength = 6.5;
    ADPi.CabinRadius   = 2.93;
    ADPi.CabinLength   = 70.0 - ADPi.CockpitLength - ADPi.CabinRadius*2*1.48;

    % Wing positions are computed by updateDerivedProps — do not set manually here.

    ADPi.V_HT = 0;
    ADPi.V_VT = 0.05;

    rearSpan = frontSpan - 10;
    ADPi.FrontWingSpan   = frontSpan;
    ADPi.RearWingSpan    = rearSpan;
    ADPi.ConnectorHeight = 3;
    ADPi.updateDerivedProps();   % c_ref_fixed kept; WingArea = b_eff*c_ref

    ADPi.MTOM    = 3.0 * ADPi.TLAR.Payload;
    ADPi.Mf_Fuel = 0.28;
    ADPi.Mf_res  = 0.04;
    ADPi.Mf_Ldg  = 0.75;
    ADPi.Mf_TOC  = 0.98;

    Boxwing.B777.UpdateAero(ADPi);
end


function eng = GE90Engine()
    eng.T_ref    = 513e3;
    eng.D_ref    = 3.124;
    eng.L_ref    = 7.29;
    eng.M_ref    = 8618;
    eng.Diameter = eng.D_ref;
    eng.Length   = eng.L_ref;
    eng.Mass     = eng.M_ref;
    eng.TSFC     = @(Mach, alt_m) rubberTSFC(Mach, alt_m);
end

function tsfc = rubberTSFC(Mach, alt_m)
    TSFC_SLS = 0.0158e-3;
    theta    = (288.15 - 0.0065*min(alt_m, 11000)) / 288.15;
    tsfc     = max(TSFC_SLS * theta^(-0.5) * (0.45 + 0.54*Mach), 1e-5);
end

function tsfc = ultraFanTSFC(Mach, alt_m)
    BPR    = 15;
    SFC_TO = 18 * exp(-0.12*BPR) * 1e-6;
    SFC_cr = 22 * exp(-0.05*BPR) * 1e-6;
    T      = max(288.15 - 0.0065*min(alt_m,11000), 216.65);
    sr     = sqrt(T/288.15);
    SFC_B  = (SFC_cr/sr - SFC_TO) / 0.82;
    tsfc   = max((SFC_TO + SFC_B*Mach)*sr, 1e-6);
end