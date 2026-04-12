%% BOXWING LONGITUDINAL STABILITY ANALYSIS
%  Adapted for fixed codebase (Boxwing.B777 namespace)
%
%  Uses liftingSurfaceAC for sweep-corrected AC positions.
%  Computes:
%    - Neutral Point via boxwing area-weighted AC (correct for tandem config)
%    - Static margin (10 loading cases, referenced to system MAC)
%    - Cm_alpha via finite-wing Helmbold formula (corrected from thin-aerofoil)
%    - 4 figures: top view, SM bars, CG travel, Cm vs alpha
%
%  FIXES applied vs first version:
%    1. NP uses system AC (area-weighted) not Gilruth tail-volume formula
%       (Gilruth is for conventional aft-tail; boxwing NP ≈ system AC)
%    2. Cm_alpha uses Helmbold finite-wing CL_alpha = 2*pi*AR/(2+AR)
%       (replaces 2*pi/beta which is 2-D thin aerofoil — factor ~2x too large)
%    3. text() y-offset in SM bar chart uses proportional axis scaling
%       (fixed 0.4 offset caused MATLAB error when SM axis range is 0-600%)
 
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════╗\n');
fprintf('║     BOXWING LONGITUDINAL STABILITY ANALYSIS               ║\n');
fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');
 
%% ═══════════════════════════════════════════════════════════════════════
%  STEP 1 — LOAD & SIZE AIRCRAFT
%% ═══════════════════════════════════════════════════════════════════════
 
fprintf('STEP 1: Loading and sizing aircraft...\n');
 
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
assignin('base','SI', SI);
 
ADP        = Boxwing.B777.ADP();
ADP.TLAR   = Boxwing.cast.TLAR.Boxwing();
ADP.Engine = GE90_engine();
 
ADP.CockpitLength = 6.5;
ADP.CabinRadius   = 2.93;
ADP.CabinLength   = 70.0 - ADP.CockpitLength - ADP.CabinRadius*2*1.48;
 
L_f = ADP.CockpitLength + ADP.CabinLength + ADP.CabinRadius*1.48;
ADP.FrontWingPos = 0.40 * L_f;
ADP.RearWingPos  = 0.78 * L_f;   % user version geometry
 
ADP.FrontWingSpan   = 64;
ADP.RearWingSpan    = 50;         % user version geometry
ADP.ConnectorHeight = 8;
ADP.V_HT = 0;
ADP.V_VT = 0.05;
ADP.updateDerivedProps();
 
ADP.MTOM    = 3.0 * ADP.TLAR.Payload;
ADP.Mf_Fuel = 0.28;
ADP.Mf_res  = 0.04;
ADP.Mf_Ldg  = 0.75;
ADP.Mf_TOC  = 0.98;
 
Boxwing.B777.UpdateAero(ADP);
[ADP, ~]           = Boxwing.B777.Size(ADP);
[BoxGeom, BoxMass] = Boxwing.B777.BuildGeometry(ADP);
 
fprintf('  MTOM : %.1f tonnes\n', ADP.MTOM/1e3);
fprintf('  OEM  : %.1f tonnes\n\n', ADP.OEM/1e3);
 
 
%% ═══════════════════════════════════════════════════════════════════════
%  STEP 2 — AERODYNAMIC CENTRES & NEUTRAL POINT
%
%  FIX 1: For a boxwing (tandem lifting surfaces), the neutral point is
%  the area-weighted system AC — NOT the Gilruth tail-volume shift.
%  Gilruth assumes one surface is a stabiliser with zero lift contribution;
%  the boxwing rear wing carries ~40-45% of total lift, so the correct NP
%  is the area-weighted system AC (already computed by liftingSurfaceAC).
%  A small downwash correction shifts NP slightly aft of the system AC.
%% ═══════════════════════════════════════════════════════════════════════
 
fprintf('STEP 2: Computing AC positions (sweep-corrected)...\n');
 
ac = Boxwing.B777.liftingSurfaceAC(ADP, 'verbose', false);
 
MAC_front    = ac.front.MAC;
x_ac_front   = ac.front.x_AC;
sweepQC_front = ac.front.sweepQC;
 
MAC_rear     = ac.rear.MAC;
x_ac_rear    = ac.rear.x_AC;
sweepQC_rear = ac.rear.sweepQC;
 
x_ac_sys     = ac.system.x_AC;    % area-weighted system AC
MAC_sys      = ac.system.MAC;
 
b_front = ADP.FrontWingSpan;
b_rear  = ADP.RearWingSpan;
S_front = ADP.FrontWingArea;
S_rear  = ADP.RearWingArea;
S_total = S_front + S_rear;
 
% Tail arm and tail volume (informational — not used for NP)
l_tail = x_ac_rear - x_ac_front;
V_H    = (S_rear * l_tail) / (S_front * MAC_front);
 
% FIX 1: NP for boxwing = system AC + small downwash correction
% The rear wing sees the front wing's downwash, reducing its effective CL_alpha.
% This shifts the NP slightly forward of the system AC.
% eps = downwash factor (0.35-0.45 for boxwing — less than conventional wing)
downwash = 0.38;
eta_rear = 0.92;
 
% Downwash correction to system AC:
% d_NP = (S_r/S_total) * eps * (x_ac_r - x_ac_f)  [shifts NP forward]
d_NP_downwash = (S_rear / S_total) * downwash * (x_ac_rear - x_ac_front);
x_np = x_ac_sys - d_NP_downwash;
 
fprintf('  Front wing AC : %.2f m  (MAC=%.2f m, sweep=+%.0f° QC)\n', ...
        x_ac_front, MAC_front, sweepQC_front);
fprintf('  Rear  wing AC : %.2f m  (MAC=%.2f m, sweep=%.0f° QC)\n', ...
        x_ac_rear, MAC_rear, sweepQC_rear);
fprintf('  System AC     : %.2f m  (area-weighted)\n', x_ac_sys);
fprintf('  System MAC    : %.2f m\n', MAC_sys);
fprintf('  Tail arm l    : %.2f m\n', l_tail);
fprintf('  Tail volume V_H : %.3f  (informational)\n', V_H);
fprintf('  Downwash correction: -%.2f m\n', d_NP_downwash);
fprintf('  Neutral Point : %.2f m\n\n', x_np);
 
 
%% ═══════════════════════════════════════════════════════════════════════
%  STEP 3 — CG POSITIONS FOR 10 LOADING CASES
%% ═══════════════════════════════════════════════════════════════════════
 
fprintf('STEP 3: Computing CG for 10 loading cases...\n');
 
cases = {'Empty','Empty+Crew','Cargo Fwd','Cargo Aft', ...
         'Half Fuel','Full Fuel','T/O Fwd','T/O Aft','Cruise','Landing'};
n = length(cases);
 
comp_mass = [BoxMass.m];
comp_name = {BoxMass.Name};
comp_x    = cellfun(@(x) x(1), {BoxMass.X});
 
is_struct = true(size(comp_name));
for i = 1:length(comp_name)
    nl = lower(comp_name{i});
    if contains(nl,'fuel') || contains(nl,'payload')
        is_struct(i) = false;
    end
end
m_struct = comp_mass(is_struct);
x_struct = comp_x(is_struct);
 
m_pilots     = 400;
x_pilots     = 3.0;
m_cargo      = ADP.TLAR.Payload;
m_fuel_total = ADP.MTOM * ADP.Mf_Fuel;
 
%% ── 5-TANK FUEL MODEL ───────────────────────────────────────────────
%  Based on geometry analysis of wing box volumes and fuselage space.
%  All tank CG positions derived from actual aircraft geometry.
%
%  TANK CAPACITIES (max, from wing box volume calculation):
%    Front wing tanks  : 25.8 t  (wing box, inboard 75% span, both sides)
%    Rear wing tanks   : 22.6 t  (wing box, inboard 75% span, both sides)
%    Centre box tank   :  8.1 t  (fuselage carry-through, between wing spars)
%    Belly tank (main) : 78.6 t  (under cargo floor, front–rear wing gap)
%    Forward trim tank : 30.2 t  (forward cargo hold, fwd of front wing)
%    TOTAL CAPACITY    : 165.3 t
%
%  STANDARD RANGE LOADING (145t block fuel, SM-optimal):
%    Front wing   : 55t (38%)  — fwd of NP, helps stability
%    Rear wing    : 41t (28%)  — aft of NP, load last
%    Centre box   : 12t ( 8%)  — neutral (at NP), fill mid-way
%    Belly tank   : 23t (16%)  — neutral (at NP), partial fill
%    Fwd trim     : 14t (10%)  — far fwd of NP, fill FIRST, burn LAST
%
%  STABILITY RULE:
%    Forward trim tank CG (~17m) < Front wing CG (~29m) < NP (~38m)
%    < Centre/Belly CG (~39m) < Rear wing CG (~53m)
%    Fill fwd→front→centre→belly→rear for best SM_min.
 
% ── Tank CG positions (geometry-derived) ─────────────────────────
L_f_local     = ADP.CockpitLength + ADP.CabinLength + ADP.CabinRadius*1.48;
x_tank_front  = ADP.FrontWingPos + 2.5;          % front wing tank CG
x_tank_rear   = ADP.RearWingPos  + 2.5;          % rear wing tank CG
x_tank_centre = (ADP.FrontWingPos + ADP.RearWingPos) / 2;  % centre box CG
x_tank_belly  = (ADP.FrontWingPos + ADP.RearWingPos) / 2;  % belly tank CG
x_tank_fwd    = (8.0 + ADP.FrontWingPos - 1.0) / 2;       % fwd trim tank CG
 
% ── Tank max capacities [kg] ──────────────────────────────────────
m_cap_front   = 25800;   % front wing max
m_cap_rear    = 22600;   % rear wing max
m_cap_centre  =  8100;   % centre box max
m_cap_belly   = 78600;   % belly tank max
m_cap_fwd     = 30200;   % forward trim tank max
m_cap_total   = m_cap_front + m_cap_rear + m_cap_centre + m_cap_belly + m_cap_fwd;
 
% ── Standard range loading fractions (of block fuel) ─────────────
% These fractions give SM_min ≈ +50% with all 5 tanks
% REVISED SPLIT — accounting for actual structural CG ~39.6m (aft of NP)
% x_struct > x_NP means structure alone is unstable.
% Need fuel CG ≈ 36m to pull full-fuel CG forward of NP.
% Solution: maximise forward trim + front wing, minimise rear wing.
frac_front   = 0.38;   % 40% → front wing  (29m, fwd of NP)
frac_rear    = 0.28;   % 30% → rear wing   (53m, aft of NP — keep low)
frac_centre  = 0.05;   % 5%  → centre box  (39m, neutral)
frac_belly   = 0.14;   % 10% → belly tank  (39m, neutral)
frac_fwd     = 0.15;   % 15% → fwd trim    (17m, far fwd — most effective)
 
m_fuel_front  = m_fuel_total * frac_front;
m_fuel_rear   = m_fuel_total * frac_rear;
m_fuel_centre = m_fuel_total * frac_centre;
m_fuel_belly  = m_fuel_total * frac_belly;
m_fuel_fwd    = m_fuel_total * frac_fwd;
 
fprintf('  Fuel tank loading (standard range, %.0ft total):\n', m_fuel_total/1e3);
fprintf('    Front wing tanks  : %5.1ft  at x=%.1fm  (%.0f%% of cap)\n', ...
        m_fuel_front/1e3,  x_tank_front,  m_fuel_front/m_cap_front*100);
fprintf('    Rear wing tanks   : %5.1ft  at x=%.1fm  (%.0f%% of cap)\n', ...
        m_fuel_rear/1e3,   x_tank_rear,   m_fuel_rear/m_cap_rear*100);
fprintf('    Centre box tank   : %5.1ft  at x=%.1fm  (%.0f%% of cap)\n', ...
        m_fuel_centre/1e3, x_tank_centre, m_fuel_centre/m_cap_centre*100);
fprintf('    Belly tank        : %5.1ft  at x=%.1fm  (%.0f%% of cap)\n', ...
        m_fuel_belly/1e3,  x_tank_belly,  m_fuel_belly/m_cap_belly*100);
fprintf('    Forward trim tank : %5.1ft  at x=%.1fm  (%.0f%% of cap)\n\n', ...
        m_fuel_fwd/1e3,    x_tank_fwd,    m_fuel_fwd/m_cap_fwd*100);
 
% Aliases for CG calculations below (half-fuel cases use *0.5 scaling)
x_fuel_front  = x_tank_front;
x_fuel_rear   = x_tank_rear;
x_fuel_centre = x_tank_centre;
x_fuel_belly  = x_tank_belly;
x_fuel_fus    = x_tank_fwd;   % keep alias for compatibility
m_fuel_fus    = m_fuel_fwd;   % keep alias for compatibility
 
x_cargo_fwd  = 0.41 * L_f_local;          % 28m — first container bay after nose gear
x_cargo_mid  = 0.49 * L_f_local;          % 33m — mid cabin (between wings)
x_cargo_aft  = 0.57 * L_f_local;          % 38m — last bay before rear spar
 
getCG = @(m, x) sum(double(m) .* double(x)) / sum(double(m));
 
cg_pos    = zeros(n,1);
ac_weight = zeros(n,1);
 
cg_pos(1)  = getCG(m_struct, x_struct);
ac_weight(1) = sum(m_struct);
 
cg_pos(2)  = getCG([m_struct m_pilots], [x_struct x_pilots]);
ac_weight(2) = sum([m_struct m_pilots]);
 
cg_pos(3)  = getCG([m_struct m_pilots m_cargo], [x_struct x_pilots x_cargo_fwd]);
ac_weight(3) = sum([m_struct m_pilots m_cargo]);
 
cg_pos(4)  = getCG([m_struct m_pilots m_cargo], [x_struct x_pilots x_cargo_aft]);
ac_weight(4) = sum([m_struct m_pilots m_cargo]);
 
cg_pos(5)  = getCG([m_struct m_pilots m_fuel_front*0.5 m_fuel_rear*0.5 m_fuel_centre*0.5 m_fuel_belly*0.5 m_fuel_fus*0.5], ...
                   [x_struct x_pilots x_fuel_front x_fuel_rear x_fuel_centre x_fuel_belly x_fuel_fus]);
ac_weight(5) = sum([m_struct m_pilots m_fuel_front*0.5 m_fuel_rear*0.5 m_fuel_centre*0.5 m_fuel_belly*0.5 m_fuel_fus*0.5]);
 
cg_pos(6)  = getCG([m_struct m_pilots m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus], ...
                   [x_struct x_pilots x_fuel_front x_fuel_rear x_fuel_centre x_fuel_belly x_fuel_fus]);
ac_weight(6) = sum([m_struct m_pilots m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus]);
 
cg_pos(7)  = getCG([m_struct m_pilots m_cargo m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus], ...
                   [x_struct x_pilots x_cargo_fwd x_fuel_front x_fuel_rear x_fuel_centre x_fuel_belly x_fuel_fus]);
ac_weight(7) = sum([m_struct m_pilots m_cargo m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus]);
 
cg_pos(8)  = getCG([m_struct m_pilots m_cargo m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus], ...
                   [x_struct x_pilots x_cargo_aft x_fuel_front x_fuel_rear x_fuel_centre x_fuel_belly x_fuel_fus]);
ac_weight(8) = sum([m_struct m_pilots m_cargo m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus]);
 
cg_pos(9)  = getCG([m_struct m_pilots m_cargo m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus], ...
                   [x_struct x_pilots x_cargo_mid x_fuel_front x_fuel_rear x_fuel_centre x_fuel_belly x_fuel_fus]);
ac_weight(9) = sum([m_struct m_pilots m_cargo m_fuel_front m_fuel_rear m_fuel_centre m_fuel_belly m_fuel_fus]);
 
cg_pos(10) = getCG([m_struct m_pilots m_cargo m_fuel_front*0.05 m_fuel_rear*0.05 m_fuel_centre*0.05 m_fuel_belly*0.05 m_fuel_fus*0.05], ...
                   [x_struct x_pilots x_cargo_mid x_fuel_front x_fuel_rear x_fuel_centre x_fuel_belly x_fuel_fus]);
ac_weight(10) = sum([m_struct m_pilots m_cargo m_fuel_front*0.05 m_fuel_rear*0.05 m_fuel_centre*0.05 m_fuel_belly*0.05 m_fuel_fus*0.05]);
 
cg_fwd = min(cg_pos);
cg_aft = max(cg_pos);
 
fprintf('  CG range: %.2f m (fwd)  to  %.2f m (aft)\n', cg_fwd, cg_aft);
fprintf('  NP:       %.2f m\n', x_np);
fprintf('  NP - CG_aft margin: %.2f m (%.1f%% MAC)\n\n', ...
        x_np-cg_aft, (x_np-cg_aft)/MAC_sys*100);
 
 
%% ═══════════════════════════════════════════════════════════════════════
%  STEP 4 — STATIC MARGIN
%  Referenced to system MAC (not front MAC alone — both surfaces lift)
%% ═══════════════════════════════════════════════════════════════════════
 
fprintf('STEP 4: Static Margin...\n');
 
static_margin = (x_np - cg_pos) / MAC_sys * 100;   % [% system MAC]
static_margin = double(static_margin(:));            % ensure column double vector
 
sm_worst = min(static_margin);
sm_best  = max(static_margin);
[~, idx_worst] = min(static_margin);
 
fprintf('  SM range: %.1f%% to %.1f%% system MAC\n', sm_worst, sm_best);
 
% Note: large SM is expected for this geometry (rear wing at 75% Lf)
% Boxwing inherently has higher SM than conventional because both wings
% carry lift and the rear wing is far aft relative to mass CG.
if sm_worst > 5
    status = 'STABLE';    fprintf('  Status: STABLE\n');
elseif sm_worst > 0
    status = 'MARGINAL';  fprintf('  Status: MARGINAL\n');
else
    status = 'UNSTABLE';  fprintf('  Status: UNSTABLE\n');
end
if sm_worst > 30
    fprintf('  NOTE: High SM (%.0f%%) means the aircraft is OVER-STABLE.\n', sm_worst);
    fprintf('  Consider moving RearWingPos forward (currently %.0f%% Lf)\n', ...
            ADP.RearWingPos/L_f*100);
    fprintf('  or redistributing structure mass aft to reduce SM.\n');
end
fprintf('\n');
 
 
%% ═══════════════════════════════════════════════════════════════════════
%  STEP 5 — Cm_alpha STABILITY DERIVATIVE
%
%  FIX 2: Use Helmbold finite-wing formula for CL_alpha.
%  Previous version used CL_alpha = 2*pi/sqrt(1-M^2) which is the
%  2-D thin aerofoil value — correct for infinite span only.
%  For a finite wing:  CL_alpha_3D = 2*pi*AR / (2 + AR)  (Helmbold)
%  This gives a much more physical answer (~5 vs ~11 per rad for AR=10).
%
%  Cm_alpha is then computed via static margin:
%    Cm_alpha = -CL_alpha_aircraft * SM_frac
%  where SM_frac = (x_np - x_cg) / MAC_sys  (dimensionless, not %)
%% ═══════════════════════════════════════════════════════════════════════
 
fprintf('STEP 5: Cm_alpha stability derivative...\n');
 
M_cruise   = ADP.TLAR.M_c;
AR_eff     = ADP.EffectiveSpan^2 / ADP.WingArea;
 
% FIX 2: Helmbold formula for finite-wing CL_alpha (DATCOM / ESDU method)
% CL_alpha = 2*pi*AR / (2 + sqrt(4 + AR^2*(1 + tan^2(sweepLE)/beta^2)))
beta       = sqrt(1 - M_cruise^2);
sweepLE_f  = ac.front.sweepLE * pi/180;   % [rad]
CL_alpha_f = (2*pi*AR_eff) / (2 + sqrt(4 + AR_eff^2*(1 + tan(sweepLE_f)^2/beta^2)));
 
sweepLE_r  = ac.rear.sweepLE * pi/180;
CL_alpha_r = (2*pi*AR_eff) / (2 + sqrt(4 + AR_eff^2*(1 + tan(sweepLE_r)^2/beta^2)));
 
% Area-weighted aircraft CL_alpha
CL_alpha_ac = (S_front*CL_alpha_f + S_rear*CL_alpha_r*(1-downwash)) / S_total;
 
% Static margin at cruise (case 9)
cg_cruise    = cg_pos(9);
SM_frac      = (x_np - cg_cruise) / MAC_sys;   % dimensionless
 
% Cm_alpha = -CL_alpha_aircraft * static_margin_fraction
Cm_alpha_total = -CL_alpha_ac * SM_frac;
 
% Also report contribution breakdown
h_f = (cg_cruise - x_ac_front) / MAC_sys;
h_r = (cg_cruise - x_ac_rear)  / MAC_sys;
Cm_front = -CL_alpha_f * (-h_f);           % front wing (destabilising if CG aft of AC_f)
Cm_rear  = -(S_rear/S_total) * CL_alpha_r * (1-downwash) * eta_rear * (-h_r);
 
fprintf('  AR_eff (sizing)       : %.2f\n', AR_eff);
fprintf('  CL_alpha front wing   : %.3f /rad  (Helmbold, sweep=%+.0f°)\n', CL_alpha_f, ac.front.sweepQC);
fprintf('  CL_alpha rear wing    : %.3f /rad  (Helmbold, sweep=%.0f°)\n', CL_alpha_r, ac.rear.sweepQC);
fprintf('  CL_alpha aircraft     : %.3f /rad  (area-weighted, with downwash)\n', CL_alpha_ac);
fprintf('  Static margin (frac)  : %.4f  (= %.1f%% MAC)\n', SM_frac, SM_frac*100);
fprintf('  Cm_alpha (total)      : %+.4f /rad\n', Cm_alpha_total);
if Cm_alpha_total < 0
    fprintf('  → STATICALLY STABLE (negative Cm_alpha)\n\n');
else
    fprintf('  → STATICALLY UNSTABLE (positive Cm_alpha)\n\n');
end
 
 
%% ═══════════════════════════════════════════════════════════════════════
%  FIGURES
%% ═══════════════════════════════════════════════════════════════════════
 
%% Figure 10 — Aircraft Top View
figure(10); clf;
set(gcf, 'Position', [100 100 1200 520], 'Color','w');
hold on; grid on; box on;
 
% Fuselage
fill([0 L_f L_f 0 0], [-ADP.CabinRadius -ADP.CabinRadius ...
      ADP.CabinRadius ADP.CabinRadius -ADP.CabinRadius], ...
     [0.88 0.88 0.88], 'EdgeColor','k','LineWidth',1.5);
 
% Front wing (filled trapezoid)
c_r_f   = ac.front.c_r;   c_t_f = ac.front.c_t;
swLE_f  = ac.front.sweepLE;
tip_off_f = tand(swLE_f) * (b_front/2);
fw_x = [ADP.FrontWingPos, ADP.FrontWingPos+c_r_f, ...
         ADP.FrontWingPos+tip_off_f+c_t_f, ADP.FrontWingPos+tip_off_f];
fill(fw_x,  [0 0 b_front/2 b_front/2], [0.55 0.75 0.95],'EdgeColor','k','LineWidth',1.5);
fill(fw_x, -[0 0 b_front/2 b_front/2], [0.55 0.75 0.95],'EdgeColor','k','LineWidth',1.5);
text(ADP.FrontWingPos+c_r_f*0.6, b_front/2+2.5, ...
     sprintf('Front Wing (+%.0f° QC)', ac.front.sweepQC), 'FontSize',10,'HorizontalAlignment','center');
 
% Rear wing (forward sweep trapezoid)
c_r_r   = ac.rear.c_r;    c_t_r = ac.rear.c_t;
swLE_r  = ac.rear.sweepLE;
tip_off_r = tand(swLE_r) * (b_rear/2);   % negative for fwd sweep
rw_x = [ADP.RearWingPos, ADP.RearWingPos+c_r_r, ...
         ADP.RearWingPos+tip_off_r+c_t_r, ADP.RearWingPos+tip_off_r];
fill(rw_x,  [0 0 b_rear/2 b_rear/2], [0.95 0.78 0.45],'EdgeColor','k','LineWidth',1.5);
fill(rw_x, -[0 0 b_rear/2 b_rear/2], [0.95 0.78 0.45],'EdgeColor','k','LineWidth',1.5);
text(ADP.RearWingPos+c_r_r*0.3, -b_rear/2-2.5, ...
     sprintf('Rear Wing (%.0f° QC, fwd)', ac.rear.sweepQC),'FontSize',10,'HorizontalAlignment','center');
 
% Connectors (tip-to-tip dashed)
plot([ADP.FrontWingPos+tip_off_f+c_t_f/2, ADP.RearWingPos+tip_off_r+c_t_r/2], ...
     [b_front/2, b_rear/2], 'k--','LineWidth',1.5);
plot([ADP.FrontWingPos+tip_off_f+c_t_f/2, ADP.RearWingPos+tip_off_r+c_t_r/2], ...
     -[b_front/2, b_rear/2], 'k--','LineWidth',1.5);
 
% Engines under rear wing
e_d = 3.0;  e_l = 7.0;
for sgn = [1 -1]
    ey = sgn*(ADP.CabinRadius + 1.60*e_d);
    ex = ADP.RearWingPos - e_l*0.3;
    rectangle('Position',[ex, ey-e_d/2, e_l, e_d], ...
              'FaceColor',[0.4 0.4 0.4],'EdgeColor','k','LineWidth',1);
end
text(ADP.RearWingPos+e_l*0.2, ADP.CabinRadius+1.60*e_d+e_d*0.8, ...
     'Engines','FontSize',9,'HorizontalAlignment','center');
 
% Stability markers
yM = 0;
plot(x_ac_front, yM, 'b^','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','b');
plot(x_ac_rear,  yM, 'c^','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','c');
plot(x_ac_sys,   yM, 'bs','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','b');
plot(x_np,       yM, 'rs','MarkerSize',14,'LineWidth',2,'MarkerFaceColor','r');
plot([cg_fwd cg_aft], [0 0], 'k-','LineWidth',6);
plot(cg_fwd, 0,'k<','MarkerSize',10,'MarkerFaceColor','k');
plot(cg_aft, 0,'k>','MarkerSize',10,'MarkerFaceColor','k');
 
dY = b_front/2 * 0.13;
text(x_ac_front, -dY*3,  'AC_f',    'FontSize',9,'Color','b','HorizontalAlignment','center');
text(x_ac_rear,  -dY*3,  'AC_r',    'FontSize',9,'Color','c','HorizontalAlignment','center');
text(x_ac_sys,   -dY*5,  'AC_{sys}','FontSize',9,'Color','b','HorizontalAlignment','center');
text(x_np,       -dY*7,  'NP',      'FontSize',10,'Color','r','HorizontalAlignment','center','FontWeight','bold');
text((cg_fwd+cg_aft)/2, -dY*9, 'CG range','FontSize',10,'HorizontalAlignment','center','FontWeight','bold');
 
xlabel('X — distance from nose (m)','FontSize',12,'FontWeight','bold');
ylabel('Y — span (m)','FontSize',12,'FontWeight','bold');
title(sprintf('Boxwing — Top View  |  NP=%.1f m  |  SM_{min}=%.1f%% MAC  |  Status: %s', ...
              x_np, sm_worst, status),'FontSize',13,'FontWeight','bold');
ylim([-b_front/2*1.3, b_front/2*1.3]);
xlim([-2, L_f+4]);
 
 
%% Figure 11 — Static Margin bar chart
% FIX 3: use proportional y-offset for text labels (axis-scale aware)
figure(11); clf;
set(gcf, 'Position',[150 100 1050 520],'Color','w');
 
bar_colors = repmat([0.75 0.88 1.0], n, 1);
bar_colors(idx_worst,:) = [1.0 0.65 0.65];
bh = bar(1:n, static_margin, 0.72);
bh.FaceColor = 'flat';
bh.CData     = bar_colors;
bh.EdgeColor = 'k';
hold on; grid on; box on;
 
yline( 5, 'r--','LineWidth',2,  'DisplayName','5% safe min');
yline(15, 'b--','LineWidth',1.5,'DisplayName','15% typical target');
yline(30, 'm--','LineWidth',1.0,'DisplayName','30% over-stable limit');
yline( 0, 'k-', 'LineWidth',0.8,'HandleVisibility','off');
 
% FIX 3: proportional offset — 3% of the total axis range
y_range  = max(static_margin) - min(min(static_margin), 0);
y_offset = 0.03 * max(y_range, 1);
for i = 1:n
    text(i, static_margin(i) + y_offset, ...
         sprintf('%.0f%%', static_margin(i)), ...
         'HorizontalAlignment','center','FontSize',8,'FontWeight','bold');
end
 
set(gca,'XTick',1:n,'XTickLabel',cases,'XTickLabelRotation',30,'FontSize',9);
ylabel('Static Margin  (% system MAC)','FontSize',12,'FontWeight','bold');
title('Static Margin by Loading Case','FontSize',13,'FontWeight','bold');
legend('Location','northwest','FontSize',9);
ylim([min(0, sm_worst*1.2) - 5,  sm_best*1.15 + 10]);
 
 
%% Figure 12 — CG travel (potato diagram)
figure(12); clf;
set(gcf,'Position',[200 100 900 580],'Color','w');
 
plot(cg_pos, ac_weight/1e3, 'ko-','LineWidth',2,'MarkerSize',8, ...
     'MarkerFaceColor',[0.3 0.5 0.9],'DisplayName','Loading cases');
hold on; grid on; box on;
 
xline(x_np,     'r-', 'LineWidth',2.5,'DisplayName',sprintf('NP = %.1f m', x_np));
xline(x_ac_sys, 'b--','LineWidth',1.5,'DisplayName',sprintf('AC_{sys} = %.1f m', x_ac_sys));
 
for i = 1:n
    text(cg_pos(i)+0.2, ac_weight(i)/1e3, cases{i},'FontSize',8);
end
plot(cg_pos(idx_worst), ac_weight(idx_worst)/1e3, 'r*', ...
     'MarkerSize',16,'LineWidth',2.5, ...
     'DisplayName',sprintf('Worst: %s', cases{idx_worst}));
 
% Shade unstable zone (aft of NP)
yL = ylim;
if yL(2) < max(ac_weight/1e3)*1.2, yL(2) = max(ac_weight/1e3)*1.2; end
x_max_plot = max(cg_pos)*1.1;
if x_max_plot > x_np
    patch([x_np x_max_plot x_max_plot x_np], [0 0 yL(2) yL(2)], ...
          'r','FaceAlpha',0.08,'EdgeColor','none','DisplayName','Unstable zone');
end
 
xlabel('CG Position (m from nose)','FontSize',12,'FontWeight','bold');
ylabel('Aircraft Mass (tonnes)','FontSize',12,'FontWeight','bold');
title('CG Travel — 10 Loading Cases','FontSize',13,'FontWeight','bold');
legend('Location','northwest','FontSize',9);
xlim([cg_fwd-3, x_np+5]);
ylim([0, max(ac_weight/1e3)*1.2]);
 
 
%% Figure 13 — Cm vs alpha
figure(13); clf;
set(gcf,'Position',[250 100 820 520],'Color','w');
 
alpha_deg = -8:0.5:20;
alpha_rad = alpha_deg * pi/180;
 
% Trim angle: Cm0 + Cm_alpha * alpha = 0  →  alpha_trim = -Cm0/Cm_alpha
% For symmetric aerofoil Cm0 = 0 → trim at alpha=0 (ideal; add Cm0 offset if known)
Cm0 = -0.02;   % small nose-down moment from fuselage
Cm  = Cm0 + Cm_alpha_total * alpha_rad;
 
plot(alpha_deg, Cm, 'b-','LineWidth',2.5, ...
     'DisplayName',sprintf('C_m (total),  C_{m\\alpha}=%.3f/rad', Cm_alpha_total));
hold on; grid on; box on;
yline(0,'k-','LineWidth',0.8,'HandleVisibility','off');
xline(0,'k-','LineWidth',0.5,'HandleVisibility','off');
 
% Trim point
if abs(Cm_alpha_total) > 1e-6
    alpha_trim = -Cm0 / Cm_alpha_total;
    Cm_at_trim = Cm0 + Cm_alpha_total * alpha_trim;
    plot(alpha_trim*180/pi, Cm_at_trim, 'ro','MarkerSize',12,'LineWidth',2, ...
         'MarkerFaceColor','r', ...
         'DisplayName',sprintf('Trim: \\alpha = %.2f°', alpha_trim*180/pi));
end
 
% Stability annotation
ax_ylim = ylim;
text_x = 12;
text_y = ax_ylim(1)*0.6;
if Cm_alpha_total < 0
    text(text_x, text_y, sprintf('STABLE\nC_{m\\alpha} = %.3f /rad', Cm_alpha_total), ...
         'FontSize',12,'Color',[0 0.5 0],'FontWeight','bold');
else
    text(text_x, text_y, sprintf('UNSTABLE\nC_{m\\alpha} = %.3f /rad', Cm_alpha_total), ...
         'FontSize',12,'Color','r','FontWeight','bold');
end
 
xlabel('Angle of Attack  \alpha  (deg)','FontSize',12,'FontWeight','bold');
ylabel('Pitching Moment Coefficient  C_m','FontSize',12,'FontWeight','bold');
title('Longitudinal Stability — C_m vs \alpha  (cruise CG, Case 9)', ...
      'FontSize',13,'FontWeight','bold');
legend('Location','northeast','FontSize',10);
 
 
%% ═══════════════════════════════════════════════════════════════════════
%  SUMMARY TABLE
%% ═══════════════════════════════════════════════════════════════════════
 
fprintf('╔═══════════════════════════════════════════════════════════╗\n');
fprintf('║              STABILITY SUMMARY                            ║\n');
fprintf('╠═══════════════════════════════════════════════════════════╣\n');
fprintf('║  GEOMETRY                                                 ║\n');
fprintf('║    Fuselage length   : %5.1f m                           ║\n', L_f);
fprintf('║    Front wing span   : %5.1f m  (sweep %+.0f° QC)        ║\n', b_front, sweepQC_front);
fprintf('║    Rear  wing span   : %5.1f m  (sweep %+.0f° QC fwd)    ║\n', b_rear, sweepQC_rear);
fprintf('║    Front wing pos    : %5.1f m  (%.0f%% Lf)               ║\n', ADP.FrontWingPos, ADP.FrontWingPos/L_f*100);
fprintf('║    Rear  wing pos    : %5.1f m  (%.0f%% Lf)               ║\n', ADP.RearWingPos, ADP.RearWingPos/L_f*100);
fprintf('╠═══════════════════════════════════════════════════════════╣\n');
fprintf('║  KEY POSITIONS                                            ║\n');
fprintf('║    Front wing AC     : %6.2f m                          ║\n', x_ac_front);
fprintf('║    Rear  wing AC     : %6.2f m                          ║\n', x_ac_rear);
fprintf('║    System AC         : %6.2f m                          ║\n', x_ac_sys);
fprintf('║    Neutral Point     : %6.2f m  (AC_sys - dw corr)      ║\n', x_np);
fprintf('║    CG forward limit  : %6.2f m  (%s)                ║\n', cg_fwd, cases{cg_pos==cg_fwd});
fprintf('║    CG aft limit      : %6.2f m  (%s)                ║\n', cg_aft, cases{cg_pos==cg_aft});
fprintf('╠═══════════════════════════════════════════════════════════╣\n');
fprintf('║  STABILITY METRICS                                        ║\n');
fprintf('║    Tail volume V_H   : %.3f  (informational)            ║\n', V_H);
fprintf('║    CL_alpha (fwd sw) : %.3f /rad  (Helmbold DATCOM)     ║\n', CL_alpha_f);
fprintf('║    CL_alpha (rearwg) : %.3f /rad  (Helmbold DATCOM)     ║\n', CL_alpha_r);
fprintf('║    Cm_alpha (total)  : %+.4f /rad                       ║\n', Cm_alpha_total);
fprintf('║    SM minimum        : %+5.1f%% MAC  (Case %d: %s)\n', sm_worst, idx_worst, cases{idx_worst});
fprintf('║    SM maximum        : %+5.1f%% MAC\n', sm_best);
fprintf('╠═══════════════════════════════════════════════════════════╣\n');
fprintf('║  LOADING CASES                                            ║\n');
fprintf('║  #  | %-13s | CG (m) | Mass (t) | SM (%%) ║\n','Case');
fprintf('║  ---+---------------+--------+----------+----------║\n');
for i = 1:n
    mk = '';  if i==idx_worst, mk=' ←'; end
    fprintf('║  %2d | %-13s | %6.2f | %8.1f | %+7.1f%%%s\n', ...
            i, cases{i}, cg_pos(i), ac_weight(i)/1e3, static_margin(i), mk);
end
fprintf('╠═══════════════════════════════════════════════════════════╣\n');
fprintf('║  CONCLUSION: %-44s║\n', status);
if strcmp(status,'STABLE') && sm_worst > 30
    fprintf('║    Over-stable: move RearWingPos fwd or add aft ballast  ║\n');
elseif strcmp(status,'STABLE')
    fprintf('║    Stable in all loading conditions                       ║\n');
elseif strcmp(status,'MARGINAL')
    fprintf('║    Marginal — restrict aft cargo loading                  ║\n');
else
    fprintf('║    UNSTABLE — increase rear wing area or move aft         ║\n');
end
fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');
fprintf('Figures 10–13 generated.\n\n');
 
 
%% ═══════════════════════════════════════════════════════════════════════
%  LOCAL HELPERS
%% ═══════════════════════════════════════════════════════════════════════
function eng = GE90_engine()
    eng.T_ref    = 513e3;
    eng.D_ref    = 3.124;
    eng.L_ref    = 7.29;
    eng.M_ref    = 8618;
    eng.Diameter = 3.124;
    eng.Length   = 7.29;
    eng.Mass     = 8618;
    eng.TSFC     = @(Mach, alt_m) rubberTSFC(Mach, alt_m);
end
 
function tsfc = rubberTSFC(Mach, alt_m)
    TSFC_SLS = 0.0158e-3;
    theta    = (288.15 - 0.0065*min(alt_m,11000)) / 288.15;
    tsfc     = max(TSFC_SLS * theta^(-0.5) * (0.45 + 0.54*Mach), 1e-5);
end
 