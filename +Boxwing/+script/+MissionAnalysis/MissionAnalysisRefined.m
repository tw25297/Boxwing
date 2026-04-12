function [BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, cruise_FL, detail] = ...
    MissionAnalysisRefined(ADP, tripRange_m, M_TO_in)
% MISSIONANALYSISREFINED  Physics-based full mission analysis.
%
%  DROP-IN REPLACEMENT for MissionAnalysis.m with identical output signature
%  plus an optional 7th output (detail struct) for diagnostics.
%
%  SPEC COMPLIANCE (CADEM0016 Appendix A):
%    A.1  Taxi          : 20 min at idle power
%    A.2  Takeoff       : 1 min at max thrust
%    A.3  Climb         : 1,500 ft → ICA, 500 ft segments, ≤250 kts CAS < FL100
%    A.4  Cruise        : step-climb (2,000 ft steps), ROC ≥ 300 ft/min check
%    A.5  Descent       : cruise alt → 1,500 ft, 500 ft segments, 1,800 ft/min ROD
%    A.6  Approach      : 5 min at approach conditions
%    A.7  To Gate       : 20 min at idle power
%    A.8  Alternate     : 350 km full profile (climb + cruise + descent)
%    A.8.1 Loiter       : 30 min at V_md (max L/D speed) at 1,500 ft
%    A.8.2 Contingency  : max(5 min loiter fuel, 3% trip fuel)
%
%  PHYSICS ASSUMPTIONS (stated explicitly):
%    - Thrust lapse   : T_alt = T_SLS * sigma^0.75  (Mattingly turbofan)
%    - Idle thrust    : T_idle = 5% T_SLS * sigma^0.75
%    - Idle TSFC      : TSFC_model * 2.5  (engines inefficient at low power)
%    - ROD prescribed : 1,800 ft/min; T_idle back-calculated from force balance
%    - CAS → TAS      : TAS = V_CAS / sqrt(sigma)  (simplified, M < 0.4)
%    - Cruise step    : step up when ROC at (h + 2,000 ft) ≥ 300 ft/min
%    - Climb thrust   : max available at each segment midpoint altitude
%    - Cruise Breguet : applied per 200 km chunk with weight-updated CL
%
%  INPUTS:
%    ADP          - Aircraft Design Parameters object (handle)
%                   Required fields: TLAR.M_c, TLAR.Alt_cruise, TLAR.Alt_max,
%                   TLAR.V_app, Engine.T_Static, Engine.TSFC(M,h),
%                   AeroPolar.CD(CL), AeroPolar.CD0, AeroPolar.Beta,
%                   WingArea, MTOM
%    tripRange_m  - Design mission range [m]
%    M_TO_in      - (optional) take-off mass [kg]; default = ADP.MTOM
%
%  OUTPUTS:
%    BlockFuel    - Total block fuel [kg]
%    TripFuel     - Trip fuel: taxi through to-gate [kg]
%    ResFuel      - Reserve fuel: alternate + loiter + contingency [kg]
%    Mf_TOC       - Mass fraction at top of climb [-]  (W_TOC / W_TO)
%    MissionTime  - Block time [s]
%    cruise_FL    - Final cruise flight level [e.g. 390 = FL390]
%    detail       - Struct: per-segment fuel/time/dist + TTC + warnings

% ── Input handling ─────────────────────────────────────────────────────
if nargin < 3 || isempty(M_TO_in)
    M_TO = ADP.MTOM;
else
    M_TO = M_TO_in;
end

if ~isfinite(M_TO) || M_TO <= 0
    error('MissionAnalysisRefined: invalid M_TO = %.1f kg', M_TO);
end
if isempty(ADP.AeroPolar) || ~isfinite(ADP.AeroPolar.CD0) || ADP.AeroPolar.CD0 <= 0
    error('MissionAnalysisRefined: AeroPolar not set. Call UpdateAero first.');
end

% Physical constants 
G       = 9.80665;      % [m/s²]
[RHO_SL,~,~,~,~,~,~]  = Boxwing.cast.atmos(0);      % [kg/m³] ISA SL density

% Unpack ADP 
M_c    = ADP.TLAR.M_c;
h_ICA  = ADP.TLAR.Alt_cruise;        % [m] initial cruise altitude
h_ceil = ADP.TLAR.Alt_max;           % [m] operational ceiling
V_app  = ADP.TLAR.V_app;             % [m/s] approach speed
S_ref  = ADP.WingArea;               % [m²]
polar  = ADP.AeroPolar;
eng    = ADP.Engine;
T_SLS  = 2 * ADP.T_Static;           % [N] total SLS thrust (2 engines)

% Mission parameters 
DH_CLIMB   = SI.ft *500;    % [m] climb segment step
DH_DESCENT = SI.ft *500;    % [m] descent segment step
DH_STEP    = SI.ft *2000;    % [m] step-climb increment
V_CAS_LIM  = SI.knt *250;  % [m/s] CAS limit below FL100
H_SPD_LIM  = SI.ft *10000;    % [m]   altitude of speed restriction (FL100)
H_INIT     = SI.ft *1500;    % [m]   end of takeoff / start of main climb
H_LOITER   = SI.ft *1500;    % [m]   loiter altitude
ALT_RANGE  = 350e3;           % [m]   alternate mission range
ROD_MS     = SI.ft *1800 /60; % [m/s] target rate of descent (~9.1 m/s)
IDLE_FRAC  = 0.05;            % idle thrust = 5% of SLS × density lapse
IDLE_TSFC_MULT = 2.5;         % TSFC at idle ≈ 2.5 × engine model at that M,h
ROC_STEP_MIN = SI.ft *300 /60;% [m/s] min ROC for step-climb eligibility

% State initialisation 
W     = M_TO * G;    % current aircraft weight [N]
mf    = 0;           % cumulative fuel mass burned [kg]
t_tot = 0;           % cumulative time [s]
d_tot = 0;           % cumulative distance [m]
warns = {};
segs  = struct('name',{},'fuel',{},'time',{},'dist',{});
TTC_s = NaN;

%% 1  TAXI --> 20 min at idle
t_taxi  = 20 * 60; % [s]
T_id_sl = IDLE_FRAC * T_SLS;
mf_taxi = T_id_sl * eng.TSFC(0.05, 0) * IDLE_TSFC_MULT * t_taxi;
[W, mf, t_tot] = Boxwing.script.MissionAnalysis.stateUpdate(W, mf, t_tot, mf_taxi, t_taxi, G);
segs = Boxwing.script.MissionAnalysis.logSeg(segs, 'Taxi', mf_taxi, t_taxi, 0);

%% 2  TAKE-OFF --> 1 min at max thrust (ground roll + initial climb to 1,500 ft) 
t_TO   = 60;                                    % [s]
mf_TO  = T_SLS * eng.TSFC(0.25, 0) * t_TO;
[W, mf, t_tot] = Boxwing.script.MissionAnalysis.stateUpdate(W, mf, t_tot, mf_TO, t_TO, G);
d_tot  = d_tot + 400;                           % ~400 m ground roll
segs   = Boxwing.script.MissionAnalysis.logSeg(segs, 'Takeoff', mf_TO, t_TO, 400);

%% 3  CLIMB --> 1,500 ft - ICA  (500 ft segments)
[mf_cl, t_cl, dx_cl, W, TTC_s] = Boxwing.script.MissionAnalysis.doClimb(W, H_INIT, h_ICA, DH_CLIMB, ...
    M_c, S_ref, T_SLS, eng, polar, V_CAS_LIM, H_SPD_LIM, G, RHO_SL);

mf    = mf    + mf_cl;
t_tot = t_tot + t_cl;
d_tot = d_tot + dx_cl;
segs  = Boxwing.script.MissionAnalysis.logSeg(segs, 'Climb', mf_cl, t_cl, dx_cl);
Mf_TOC = W / (M_TO * G);   % mass fraction at top of climb

if TTC_s > 30 * 60
    warns{end+1} = sprintf('SPEC VIOLATION: TTC = %.1f min > 30 min limit', TTC_s/60);
end

%% 4  CRUISE --> step-climb, Breguet per 200 km chunk
% Estimate descent ground distance to compute net cruise range.
% Descent from h_ICA at ROD_MS target ≈ TAS_ICA × Δh / ROD_MS.
[~, a_ica,~,~,~,~,~,] = Boxwing.cast.atmos(h_ICA); 
TAS_ica      = M_c * a_ica;
dx_desc_est  = TAS_ica * (h_ICA - H_INIT) / ROD_MS;

R_cruise = tripRange_m - dx_cl - dx_desc_est;
if R_cruise < 20e3
    R_cruise = 20e3;
    warns{end+1} = 'Trip range very short: cruise clamped to 20 km';
end

[mf_cr, t_cr, dx_cr, h_cr_end, W, warns_cr] = Boxwing.script.MissionAnalysis.doCruise(W, h_ICA, h_ceil, R_cruise, DH_STEP, ...
    M_c, S_ref, T_SLS, eng, polar, ROC_STEP_MIN, G, RHO_SL);

warns   = [warns, warns_cr];
mf      = mf    + mf_cr;
t_tot   = t_tot + t_cr;
d_tot   = d_tot + dx_cr;
segs    = Boxwing.script.MissionAnalysis.logSeg(segs, 'Cruise', mf_cr, t_cr, dx_cr);
cruise_FL = round(h_cr_end / SI.ft / 100);

%% 5  DESCENT --> cruise alt → 1,500 ft  (500 ft segments)
[mf_de, t_de, dx_de, W] = Boxwing.script.MissionAnalysis.doDescent(W, h_cr_end, H_INIT, DH_DESCENT, ...
    M_c, S_ref, T_SLS, IDLE_FRAC, IDLE_TSFC_MULT, eng, polar, ROD_MS, ...
    V_CAS_LIM, H_SPD_LIM, G, RHO_SL);

mf    = mf    + mf_de;
t_tot = t_tot + t_de;
d_tot = d_tot + dx_de;
segs  = Boxwing.script.MissionAnalysis.logSeg(segs, 'Descent', mf_de, t_de, dx_de);

%% 6  APPROACH --> 5 min at approach conditions
t_app  = 5 * 60;
[rho_a, a_a,~,~,~,~,~] = Boxwing.cast.atmos(SI.ft * 500); 
M_app  = V_app / a_a;
q_app  = 0.5 * rho_a * V_app^2;
CL_app = Boxwing.script.MissionAnalysis.clamp(W / (q_app * S_ref), 0.2, 2.5);
D_app  = q_app * S_ref * polar.CD(CL_app);
T_app  = D_app * 1.05;        % slight thrust above drag during approach
mf_app = eng.TSFC(M_app, SI.ft * 500) * T_app * t_app;
[W, mf, t_tot] = Boxwing.script.MissionAnalysis.stateUpdate(W, mf, t_tot, mf_app, t_app, G);
d_tot  = d_tot + V_app * t_app;
segs   = Boxwing.script.MissionAnalysis.logSeg(segs, 'Approach', mf_app, t_app, V_app * t_app);

%% 7  TO GATE --> 20 min at idle
t_gate  = 20 * 60;
mf_gate = IDLE_FRAC * T_SLS * eng.TSFC(0.05, 0) * IDLE_TSFC_MULT * t_gate;
[W, mf, t_tot] = Boxwing.script.MissionAnalysis.stateUpdate(W, mf, t_tot, mf_gate, t_gate, G);
segs = Boxwing.script.MissionAnalysis.logSeg(segs, 'ToGate', mf_gate, t_gate, 0);

%% TRIP FUEL
TripFuel = mf;

%% LOITER FUEL RATE
[rho_lo, a_lo,~,~,~,~,~] = Boxwing.cast.atmos(H_LOITER);
% CL at maximum L/D: CL_md = sqrt(CD0 / Beta) where Beta = 1/(pi*AR*e)
CL_md       = Boxwing.script.MissionAnalysis.clamp(sqrt(polar.CD0 / polar.Beta), 0.3, 1.5);
CD_md       = polar.CD(CL_md);
V_md        = sqrt(2 * W / (rho_lo * S_ref * CL_md));   % [m/s]
M_lo        = Boxwing.script.MissionAnalysis.clamp(V_md / a_lo, 0.05, 0.45);
D_md        = 0.5 * rho_lo * V_md^2 * S_ref * CD_md;   % = T_lo in level flt
mf_lo_rate  = eng.TSFC(M_lo, H_LOITER) * D_md;         % [kg/s]

%% 8  CONTINGENCY --> max(5 min loiter, 3% trip fuel)
mf_cont = max(mf_lo_rate * 5*60,  0.03 * TripFuel);
segs = Boxwing.script.MissionAnalysis.logSeg(segs, 'Contingency', mf_cont, 0, 0);

%% 9  ALTERNATE MISSION --> 350 km full profile
%  Uses FL250 (or ceiling if lower) as alternate cruise altitude.
%  Same segmented climb/cruise/descent as main mission.

h_alt_cr = min(SI.ft * 25000, h_ceil);

% Alternate climb: H_LOITER → h_alt_cr
[mf_alt_cl, t_alt_cl, dx_alt_cl, W, ~] = Boxwing.script.MissionAnalysis.doClimb(W, H_LOITER, h_alt_cr, DH_CLIMB, ...
    M_c, S_ref, T_SLS, eng, polar, V_CAS_LIM, H_SPD_LIM, G, RHO_SL);

% Alternate descent distance estimate (same logic as main mission)
[~, a_altcr,~,~,~,~,~] = Boxwing.cast.atmos(h_alt_cr);
TAS_altcr    = M_c * a_altcr;
dx_alt_de_est = TAS_altcr * (h_alt_cr - H_LOITER) / ROD_MS;

% Alternate cruise (Breguet, single segment - short mission)
R_alt_cr = max(ALT_RANGE - dx_alt_cl - dx_alt_de_est, 10e3);
[rho_ac, a_ac,~,~,~,~,~] = Boxwing.cast.atmos(h_alt_cr);
TAS_ac = M_c * a_ac;
q_ac   = 0.5 * rho_ac * TAS_ac^2;
CL_ac  = Boxwing.script.MissionAnalysis.clamp(W / (q_ac * S_ref), 0.20, 1.20);
LD_ac  = CL_ac / polar.CD(CL_ac);
TSFC_ac = eng.TSFC(M_c, h_alt_cr);
frac_ac  = exp(-(R_alt_cr * G * TSFC_ac) / (TAS_ac * LD_ac));
mf_alt_cr = (W - W * frac_ac) / G;
t_alt_cr  = R_alt_cr / TAS_ac;
W         = W * frac_ac;

% Alternate descent: h_alt_cr → H_LOITER
[mf_alt_de, t_alt_de, dx_alt_de, W] = Boxwing.script.MissionAnalysis.doDescent(W, h_alt_cr, H_LOITER, DH_DESCENT, ...
    M_c, S_ref, T_SLS, IDLE_FRAC, IDLE_TSFC_MULT, eng, polar, ...
    ROD_MS, V_CAS_LIM, H_SPD_LIM, G, RHO_SL);

mf_alt = mf_alt_cl + mf_alt_cr + mf_alt_de;
t_alt  = t_alt_cl  + t_alt_cr  + t_alt_de;
t_tot  = t_tot + t_alt;
segs   = Boxwing.script.MissionAnalysis.logSeg(segs, 'Alternate', mf_alt, t_alt, ALT_RANGE);

%% 10 LOITER --> 30 min at V_md at 1,500 ft
t_loiter = 30 * 60;
% Recompute V_md at final (lighter) weight after alternate
V_md2    = sqrt(2 * W / (rho_lo * S_ref * CL_md));
M_lo2    = Boxwing.script.MissionAnalysis.clamp(V_md2 / a_lo, 0.05, 0.45);
D_md2    = 0.5 * rho_lo * V_md2^2 * S_ref * CD_md;   % T = D in level flight
mf_loit  = eng.TSFC(M_lo2, H_LOITER) * D_md2 * t_loiter;
[W, ~, t_tot] = Boxwing.script.MissionAnalysis.stateUpdate(W, 0, t_tot, mf_loit, t_loiter, G);
segs = Boxwing.script.MissionAnalysis.logSeg(segs, 'Loiter', mf_loit, t_loiter, 0);

%% FINAL OUTPUTS
ResFuel     = mf_alt + mf_loit + mf_cont;
BlockFuel   = TripFuel + ResFuel;
MissionTime = t_tot;

if ~isfinite(BlockFuel) || BlockFuel <= 0 || BlockFuel >= M_TO
    error('MissionAnalysisRefined: BlockFuel = %.1f kg is non-physical.', BlockFuel);
end

% Detail struct for diagnostics
detail = struct();
detail.BlockFuel      = BlockFuel;
detail.TripFuel       = TripFuel;
detail.ResFuel        = ResFuel;
detail.AlternateFuel  = mf_alt;
detail.LoiterFuel     = mf_loit;
detail.ContFuel       = mf_cont;
detail.TTC_s          = TTC_s;
detail.TTC_min        = TTC_s / 60;
detail.CruiseAlt_m    = h_cr_end;
detail.CruiseFL       = cruise_FL;
detail.Mf_TOC         = Mf_TOC;
detail.Mf_Fuel        = BlockFuel / M_TO;
detail.MissionTime_hr = MissionTime / 3600;
detail.RangeFlown_m   = d_tot;
detail.segments       = segs;
detail.warnings       = warns;

% Surface any spec violations as MATLAB warnings
for i = 1:numel(warns)
    warning('MissionAnalysisRefined:specViolation', '%s', warns{i});
end

end % main function 




