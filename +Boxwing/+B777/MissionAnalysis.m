function [BlockFuel, TripFuel, ResFuel, Mf_TOC, MissionTime, cruise_FL] = ...
         MissionAnalysis(ADP, tripRange, M_TO)
%MISSIONANALYSIS  Fuel burn for the boxwing freighter mission.
%
%  FIXES vs previous version:
%   1. Cruise CL uses WingLoading (bounded) not raw MTOM*g/WingArea
%      — prevents CL >> 1 when MTOM snowballs mid-loop
%   2. LD guard threshold raised from 1 to 5 (LD=1.1 is still nonphysical)
%   3. Alternate section: M_alt already corrected in your branch; kept as-is
%   4. Loiter CL also uses WingLoading for consistency

arguments
    ADP
    tripRange
    M_TO = ADP.MTOM;
end

if ~isfinite(M_TO) || M_TO <= 0
    error('MissionAnalysis: M_TO = %.1f is invalid.', M_TO);
end

EWF = 1.0;
fs  = zeros(1,4);
ts  = zeros(1,4);

%% ---- CRUISE ----
alt = ADP.TLAR.Alt_cruise;
[rho, a, ~, ~] = BoxWing.cast.atmos(alt);
M_cruise = ADP.TLAR.M_c;

% FIX 1: use WingLoading (bounded to 5500-8500 N/m²) not raw MTOM/Area
% Old: CL_c = (EWF*M_TO*9.81) / (0.5*rho*(a*M_cruise)^2 * ADP.WingArea)
%   → when MTOM snowballs to 3000t but WingArea only 600m², CL_c = 6.7
% Fixed: derive CL from the W/S that ConstraintAnalysis already bounded
WS_current = ADP.WingLoading;   % [N/m²] clamped by Size.m to 5500-8500
q_cr = 0.5 * rho * (a * M_cruise)^2;
Mf_TOC_safe = max(min(ADP.Mf_TOC, 1.0), 0.90);
CL_c = (WS_current * Mf_TOC_safe) / q_cr;
CL_c = max(0.30, min(CL_c, 0.80));   % physical clamp

CD_c = ADP.AeroPolar.CD(CL_c);
LD_c = CL_c / CD_c;

% FIX 2: tighter LD guard (LD=2 is still nonphysical for cruise)
if ~isfinite(LD_c) || LD_c <= 5
    error('MissionAnalysis: nonphysical L/D at cruise (LD=%.3g). CD0=%.4f AR=%.2f CL=%.3f', ...
        LD_c, ADP.AeroPolar.CD0, ADP.AeroPolar.AR, CL_c);
end

cruise_FL  = round(alt * SI.ft / 1e2, 0);
TSFC_c     = ADP.Engine.TSFC(M_cruise, alt);
V          = M_cruise * a;

if ~isfinite(TSFC_c) || TSFC_c <= 0
    error('MissionAnalysis: nonphysical TSFC (%.3g). Check engine model.', TSFC_c);
end

fs(1) = exp(-(tripRange * 9.81 * TSFC_c) / (V * LD_c));
ts(1) = tripRange / V;
EWF   = EWF * fs(1);

if ~isfinite(fs(1)) || fs(1) <= 0 || fs(1) >= 1
    error('MissionAnalysis: cruise fuel fraction = %.4f is nonphysical.', fs(1));
end

%% ---- ALTERNATE ----
altA = ADP.TLAR.Alt_alternate;
[rhoA, aA, ~, ~] = BoxWing.cast.atmos(altA);
M_alt = 0.4;    % low-speed alternate
VA    = M_alt * aA;

% Use WingLoading for alternate CL too
CL_a = (WS_current * EWF) / (0.5 * rhoA * VA^2);
CL_a = max(0.30, min(CL_a, 1.20));   % alternate can be higher CL
CD_a = ADP.AeroPolar.CD(CL_a);
LD_a = CL_a / CD_a;
if ~isfinite(LD_a) || LD_a <= 1
    error('MissionAnalysis: nonphysical L/D on alternate (LD=%.3g).', LD_a);
end

TSFC_a = ADP.Engine.TSFC(M_alt, altA);
fs(2)  = exp(-(ADP.TLAR.Range_alternate * 9.81 * TSFC_a) / (VA * LD_a));
ts(2)  = ADP.TLAR.Range_alternate / VA;
EWF    = EWF * fs(2);

%% ---- LOITER ----
[rho0, a0, ~, ~] = BoxWing.cast.atmos(0);
V_loit = 150;   % [m/s]
Mach_l = V_loit / a0;

% FIX: use WingLoading for loiter CL too
CL_l = (WS_current * EWF) / (0.5 * rho0 * V_loit^2);
CL_l = max(0.30, min(CL_l, 1.20));
CD_l   = ADP.AeroPolar.CD(CL_l);
LD_l   = CL_l / CD_l;
TSFC_l = ADP.Engine.TSFC(Mach_l, 0);

fs(3) = exp(-(ADP.TLAR.Loiter * 9.81 * TSFC_l) / LD_l);
ts(3) = ADP.TLAR.Loiter;
EWF   = EWF * fs(3);

%% ---- CONTINGENCY 3% ----
df    = (1 - EWF) * 0.03;
fs(4) = 1 - df / EWF;
ts(4) = 5 * 60;
EWF   = EWF * fs(4);

%% ---- OUTPUTS ----
BlockFuel        = (1 - EWF) * M_TO;
TripFuel         = (1 - fs(1)) * M_TO;
W_after_cruise   = fs(1) * M_TO;
ResFuel          = (1 - prod(fs(2:end))) * W_after_cruise;
Mf_TOC           = 0.98;   % stable placeholder (climb not explicitly modelled)
MissionTime      = ts(1);

if ~isfinite(BlockFuel) || BlockFuel <= 0 || BlockFuel >= M_TO
    error('MissionAnalysis: BlockFuel = %.1f kg is nonphysical (MTOM=%.1f kg).', BlockFuel, M_TO);
end

end