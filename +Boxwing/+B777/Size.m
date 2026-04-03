function [obj, out] = Size(obj, verbose)
%SIZE  Iteratively size the boxwing until MTOM converges.

if nargin < 2
    verbose = true;
end

delta    = inf;
iter     = 0;
MAX_ITER = 120;
relax    = 0.30;
tol      = 200;
MTOM_CAP = 15 * obj.TLAR.Payload;

if verbose
    fprintf('  Iter |   MTOM (t) |  OEM (t) | Fuel (t) |  W/S  |  delta (kg)\n');
    fprintf('  -----|------------|----------|----------|-------|------------\n');
end

% Fixed geometry from AR_target and span
b_eff   = obj.EffectiveSpan;
AR_tgt  = obj.AR_target;
S_fixed = b_eff^2 / AR_tgt;

obj.WingArea         = S_fixed;
obj.TotalLiftingArea = S_fixed;
obj.FrontWingArea    = S_fixed * 0.55;
obj.RearWingArea     = S_fixed * 0.45;
obj.Span             = b_eff;

[rho_c, a_c] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
q_c    = 0.5 * rho_c * (obj.TLAR.M_c * a_c)^2;
rho_sl = 1.225;

BlockFuel = 0;
TripFuel  = 0;
ResFuel   = 0;
Mf_TOC    = obj.Mf_TOC;

while delta > tol && iter < MAX_ITER
    iter = iter + 1;

    %% Wing loading from landing constraint (upper bound only)
    V_stall     = obj.TLAR.V_app / 1.30;
    CL_max_land = max(obj.CL_max + obj.Delta_Cl_ld, 2.5);
    WS_max_land = (0.5 * rho_sl * V_stall^2 * CL_max_land) / max(obj.Mf_Ldg, 0.60);
    WS_actual   = obj.MTOM * 9.81 / S_fixed;
    WS          = max(min(WS_actual, WS_max_land), 4000);

    %% T/W
    Mf_TOC_safe = max(min(obj.Mf_TOC, 1.0), 0.90);
    CL_c = max(0.35, min((WS * Mf_TOC_safe) / q_c, 0.75));

    if ~isempty(obj.AeroPolar)
        CD_c = obj.AeroPolar.CD(CL_c);
    else
        CD_c = obj.CD0 + CL_c^2 / (pi * AR_tgt * obj.e);
    end
    TW_cr = (q_c * CD_c / WS) * 1.10;
    TW    = max(0.20, min(TW_cr / (rho_c / rho_sl)^0.75, 0.45));

    obj.WingLoading         = WS;
    obj.ThrustToWeightRatio = TW;

    %% Enforce fixed geometry
    obj.WingArea         = S_fixed;
    obj.TotalLiftingArea = S_fixed;
    obj.FrontWingArea    = S_fixed * 0.55;
    obj.RearWingArea     = S_fixed * 0.45;
    obj.Span             = b_eff;
    obj.Thrust           = TW * obj.MTOM * 9.81;

    %% Geometry + CG
    [~, BWMass] = BoxWing.B777.BuildGeometry(obj);
    all_x = cellfun(@(x) x(1), {BWMass.X});
    all_m = [BWMass.m];
    x_cg  = sum(all_m .* all_x) / sum(all_m);

    %% CL_cruise before polar
    CL_cruise_new = (obj.MTOM * 9.81 * Mf_TOC_safe) / (q_c * S_fixed);
    obj.CL_cruise = max(0.30, min(CL_cruise_new, 0.80));

    %% Update aero (silent inside loop)
    BoxWing.B777.UpdateAero(obj, x_cg, false);

    %% Mission analysis
    try
        [BlockFuel, TripFuel, ResFuel, Mf_TOC, ~] = ...
            BoxWing.B777.MissionAnalysis(obj, obj.TLAR.Range, obj.MTOM);
    catch ME
        fprintf('  FAILED at iter %d: %s\n', iter, ME.message);
        break;
    end

    %% OEM
    allNames = cellfun(@(x) x, {BWMass.Name}, 'UniformOutput', false);
    isFuel   = cellfun(@(n) strcmp(n,'Fuel Front Wing') || strcmp(n,'Fuel Rear Wing'), allNames);
    isPay    = cellfun(@(n) strcmp(n,'Payload'), allNames);
    obj.OEM  = sum([BWMass(~isFuel & ~isPay).m]);

    %% MTOM closure
    mtom_new = obj.OEM + obj.TLAR.Payload + BlockFuel;
    delta    = abs(obj.MTOM - mtom_new);
    obj.MTOM = (1 - relax) * obj.MTOM + relax * mtom_new;

    if obj.MTOM > MTOM_CAP
        fprintf('  ERROR: MTOM (%.0f t) > cap (%.0f t).\n', obj.MTOM/1e3, MTOM_CAP/1e3);
        break;
    end

    obj.Mf_Fuel = BlockFuel / obj.MTOM;
    obj.Mf_TOC  = Mf_TOC;
    obj.Mf_Ldg  = max(0.55, min((obj.MTOM - TripFuel) / obj.MTOM, 0.90));
    obj.Mf_res  = max(0.02, min(ResFuel / obj.MTOM, 0.10));

    if verbose
        fprintf('  %4d | %10.1f | %8.1f | %8.1f | %5.0f | %11.1f\n', ...
                iter, obj.MTOM/1e3, obj.OEM/1e3, BlockFuel/1e3, WS, delta);
    end
end

%% Print final aero summary once
if verbose
    BoxWing.B777.UpdateAero(obj, x_cg, true);   % verbose=true for final print
    if delta <= tol
        fprintf('  Converged in %d iterations (delta=%.0f kg).\n\n', iter, delta);
    else
        fprintf('  WARNING: max iterations (%d), delta=%.0f kg\n', MAX_ITER, delta);
    end
end

out.BlockFuel = BlockFuel;
out.OEM       = obj.OEM;
out.MTOM      = obj.MTOM;
end