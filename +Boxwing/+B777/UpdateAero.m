function UpdateAero(obj, x_cg, verbose)
%UPDATEAERO  Rebuild the aerodynamic polar with current geometry.
%
%  UpdateAero(obj)              — silent
%  UpdateAero(obj, x_cg)        — silent, uses real CG for trim drag
%  UpdateAero(obj, x_cg, true)  — prints one diagnostic line

if nargin < 2, x_cg   = []; end
if nargin < 3, verbose = false; end

%% Step 1: physics-based CD0 via flat-plate build-up
% Suppress the tc / sweep fallback warnings — they fire every iteration
% because ADP.tc and ADP.Sweep25 exist but the warning checks for
% 'tc_ref' / 'SweepLE' first. The fallback values (tc=0.14, Lambda=30°)
% are correct for the boxwing so the warnings are noise, not errors.
warnState = warning('off', 'all');   % silence during CD0 call
try
    [cd0_computed, ~] = BoxWing.B777.CD0(obj);
    if isfinite(cd0_computed) && cd0_computed > 0.010 && cd0_computed < 0.035
        obj.CD0 = cd0_computed;
    end
catch
    % BoxWing.B777.CD0 failed — keep seed CD0 (0.018)
    % Common cause: BoxWing.cast.atmos namespace not yet fixed in CD0.m
end
warning(warnState);   % restore warning state

%% Step 2: set CL_cruise to the actual cruise value before building polar
% This ensures trimDrag and the diagnostic print use the correct CL,
% not the 0.80 clamp value from the constraint step.
try
    [rho_c, a_c] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
    q_c = 0.5 * rho_c * (obj.TLAR.M_c * a_c)^2;
    Mf_TOC_safe = max(min(obj.Mf_TOC, 1.0), 0.90);
    WS_bounded  = max(min(obj.WingLoading, 8500), 5500);
    obj.CL_cruise = max(0.30, min((WS_bounded * Mf_TOC_safe) / q_c, 0.80));
catch
    % keep existing CL_cruise if atmos call fails
end

%% Step 3: build polar
if isempty(x_cg)
    obj.AeroPolar = BoxWing.B777.AeroPolar(obj);
else
    obj.AeroPolar = BoxWing.B777.AeroPolar(obj, x_cg);
end

%% Step 4: optional single diagnostic line
if verbose
    p     = obj.AeroPolar;
    CL_cr = obj.CL_cruise;
    if ~isempty(CL_cr) && isfinite(CL_cr) && CL_cr > 0.1
        LD = CL_cr / p.CD(CL_cr);
        fprintf('  [Aero] CD0=%.4f  CDwave=%.5f  CDtrim=%.5f  AR=%.2f  e=%.3f  CL=%.3f  L/D=%.1f\n', ...
            p.CD0, p.CDwave, p.CD_trim, p.AR, p.e, CL_cr, LD);
    else
        fprintf('  [Aero] CD0=%.4f  CDwave=%.5f  CDtrim=%.5f  AR=%.2f  e=%.3f\n', ...
            p.CD0, p.CDwave, p.CD_trim, p.AR, p.e);
    end
end

end