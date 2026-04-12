%% Boxwing.B777.CD0  Full aircraft parasite drag buildup
%  Wing:      Michel transition criterion + Schlichting mixed Cf + Raymer FF
%  Fuselage:  Raymer body-of-revolution FF (eq 12.31) + wetted area (eq 12.11)
%  Nacelles:  Raymer nacelle FF (eq 12.32) + Q=1.3 wing-mounted interference
%  Tail:      Same flat-plate method as wing (HTP + VTP)
%  Misc:      3% of total (leakage, gaps, antennas) -- Raymer ch12 guideline
%
%  FIX (2025-01): getCf was crashing with "Function values at interval
%  endpoints must be finite and real" when called before sizing converges
%  (Re very small → michel_res sign check could NaN out).
%  Fix: clamp Re to a physically sensible floor before the fzero call and
%  use a safe fallback (full turbulent Cf) when transition search fails.
%
%  REF: Raymer, "Aircraft Design: A Conceptual Approach", Ch.12
%
%  CALL:
%    [CD0, breakdown] = Boxwing.B777.CD0(ADP)

function [CD0_total, breakdown] = CD0(obj, B7Geom, verbose)
    if nargin < 2; B7Geom  = [];    end
    if nargin < 3; verbose = false; end

    % =========================================================
    % ATMOSPHERE / CRUISE CONDITION
    % =========================================================
    [rho, a, ~, ~, mu] = Boxwing.cast.atmos(obj.TLAR.Alt_cruise);
    V = obj.TLAR.M_c * a;
    M = obj.TLAR.M_c;

    % =========================================================
    % SHARED GEOMETRY
    % =========================================================
    S_ref = obj.WingArea;
    b     = obj.Span;
    AR    = b^2 / S_ref;

    if isprop(obj,'MAC') && ~isempty(obj.MAC) && isscalar(obj.MAC) && isfinite(obj.MAC) && obj.MAC > 0
        c_ref = obj.MAC;
    else
        c_ref = S_ref / max(b, 1);
    end

    if isprop(obj,'tc') && ~isempty(obj.tc) && isfinite(obj.tc)
        tc = obj.tc;
    else
        tc = 0.14;
    end

    if isprop(obj,'Sweep25') && ~isempty(obj.Sweep25) && isfinite(obj.Sweep25)
        Lambda_deg = obj.Sweep25 - 2.0;
    else
        Lambda_deg = 30.0;
    end
    Lambda_rad = deg2rad(Lambda_deg);

    L_fuse = obj.CockpitLength + obj.CabinLength + 1.48*2*obj.CabinRadius;
    d_fuse = 2 * obj.CabinRadius;

    % =========================================================
    % LOCAL HELPER: mixed Cf with Michel transition + Eckert compressibility
    %
    % FIX: protect fzero against non-finite bracket values that arise when
    %   Re is tiny (early sizing iterations, or near-zero geometry inputs).
    %   Strategy:
    %     1. Clamp Re to >= 1e5  (below this the Michel criterion is
    %        meaningless — flow is fully laminar, use laminar Cf anyway).
    %     2. Evaluate Michel function at both bracket ends; if either is
    %        NaN/Inf, or both have the same sign, skip fzero and fall back
    %        to the full-turbulent estimate (conservative).
    % =========================================================
    function Cf = getCf(Re, M_loc)
        % Clamp Re to a floor to avoid divide-by-zero / NaN in Michel fn
        Re = max(Re, 1e5);

        % Schlichting full-turbulent
        Cf_turb = 0.455 / (log10(Re))^2.58;

        % Michel criterion for transition Reynolds number
        michel_res = @(Re_x) 0.664.*Re_x.^0.5 ...
                           - 1.174.*(1 + 22400./Re_x).*Re_x.^0.46;

        lo = 1e4;
        hi = Re;
        f_lo = michel_res(lo);
        f_hi = michel_res(hi);

        % Safe fzero: only attempt if both endpoints are finite and bracket
        Re_x_tr = Re;   % default: turbulent from root
        if isfinite(f_lo) && isfinite(f_hi) && (sign(f_lo) ~= sign(f_hi))
            try
                Re_x_tr = fzero(michel_res, [lo, hi]);
            catch
                Re_x_tr = Re;   % fallback to full turbulent
            end
        elseif isfinite(f_lo) && f_lo > 0
            % Both same sign but lo is positive → transition somewhere near lo
            try
                Re_x_tr = fzero(michel_res, [lo, min(hi, lo*100)]);
            catch
                Re_x_tr = Re;
            end
        end
        Re_x_tr = max(min(Re_x_tr, Re), lo);

        % A-factor (Schlichting table)
        Re_tr_table = [3e5,  5e5,  3e6,   1e7  ];
        A_table     = [1050, 1700, 8700,  27000 ];
        A = max(interp1(Re_tr_table, A_table, Re_x_tr, 'linear', 'extrap'), 0);

        % Mixed incompressible Cf
        Cf_i = max(Cf_turb - A/Re, 1e-6);

        % Eckert reference temperature compressibility correction
        Cf = Cf_i / (1 + 0.144*M_loc^2)^0.65;
    end

    % =========================================================
    % 1. WING CD0
    % =========================================================
    Re_wing = rho * V * c_ref / mu;
    Cf_wing = getCf(Re_wing, M);

    x_tc    = 0.37;
    FF_wing = (1 + (0.6/x_tc)*tc + 100*tc^4) * ...
              (1.34 * M^0.18 * cos(Lambda_rad)^0.28);

    S_wet_wing = 2.0 * S_ref * (1 + 0.2*tc);
    Q_wing     = 1.05;   % slight interference for boxwing tip junctions

    CD0_wing = Cf_wing * FF_wing * Q_wing * (S_wet_wing/S_ref);

    % =========================================================
    % 2. FUSELAGE CD0
    % =========================================================
    f_ratio  = L_fuse / max(d_fuse, 0.1);
    FF_fuse  = 1 + 60/f_ratio^3 + f_ratio/400;

    S_wet_fuse = (pi * d_fuse * L_fuse) * ...
                 (1 - 2/f_ratio)^(2/3) * (1 + 1/f_ratio^2);

    Re_fuse  = rho * V * L_fuse / mu;
    Cf_fuse  = getCf(Re_fuse, M);

    CD0_fuse = Cf_fuse * FF_fuse * 1.0 * (S_wet_fuse/S_ref);

    % =========================================================
    % 3. NACELLE CD0
    % =========================================================
    if isprop(obj,'Engine') && ~isempty(obj.Engine) && ...
       isfield(obj.Engine,'Diameter') && isfinite(obj.Engine.Diameter)
        d_nac = obj.Engine.Diameter;
        L_nac = obj.Engine.Length;
    elseif isprop(obj,'Engine') && ~isempty(obj.Engine) && ...
       isprop(obj.Engine,'Diameter') && isfinite(obj.Engine.Diameter)
        d_nac = obj.Engine.Diameter;
        L_nac = obj.Engine.Length;
    else
        T_total  = obj.ThrustToWeightRatio * obj.MTOM * 9.81;
        T_engine = max(T_total / 2, 1e3);
        d_nac    = 0.033 * sqrt(T_engine/1000);
        L_nac    = 1.5 * d_nac;
    end

    f_nac     = L_nac / max(d_nac, 0.1);
    FF_nac    = 1 + 0.35/f_nac;
    S_wet_nac = pi * d_nac * L_nac;
    Re_nac    = rho * V * L_nac / mu;
    Cf_nac    = getCf(Re_nac, M);
    n_eng     = 2;

    CD0_nac = Cf_nac * FF_nac * 1.30 * (n_eng * S_wet_nac / S_ref);

    % =========================================================
    % 4. TAIL CD0  (VTP only — boxwing has no HTP)
    % =========================================================
    tc_tail         = 0.10;
    x_tc_tail       = 0.30;
    Lambda_tail_rad = deg2rad(25.0);

    FF_tail = (1 + (0.6/x_tc_tail)*tc_tail + 100*tc_tail^4) * ...
              (1.34 * M^0.18 * cos(Lambda_tail_rad)^0.28);

    % HTP: boxwing has V_HT=0, so HtpArea = 0 → CD0_HTP = 0
    if isprop(obj,'HtpArea') && ~isempty(obj.HtpArea) && isfinite(obj.HtpArea) && obj.HtpArea > 0
        S_HTP     = obj.HtpArea;
        S_wet_HTP = 2.0 * S_HTP * (1 + 0.2*tc_tail);
        Re_HTP    = rho * V * sqrt(S_HTP) / mu;
        Cf_HTP    = getCf(Re_HTP, M);
        CD0_HTP   = Cf_HTP * FF_tail * 1.05 * (S_wet_HTP/S_ref);
    else
        CD0_HTP = 0;
    end

    % VTP
    if isprop(obj,'VtpArea') && ~isempty(obj.VtpArea) && isfinite(obj.VtpArea) && obj.VtpArea > 0
        S_VTP     = obj.VtpArea;
        S_wet_VTP = 2.0 * S_VTP * (1 + 0.2*tc_tail);
        Re_VTP    = rho * V * sqrt(S_VTP) / mu;
        Cf_VTP    = getCf(Re_VTP, M);
        CD0_VTP   = Cf_VTP * FF_tail * 1.05 * (S_wet_VTP/S_ref);
    else
        % Estimate from volume coefficient
        L_VT  = max(obj.RearWingPos - obj.FrontWingPos, 1);
        S_VTP = obj.V_VT * S_ref * b / L_VT;
        if isfinite(S_VTP) && S_VTP > 0
            S_wet_VTP = 2.0 * S_VTP * (1 + 0.2*tc_tail);
            Re_VTP    = rho * V * sqrt(S_VTP) / mu;
            Cf_VTP    = getCf(Re_VTP, M);
            CD0_VTP   = Cf_VTP * FF_tail * 1.05 * (S_wet_VTP/S_ref);
        else
            CD0_VTP = 0;
        end
    end

    CD0_tail = CD0_HTP + CD0_VTP;

    % =========================================================
    % 5. MISCELLANEOUS DRAG (Raymer ch12: 3%)
    % =========================================================
    CD0_subtotal = CD0_wing + CD0_fuse + CD0_nac + CD0_tail;
    CD0_misc     = 0.03 * CD0_subtotal;

    % =========================================================
    % 6. TOTAL
    % =========================================================
    CD0_total = CD0_subtotal + CD0_misc;

    % Sanity guard
    if ~isfinite(CD0_total) || CD0_total <= 0
        warning('Boxwing.B777.CD0: non-finite result, returning seed 0.018');
        CD0_total = 0.018;
    end

    % =========================================================
    % OUTPUT STRUCT
    % =========================================================
    breakdown.CD0_wing   = CD0_wing;
    breakdown.CD0_fuse   = CD0_fuse;
    breakdown.CD0_nac    = CD0_nac;
    breakdown.CD0_HTP    = CD0_HTP;
    breakdown.CD0_VTP    = CD0_VTP;
    breakdown.CD0_tail   = CD0_tail;
    breakdown.CD0_misc   = CD0_misc;
    breakdown.CD0_total  = CD0_total;
    breakdown.Cf_wing    = Cf_wing;
    breakdown.FF_wing    = FF_wing;
    breakdown.Re_wing    = Re_wing;
    breakdown.S_wet_wing = S_wet_wing;
    breakdown.S_wet_fuse = S_wet_fuse;
    breakdown.f_ratio    = f_ratio;
    breakdown.FF_fuse    = FF_fuse;

    % =========================================================
    % PRINT SUMMARY (only when verbose=true, i.e. NOT during sizing loop)
    % =========================================================
    if verbose
        fprintf('\n==============================================\n');
        fprintf('  Boxwing.B777.CD0 FULL BUILDUP\n');
        fprintf('==============================================\n');
        fprintf('  Cruise M    = %.3f\n',    M);
        fprintf('  c_ref       = %.3f m\n',  c_ref);
        fprintf('  Re_wing     = %.3e\n',    Re_wing);
        fprintf('  WING   CD0  = %.5f\n',   CD0_wing);
        fprintf('  FUSE   CD0  = %.5f\n',   CD0_fuse);
        fprintf('  NACS   CD0  = %.5f\n',   CD0_nac);
        fprintf('  TAIL   CD0  = %.5f\n',   CD0_tail);
        fprintf('  MISC   CD0  = %.5f\n',   CD0_misc);
        fprintf('  TOTAL  CD0  = %.5f\n',   CD0_total);
        fprintf('==============================================\n\n');
    end
end