%  B777.CD0  Full aircraft parasite drag buildup
%  Wing:      Michel transition criterion + Schlichting mixed Cf + Raymer FF
%  Fuselage:  Raymer body-of-revolution FF (eq 12.31) + wetted area (eq 12.11)
%  Nacelles:  Raymer nacelle FF (eq 12.32) + Q=1.3 wing-mounted interference
%  Tail:      Same flat-plate method as wing (HTP + VTP)
%  Misc:      3% of total (leakage, gaps, antennas) -- Raymer ch12 guideline
%
%  REF: Raymer, "Aircraft Design: A Conceptual Approach", Ch.12
%
%  CALL:
%    ADP.CD0 = B777.CD0(ADP)             % TubeWing, no geometry
%    ADP.CD0 = B777.CD0(ADP, B7Geom)     % BoxWing with geometry
%    [CD0, breakdown] = B777.CD0(ADP, B7Geom)  % with full breakdown struct

function [CD0_total, breakdown] = CD0(obj, B7Geom)
    if nargin < 2
        B7Geom = [];
    end

    % =========================================================
    % ATMOSPHERE / CRUISE CONDITION
    % FIX: cast.atmos → BoxWing.cast.atmos
    %      atmos.m lives at +BoxWing/+cast/atmos.m so the correct
    % =========================================================
    [rho, a, ~, ~, mu] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
    V = obj.TLAR.M_c * a;
    M = obj.TLAR.M_c;

    % =========================================================
    % SHARED GEOMETRY
    % =========================================================
    S_ref = obj.WingArea;
    b     = obj.Span;
    AR    = b^2 / S_ref;

    % -- MAC --------------------------------------------------
    if isprop(obj,'MAC') && ~isempty(obj.MAC) && isfinite(obj.MAC)
        c_ref = obj.MAC;
    else
        c_ref = S_ref / b;
        warning('B777.CD0: MAC not found, using S/b = %.3f m', c_ref);
    end

    % -- Thickness ratio --------------------------------------
    if isprop(obj,'tc_ref') && ~isempty(obj.tc_ref) && isfinite(obj.tc_ref)
        tc = obj.tc_ref;
    elseif isprop(obj,'tc') && ~isempty(obj.tc) && isfinite(obj.tc) %in case name changes
        tc = obj.tc;
    else
        tc = 0.14;
        warning('B777.CD0: t/c not found, using fallback tc = %.2f', tc);
    end

    % -- Wing sweep -------------------------------------------
    if isprop(obj,'SweepLE') && ~isempty(obj.SweepLE) && isfinite(obj.SweepLE)
        Lambda_deg = obj.SweepLE - rad2deg(atan(2*tc / AR));
    elseif isprop(obj,'Sweep25') && ~isempty(obj.Sweep25) && isfinite(obj.Sweep25)
        Lambda_deg = obj.Sweep25 - 2.0;
    else
        Lambda_deg = 30.0;
        warning('B777.CD0: sweep not found, using fallback Lambda = %.1f deg', Lambda_deg);
    end
    Lambda_rad = deg2rad(Lambda_deg);

    % -- Fuselage geometry ------------------------------------
    L_fuse = obj.CockpitLength + obj.CabinLength + 1.48*2*obj.CabinRadius;
    d_fuse = 2 * obj.CabinRadius;

    % =========================================================
    % LOCAL HELPER: mixed Cf with Michel transition + Eckert compressibility
    % same method used for all components
    % =========================================================
    function Cf = getCf(Re, M_loc)
        % Schlichting full-turbulent
        Cf_turb = 0.455 / (log10(Re))^2.58;

        % Michel criterion for transition
        michel_res  = @(Re_x) 0.664.*Re_x.^0.5 ...
                            - 1.174.*(1 + 22400./Re_x).*Re_x.^0.46;
        f_lo = michel_res(1e4);
        f_hi = michel_res(Re);
        if sign(f_lo) ~= sign(f_hi)
            Re_x_tr = fzero(michel_res, [1e4, Re]);
        else
            Re_x_tr = fzero(michel_res, 1e7);
        end
        Re_x_tr = max(min(Re_x_tr, Re), 1e4);

        % A-factor (Schlichting table)
        Re_tr_table = [3e5,  5e5,  3e6,   1e7  ];
        A_table     = [1050, 1700, 8700,  27000 ];
        A    = max(interp1(Re_tr_table, A_table, Re_x_tr, 'linear','extrap'), 0);

        % mixed incompressible Cf
        Cf_i = max(Cf_turb - A/Re, 0);

        % Eckert reference temperature compressibility correction
        Cf  = Cf_i / (1 + 0.144*M_loc^2)^0.65;
    end

    % =========================================================
    % 1. WING CD0
    % Michel criterion + Raymer FF eq 12.30
    % FF = [1 + (0.6/x_tc)*tc + 100*tc^4] * [1.34*M^0.18*cos(L)^0.28]
    % =========================================================
    Re_wing = rho * V * c_ref / mu;
    Cf_wing = getCf(Re_wing, M);

    x_tc    = 0.37;   % SC(2)-0714: max thickness at 37% chord
    FF_wing = (1 + (0.6/x_tc)*tc + 100*tc^4) * ...
              (1.34 * M^0.18 * cos(Lambda_rad)^0.28);

    % wetted area -- class specific
    if isa(obj, 'B777.ADP_BW')
        if isprop(obj,'S_wet_front') && ~isempty(obj.S_wet_front)
            S_wet_wing = obj.S_wet_front + obj.S_wet_rear + obj.S_wet_fins;
        else
            S_wet_wing = 2*S_ref*(1+0.2*tc)*(1+obj.alphaArea);
            warning('B777.CD0: BW S_wet not found, using estimate');
        end
        Q_wing = 1.10;   % tip junction interference -- Raymer table 12.6
    else
        S_wet_wing = 2.0 * S_ref * (1 + 0.2*tc);   % Raymer eq 12.28
        Q_wing     = 1.0;
    end

    CD0_wing = Cf_wing * FF_wing * Q_wing * (S_wet_wing/S_ref);

    % =========================================================
    % 2. FUSELAGE CD0
    % Raymer eq 12.31: FF = 1 + 60/f^3 + f/400
    % Raymer eq 12.11: S_wet fuselage
    % =========================================================
    f_ratio  = L_fuse / d_fuse;                       % fineness ratio [-]
    FF_fuse  = 1 + 60/f_ratio^3 + f_ratio/400;        % Raymer eq 12.31

    % Raymer eq 12.11 -- wetted area streamlined body of revolution
    S_wet_fuse = (pi * d_fuse * L_fuse) * ...
                 (1 - 2/f_ratio)^(2/3) * (1 + 1/f_ratio^2);

    Re_fuse  = rho * V * L_fuse / mu;
    Cf_fuse  = getCf(Re_fuse, M);
    Q_fuse   = 1.0;   % no external junction for fuselage itself

    CD0_fuse = Cf_fuse * FF_fuse * Q_fuse * (S_wet_fuse/S_ref);

    % =========================================================
    % 3. NACELLE CD0
    % Raymer eq 12.32: FF = 1 + 0.35/f_nac
    % Q = 1.3 wing-mounted podded engines -- Raymer table 12.6
    % =========================================================
    if isprop(obj,'Engine') && ~isempty(obj.Engine) && ...
       isprop(obj.Engine,'Diameter') && isfinite(obj.Engine.Diameter)
        d_nac = obj.Engine.Diameter;
        L_nac = obj.Engine.Length;
    else
        % statistical estimate for large turbofan (BPR~8, B777-class)
        T_total  = obj.ThrustToWeightRatio * obj.MTOM * 9.81;
        T_engine = T_total / 2;
        d_nac    = 0.033 * sqrt(T_engine/1000);
        L_nac    = 1.5 * d_nac;
        warning('B777.CD0: Engine dims not found, using statistical d=%.2fm L=%.2fm', ...
                d_nac, L_nac);
    end

    f_nac    = L_nac / d_nac;
    FF_nac   = 1 + 0.35/f_nac;
    S_wet_nac = pi * d_nac * L_nac;
    Re_nac   = rho * V * L_nac / mu;
    Cf_nac   = getCf(Re_nac, M);
    Q_nac    = 1.30;
    n_eng    = 2;

    CD0_nac  = Cf_nac * FF_nac * Q_nac * (n_eng * S_wet_nac / S_ref);

    % =========================================================
    % 4. TAIL CD0  (HTP + VTP)
    % =========================================================
    tc_tail         = 0.10;
    x_tc_tail       = 0.30;
    Lambda_tail_rad = deg2rad(25.0);

    FF_tail = (1 + (0.6/x_tc_tail)*tc_tail + 100*tc_tail^4) * ...
              (1.34 * M^0.18 * cos(Lambda_tail_rad)^0.28);

    if isprop(obj,'HtpArea') && ~isempty(obj.HtpArea) && isfinite(obj.HtpArea)
        S_HTP = obj.HtpArea;
    else
        L_HT  = obj.HtpPos - obj.WingPos;
        S_HTP = obj.V_HT * S_ref * c_ref / L_HT;
        warning('B777.CD0: HtpArea not found, estimating from V_HT = %.5f m^2', S_HTP);
    end
    S_wet_HTP = 2.0 * S_HTP * (1 + 0.2*tc_tail);
    Re_HTP    = rho * V * sqrt(S_HTP) / mu;
    Cf_HTP    = getCf(Re_HTP, M);
    CD0_HTP   = Cf_HTP * FF_tail * 1.05 * (S_wet_HTP/S_ref);

    if isprop(obj,'VtpArea') && ~isempty(obj.VtpArea) && isfinite(obj.VtpArea)
        S_VTP = obj.VtpArea;
    else
        L_VT  = obj.VtpPos - obj.WingPos;
        S_VTP = obj.V_VT * S_ref * b / L_VT;
        warning('B777.CD0: VtpArea not found, estimating from V_VT = %.5f m^2', S_VTP);
    end
    S_wet_VTP = 2.0 * S_VTP * (1 + 0.2*tc_tail);
    Re_VTP    = rho * V * sqrt(S_VTP) / mu;
    Cf_VTP    = getCf(Re_VTP, M);
    CD0_VTP   = Cf_VTP * FF_tail * 1.05 * (S_wet_VTP/S_ref);

    CD0_tail  = CD0_HTP + CD0_VTP;

    % =========================================================
    % 5. MISCELLANEOUS DRAG
    % Raymer ch12: 3% of component sum for leakage, gaps, antennas
    % =========================================================
    CD0_subtotal = CD0_wing + CD0_fuse + CD0_nac + CD0_tail;
    CD0_misc     = 0.03 * CD0_subtotal;

    % =========================================================
    % 6. TOTAL
    % =========================================================
    CD0_total = CD0_subtotal + CD0_misc;

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
    % PRINT SUMMARY
    % =========================================================
    if isa(obj, 'B777.ADP_BW')
        config = 'BOXWING';
    else
        config = 'TUBEWING';
    end

    fprintf('\n==============================================\n');
    fprintf('  B777.CD0 FULL BUILDUP - %s\n', config);
    fprintf('==============================================\n');
    fprintf('  Cruise M              = %.3f\n',         M);
    fprintf('  c_ref (MAC)           = %.3f m\n',       c_ref);
    fprintf('  Re_wing               = %.4e\n',          Re_wing);
    fprintf('----------------------------------------------\n');
    fprintf('  WING\n');
    fprintf('  Cf (mixed+compress.)  = %.4e\n',          Cf_wing);
    fprintf('  FF                    = %.4f\n',           FF_wing);
    fprintf('  S_wet                 = %.2f m^2\n',       S_wet_wing);
    fprintf('  Q                     = %.2f\n',           Q_wing);
    fprintf('  CD0_wing              = %.5f\n',           CD0_wing);
    fprintf('----------------------------------------------\n');
    fprintf('  FUSELAGE\n');
    fprintf('  L/d (fineness)        = %.2f\n',           f_ratio);
    fprintf('  FF (Raymer 12.31)     = %.4f\n',           FF_fuse);
    fprintf('  S_wet                 = %.2f m^2\n',       S_wet_fuse);
    fprintf('  CD0_fuse              = %.5f\n',           CD0_fuse);
    fprintf('----------------------------------------------\n');
    fprintf('  NACELLES (x%d)\n',                         n_eng);
    fprintf('  d=%.2fm  L=%.2fm\n',                       d_nac, L_nac);
    fprintf('  FF (Raymer 12.32)     = %.4f\n',           FF_nac);
    fprintf('  Q (wing-mounted)      = %.2f\n',           Q_nac);
    fprintf('  CD0_nacelles          = %.5f\n',           CD0_nac);
    fprintf('----------------------------------------------\n');
    fprintf('  TAIL\n');
    fprintf('  CD0_HTP               = %.5f\n',           CD0_HTP);
    fprintf('  CD0_VTP               = %.5f\n',           CD0_VTP);
    fprintf('  CD0_tail              = %.5f\n',           CD0_tail);
    fprintf('----------------------------------------------\n');
    fprintf('  CD0 subtotal          = %.5f\n',           CD0_subtotal);
    fprintf('  CD0 misc (3%%)         = %.5f\n',           CD0_misc);
    fprintf('----------------------------------------------\n');
    fprintf('  CD0 TOTAL             = %.5f  <-- output\n', CD0_total);
    fprintf('----------------------------------------------\n');
    fprintf('  BREAKDOWN FRACTIONS\n');
    fprintf('  Wing      %.1f%%\n', 100*CD0_wing/CD0_total);
    fprintf('  Fuselage  %.1f%%\n', 100*CD0_fuse/CD0_total);
    fprintf('  Nacelles  %.1f%%\n', 100*CD0_nac/CD0_total);
    fprintf('  Tail      %.1f%%\n', 100*CD0_tail/CD0_total);
    fprintf('  Misc      %.1f%%\n', 100*CD0_misc/CD0_total);
    fprintf('==============================================\n\n');

end