classdef ADP < handle
    %ADP Aircraft Design Parameters - Boxwing Freighter
    %
    %  CHANGES vs previous version:
    %   1. FrontWingSpan = 64.9 m, RearWingSpan = 54.9 m  (rear = front - 10)
    %      EffectiveSpan = (64.9 + 54.9) / 2 = 59.9 m  (ICAO Cat E)
    %   2. AR_target = 10.0  (enforced in updateDerivedProps and Size.m)
    %      WingArea = 59.9^2 / 10 = 358.8 m^2
    %   3. updateDerivedProps derives WingArea from span + AR_target,
    %      NOT from adding connector area — that was causing WingArea to
    %      balloon to 1419 m^2 and AR to collapse to 2.13 each iteration.
    %   4. Added CDwave and CDtrim properties for AeroPolar
    %   5. CL_max and Delta_Cl_ld corrected to physical values

    properties
        TLAR
        Engine
        AeroPolar
    end

    % Mass properties
    properties
        MTOM    = 319000;   % [kg]  initial seed
        OEM                 % [kg]  calculated by Size.m
        Mf_Ldg  = 0.75;    % landing mass fraction
        Mf_Fuel = 0.28;    % fuel mass fraction  (initial guess)
        Mf_TOC  = 0.98;    % top-of-climb mass fraction
        Mf_res  = 0.04;    % reserve fuel fraction
    end

    % Constraint analysis outputs
    properties
        ThrustToWeightRatio = 0.30;   % initial guess
        WingLoading         = 6700;   % [N/m^2] initial guess (B777F-level)
    end

    % Aerodynamic properties
    properties
        Cl_max      = 1.77;    % 2D section CLmax (SC(2)-0714 from XFOIL)
        CL_max      = 1.59;    % 3D clean wing CLmax (boxwing, lower sweep)
        Delta_Cl_ld = 1.3;     % CL increment landing flaps (double-slotted)
        Delta_Cl_to = 1.0;     % CL increment take-off flaps
        CD_TO       = 0.025;
        CL_TO       = 0.90;
        CD_LDG      = 0.030;
        CL_LDG      = 0.90;
        CL_cruise   = 0.55;    % updated each iter by UpdateAero
        LD_c        = 21;      % placeholder — overwritten by polar
        LD_app      = 12;
        CD0         = 0.018;   % seed (overwritten by BoxWing.B777.CD0 build-up)
        e           = 0.95;    % Oswald (Kroo boxwing correction applied in AeroPolar)
        CDwave      = 0.0005;  % wave drag at cruise Mach ~0.82
        CDtrim      = 0.0002;  % trim drag seed (updated by AeroPolar/trimDrag)
        % Boxwing has no conventional tail
        V_HT        = 0;
        V_VT        = 0;
    end

    % AR target — used by updateDerivedProps and Size.m to pin AR
    properties
        AR_target   = 10.0;    % FIXED aspect ratio for sizing
        Span_max    = 64.9;    % [m] ICAO Cat E wingspan limit
    end

    % Aerofoil / wing section properties — read by CD0.m
    properties
        tc      = 0.14;    % thickness/chord ratio (SC(2)-0714 ~14%)
        Sweep25 = 25.0;    % [deg] quarter-chord sweep of FRONT wing
    end

    % Sizing flags
    properties
        isSizeEng  = true;
        isSizeWing = true;
    end

    % Boxwing wing geometry
    properties
        FrontWingSpan = 64.9;   % [m]  ICAO Cat E limit
        FrontWingArea = 0;      % [m^2] set by updateDerivedProps
        FrontWingPos  = 0;      % [m]   set by updateDerivedProps

        RearWingSpan  = 54.9;   % [m]  rear = front - 10 m
        RearWingArea  = 0;      % [m^2] set by updateDerivedProps
        RearWingPos   = 0;      % [m]   set by updateDerivedProps

        ConnectorHeight = 8;    % [m] vertical gap between wings

        TotalLiftingArea = 358.8; % [m^2] updated by updateDerivedProps
        EffectiveSpan    = 59.9;  % [m]   updated by updateDerivedProps

        Mstar = 0.95;
    end

    % Names used by shared cast / sizing code
    properties
        Thrust   = 0;
        WingArea = 358.8;   % [m^2] = TotalLiftingArea, updated each iter
        Span     = 59.9;    % [m]   = EffectiveSpan
        WingPos  = 0;
        KinkPos  = 0;

        HtpArea = 0;
        VtpArea = 0;
        HtpPos  = 0;
        VtpPos  = 0;

        c_ac = 0;
        x_ac = 0;
        c_ach = 0;
        c_acv = 0;
    end

    % Fuselage geometry
    properties
        CockpitLength = 6.5;
        CabinRadius   = 2.93;
        CabinLength   = 0;      % set in constructor
    end

    properties
        x_ac_rear = 0;
        x_ac_sys  = 0;
        MAC       = 0;
    end

    properties
        etaLift   = 0.6;   % front/rear lift split
        alphaArea = 0.5;   % front/rear area split
        k_relief  = 0.75;  % load relief factor
    end

    methods
        function obj = ADP()
            obj.CabinLength = 70.0 - obj.CockpitLength ...
                              - obj.CabinRadius * 2 * 1.48;
            obj.updateDerivedProps();
        end

        function updateDerivedProps(obj)
            %UPDATEDERIVEDPROPS  Recalculate geometry from span + AR_target.
            %
            %  KEY FIX: WingArea is derived from AR_target and EffectiveSpan,
            %  NOT from summing component areas + connector area.
            %  This keeps AR = AR_target regardless of MTOM, preventing the
            %  WingArea-balloon / AR-collapse divergence.
            %
            %  Span relationship: RearWingSpan = FrontWingSpan - 10 m
            %  EffectiveSpan = (FrontWingSpan + RearWingSpan) / 2
            %                = FrontWingSpan - 5 m

            % Effective span = average of front and rear spans
            obj.EffectiveSpan = (obj.FrontWingSpan + obj.RearWingSpan) / 2;
            obj.Span          = obj.EffectiveSpan;

            % Wing area from AR target (this is the critical fix)
            obj.TotalLiftingArea = obj.EffectiveSpan^2 / obj.AR_target;
            obj.WingArea         = obj.TotalLiftingArea;

            % Split between front and rear wings
            obj.FrontWingArea = obj.WingArea * 0.55;   % 55% front
            obj.RearWingArea  = obj.WingArea * 0.45;   % 45% rear

            % Fuselage station positions
            L_f = obj.CockpitLength + obj.CabinLength + obj.CabinRadius * 1.48;
            obj.FrontWingPos = 0.40 * L_f;
            obj.RearWingPos  = 0.90 * L_f;
            obj.WingPos      = obj.FrontWingPos;
        end

        function out = AR(obj)
            if obj.TotalLiftingArea > 0
                out = obj.EffectiveSpan^2 / obj.TotalLiftingArea;
            else
                out = obj.AR_target;
            end
        end
    end
end