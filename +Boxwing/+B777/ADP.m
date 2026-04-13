classdef ADP < handle
    %ADP  Aircraft Design Parameters -- Boxwing Freighter
    %
    %  KEY FIX (updateDerivedProps):
    %    Previous version kept AR_target fixed and derived WingArea = b²/AR.
    %    This meant every span in the trade study had identical AR (10.0),
    %    so a larger span simply gave a proportionally larger wing — heavier
    %    structure with no induced-drag benefit, making MTOM always increase
    %    monotonically and masking the aerodynamic optimum.
    %
    %    Correct physics: fix the mean aerodynamic chord (c_ref) and let AR
    %    vary naturally with span.  Wing area = b_eff * c_ref.
    %    c_ref is derived once from the baseline span and AR_target and then
    %    held constant for all trade-study iterations.
    %
    %  Other properties unchanged from previous version.

    properties
        TLAR
        Engine
        AeroPolar
    end

    % Mass properties
    properties
        MTOM    = 319000;
        OEM
        Mf_Ldg  = 0.75;
        Mf_Fuel = 0.28;
        Mf_TOC  = 0.98;
        Mf_res  = 0.04;
    end

    % Constraint analysis outputs
    properties
        ThrustToWeightRatio = 0.30;
        WingLoading         = 6700;
    end

    % Aerodynamic properties
    properties
        Cl_max      = 1.77;
        CL_max      = 1.59;
        Delta_Cl_ld = 1.3;
        Delta_Cl_to = 1.0;
        CD_TO       = 0.025;
        CL_TO       = 0.90;
        CD_LDG      = 0.030;
        CL_LDG      = 0.90;
        CL_cruise   = 0.55;
        LD_c        = 21;
        LD_app      = 12;
        CD0         = 0.018;
        e           = 0.95;
        CDwave      = 0.0005;
        CDtrim      = 0.0002;
        V_HT        = 0;
        V_VT        = 0.05;
    end

    % AR target and chord reference -- used by updateDerivedProps
    properties
        AR_target   = 10.0;    % baseline AR (used only to seed c_ref)
        c_ref_fixed = 0;       % [m] mean chord -- set once, then fixed
                               %     0 means "not yet initialised"
        Span_max    = 64.9;    % [m] ICAO Cat E
    end

    % Aerofoil / section
    properties
        tc      = 0.14;
        Sweep25 = 25.0;
    end

    % Sizing flags
    properties
        isSizeEng  = true;
        isSizeWing = true;
    end

    % Boxwing wing geometry
    properties
        FrontWingSpan   = 60.0;
        FrontWingArea   = 0;
        FrontWingPos    = 0;

        RearWingSpan    = 50.0;
        RearWingArea    = 0;
        RearWingPos     = 0;

        ConnectorHeight = 3;

        TotalLiftingArea = 302.5;
        EffectiveSpan    = 55.0;

        Mstar = 0.95;
    end

    % Aliases used by shared cast/sizing code
    properties
        Thrust   = 0;
        T_Static = 500; % kN
        WingArea = 302.5;
        Span     = 55.0;
        WingPos  = 0;
        KinkPos  = 0;

        HtpArea = 0;
        VtpArea = 0;
        HtpPos  = 0;
        VtpPos  = 0;

        c_ac  = 0;
        x_ac  = 0;
        c_ach = 0;
        c_acv = 0;
    end

    % Fuselage geometry
    properties
        CockpitLength = 7.3;
        CabinRadius   = 2.8;
        CabinLength   = 70.8 - 7.3 - 2.8*2*1.48;
    end

    properties
        % Primary structure mass split (must sum to 1.0)
        fus_ratio_skin   = 0.45;   % skin + stringers fraction
        fus_ratio_frames = 0.55;   % frames + bulkheads fraction

        % CFRP mass fractions -- skin group, per segment
        fCFRP_skin_nose     = 0.30;
        fCFRP_skin_cockpit  = 0.40;
        fCFRP_skin_barrel   = 0.60;   % applied to both forward and aft barrel
        fCFRP_skin_wingbox  = 0.60;
        fCFRP_skin_aft      = 0.60;
        fCFRP_skin_tailcone = 0.70;

        % CFRP mass fractions -- frame group, per segment
        fCFRP_frame_nose     = 0.20;
        fCFRP_frame_cockpit  = 0.25;
        fCFRP_frame_barrel   = 0.35;  % applied to both forward and aft barrel
        fCFRP_frame_wingbox  = 0.35;
        fCFRP_frame_aft      = 0.35;
        fCFRP_frame_tailcone = 0.30;

        % CFRP weight-saving factors relative to aluminium-equivalent baseline
        Kmat_skin  = 0.78;   % 22% saving for CFRP skins vs Al 2024-T3
        Kmat_frame = 0.85;   % 15% saving for CFRP frames vs Al 2024-T3

        % Segment mass-intensity multipliers (mass per metre / fuselage average)
        kseg_nose      = 0.8;
        kseg_cockpit   = 1.3;
        kseg_fwdBarrel = 1.0;
        kseg_wingbox   = 1.4;
        kseg_aftBarrel = 0.9;
        kseg_tailcone  = 0.7;

        % Nose cargo-door structural penalty (fraction of corrected fuselage mass)
        % 2% covers hinge reinforcements, door skin, actuators and latching
        % structure for an upward-opening nose door (ref: 747-8F installation).
        k_door = 0.02;
    end

    properties
        x_ac_rear = 0;
        x_ac_sys  = 0;
        MAC       = 0;
    end

    % Lift/area split
    properties
        etaLift   = 0.6;
        alphaArea = 0.5;
        k_relief  = 0.75;
    end

    methods
        function obj = ADP()
            obj.CabinLength = 70.0 - obj.CockpitLength ...
                              - obj.CabinRadius * 2 * 1.48;
            obj.updateDerivedProps();
        end

        function updateDerivedProps(obj)
            %UPDATEDERIVEDPROPS  Recalculate geometry from current spans.
            %
            %  The mean chord (c_ref_fixed) is derived ONCE from the
            %  baseline span and AR_target, then kept constant so that AR
            %  varies naturally as span changes:
            %
            %    b_eff  = (FrontWingSpan + RearWingSpan) / 2
            %    WingArea = b_eff * c_ref_fixed        ← linear in span
            %    AR       = b_eff^2 / WingArea          ← varies with span
            %
            %  This produces the physically correct trade-study behaviour:
            %  increasing span raises AR (lower induced drag) but also raises
            %  structural weight, giving a genuine aerodynamic optimum.

            % 1. Effective span
            obj.EffectiveSpan = (obj.FrontWingSpan + obj.RearWingSpan) / 2;
            obj.Span          = obj.EffectiveSpan;

            % 2. Initialise c_ref_fixed on first call (when it is still 0)
            if obj.c_ref_fixed <= 0
                % Seed: baseline span / AR_target gives baseline chord
                obj.c_ref_fixed = obj.EffectiveSpan / obj.AR_target;
            end

            % 3. Wing area scales linearly with span (fixed chord)
            %    → AR = b_eff / c_ref_fixed  (varies with span)
            obj.TotalLiftingArea = obj.EffectiveSpan * obj.c_ref_fixed;
            obj.WingArea         = obj.TotalLiftingArea;

            % 4. Front/rear area split (55/45)
            obj.FrontWingArea = obj.WingArea * 0.55;
            obj.RearWingArea  = obj.WingArea * 0.45;

            % 5. Fuselage station positions
            L_f = obj.CockpitLength + obj.CabinLength + obj.CabinRadius * 1.48;
            obj.FrontWingPos = 0.40 * L_f;
            obj.RearWingPos  = 0.75 * L_f;
            obj.WingPos      = obj.FrontWingPos;

            % 6. Update MAC alias
            obj.MAC = obj.c_ref_fixed;
        end

        function out = AR(obj)
            %AR  Effective aspect ratio of the boxwing lifting system.
            if obj.TotalLiftingArea > 0
                out = obj.EffectiveSpan^2 / obj.TotalLiftingArea;
            else
                out = obj.AR_target;
            end
        end
    end
end