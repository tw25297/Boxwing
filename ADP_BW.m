classdef ADP_BW < handle
    %ADP Aircraft Design Parameters for a B777F


    % Top level Design Parameters

    % Masses
    properties
        TLAR
        Engine
        AeroPolar
    end

    properties
        MTOM    % Maximum take-off mass
        OEM     % Operational Empty Mass
        Mf_Ldg  % maximum landing mass fraction (e.g. MLDG = MTOM*Mf_Ldg)
        Mf_Fuel % fuel mass fraction 
        Mf_TOC  % "top of climb" mass fraction
        Mf_res  % "Resevre Fuel" mass fraction
    end

    % constraint Paramters
    properties
        ThrustToWeightRatio  % 
        WingLoading          % 
    end

    % --------------------- Aerodynamic (BoxWing, compatible with tube-wing pipeline) ----------------------
    properties
        % --- Airfoil definition ---
        AirfoilName = "NASA SC(2)-0714";
        tc_ref = 0.14;                 % thickness ratio (midspan reference)

        % --- Section lift characteristics (interpolated from XFOIL @ Re = 1e6, SC(2)-0714) ---
        Re_section_ref   = 1e6;
        Cl_alpha_perdeg  = 0.1233;     % [1/deg]
        Cl_alpha         = 7.06;       % [1/rad]
        alphaL0_deg      = -4.69;      % [deg]
        Cl_max           = 1.77;       % 2D clean section Clmax

        % --- Wing-level lift assumptions (conceptual placeholders) ---
        CL_max     = 1.59;             % 3D CLEAN wing CLmax (no high-lift)
        Delta_Cl_ld = 1.0;             % landing increment
        Delta_Cl_to = 0.8;             % takeoff increment

        % --- Ground run assumptions (TO/Landing config) ---
        CD_TO  = 0.05;
        CL_TO  = 0.8;
        CD_LDG = 0.08;
        CL_LDG = 0.5;

        % --- Cruise condition ---
        CL_cruise;                     % computed in sizing script (leave unset here)

        % --- Aircraft drag polar parameters (cruise) ---
        CD0    = 0.021;                % baseline (replace later with plate build-up)
        e      = 0.97;                 % refers to Wolkovitch (1986)
        CDwave = 0.001;                % wave drag increment near transonic cruise

        % --- Tail volume coefficients (BW defaults) ---
        V_HT = 0.55;
        V_VT = 0.05;

        % --- Ceiling sizing knob ---
        CL_ceiling = 1.0;
        Sweep25 = 30.0; % default sweep at 25% chord, used for Michel's criterion in CD0 estimation, can be overridden by geometry if available

        % Flap drag increments -- Raymer Table 12.7, triple slotted flap
        Delta_CD0_TO = 0.015;   % ***REFINEMENT*** CD0 increment flaps TO
        Delta_CD0_LD = 0.055;   % ***REFINEMENT*** CD0 increment flaps LD
    end

    % Sizing Flags (whether to Adjust certain values during sizing process)
    properties
        isSizeEng = true; % whether to change engine maximum Thrust Value
        isSizeWing = true; % whether to size the wing
    end

    % loop limit
    properties
        AR_target = 10;                % AR target (match tube wing) (8-12)
        Span_max = 65;                 % [m]
        WS_min = 4.0e3;
        WS_max = 1.30e4;
    end

    % Concrete properties
    properties
        Thrust;

        % planform specific
        Span;
        WingArea;
        KinkPos;    % y position of wing kink 
        WingPos;    % Wing position along fuselage
        HtpPos;     % HTP pos along fuselage
        VtpPos;     % VTP pos along fuselage

        Mstar = 0.935; % wing technology factor

        % Empennage Specific
        HtpArea;
        VtpArea
    end

    % useful properties
    properties
        c_ac  % mean geometric chord of main wing
        x_ac  % x location of mean geometric chord
        c_ach % mean geometric chord of HTP
        c_acv % mean geometric chord of VTP
    end

    % fuselage geometry
    properties
        CockpitLength = 7.3;
        CabinRadius   = 2.8;
        CabinLength   = 70.8 - 7.3 - 2.8*2*1.48;  % Lf_A350 - CockpitLength - 2*(1.48*diameter)
    end

    % =========================================================================
    %  Fuselage material model  ---  CFRP adoption by sub-component and segment
    % =========================================================================
    %
    %  Structural split rationale
    %  --------------------------
    %  Fuselage primary structure is divided into two load-path groups whose
    %  combined mass fractions must sum to 1.0:
    %
    %    Skin + stringers  (fus_ratio_skin   = 0.45)
    %      Resist hoop stress (pressurisation) and longitudinal bending.
    %      CFRP is well-suited to this role; A350 / B787 barrel skins are
    %      >50% CFRP by mass.  Kmat_skin = 0.78 represents a 22% mass saving
    %      versus Al 2024-T3 at equivalent stiffness.
    %
    %    Frames + bulkheads (fus_ratio_frames = 0.55)
    %      Maintain cross-section shape and transfer concentrated loads.
    %      Heavier per unit length; CFRP adoption is lower because thick
    %      frames and complex joints favour metal or hybrid solutions.
    %      Kmat_frame = 0.85 represents a 15% mass saving versus Al 2024-T3.
    %
    %  Per-segment CFRP fractions (fCFRP_*)
    %  ----------------------------------------
    %    Nose      skin 30% / frame 20%  -- simple ogive, few frames, limited
    %                                       production run favours aluminium
    %    Cockpit   skin 40% / frame 25%  -- A350 / B787 cockpit skin ~40% CFRP;
    %                                       window and door cutouts complicate frames
    %    Fwd/Aft   skin 60% / frame 35%  -- long uniform pressure vessel; B777X
    %    Barrel                             barrel quoted at 55-65% CFRP skins
    %    Wing-box  skin 60% / frame 35%  -- skin panels switch to CFRP; carry-
    %                                       through frames remain Al / Ti for
    %                                       damage tolerance and fatigue life
    %    Tailcone  skin 70% / frame 30%  -- thin tapered shell; ideal CFRP
    %                                       geometry; very few circumferential frames
    %
    %  Segment intensity multipliers (kseg_*)
    %  ----------------------------------------
    %  Mass per metre of each segment relative to the fuselage average.
    %  The array is renormalised in fuselage_bw.m so the total always matches
    %  the Raymer aluminium baseline exactly.
    %
    %    Nose      0.8  -- low-curvature ogive, unpressurised
    %    Cockpit   1.3  -- dense frames, avionics bay, window / door load paths
    %    FwdBarrel 1.0  -- nominal reference segment
    %    WingBox   1.4  -- carry-through frame + keel beam mass penalty
    %    AftBarrel 0.9  -- fewer cutouts, lower net bending moment
    %    Tailcone  0.7  -- tapered geometry, lightly loaded

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

    % Box-wing load distribution parameters
    properties
        etaLift   = 0.6;   % fraction of total lift carried by front wing
        alphaArea = 0.5;   % fraction of total wing area assigned to front wing
        k_relief  = 0.75;  % fuselage bending relief factor (box-wing literature)
    end

    % Wing geometry
    properties
        FrontWingPos = 0.4*63.7;
        RearWingPos  = 0.7*63.7;
        S_wet_front;  % wetted area of front wing, stored from boxwingmass.m
        S_wet_rear;   % wetted area of rear wing,  stored from boxwingmass.m
        S_wet_fins;   % wetted area of tip fins,   stored from boxwingmass.m
        MAC;          % mean aerodynamic chord,     stored from boxwingmass.m
    end

    methods
        function out = AR(obj)
            out = obj.Span.^2./obj.WingArea;
        end
    end
end