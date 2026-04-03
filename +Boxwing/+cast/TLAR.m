classdef TLAR
    %TLAR Top-Level Aircraft (Design) Requirements
    % we have TLAR for 3 aircraft if u have to make any changes make in the
    % function obj= Boxwing

    properties
        Crew
        Range
        Payload
        V_ld
        V_app
        V_climb
        GroundRun
        GroundRunLanding
        M_c
        Alt_max
        Alt_cruise
        CrewMass
    end

    properties
        H_to_screen
        TOCG_AEO_gearUp
        TOCG_OEI_gearDown
        V_tocg_ref
        ISA_deltaT_TO
    end

    properties
        TTC_alt1
        TTC_alt2
        TTC_time
        ROC_min_at_cruise
        Alt_speed_restriction
        Vmax_below_10k
        ISA_deltaT_climb
    end

    properties
        M_alt
    end

    % FIX: these three cannot use SI.xx as default values because MATLAB
    % evaluates property defaults at class-load time (before any method
    % runs), which causes infinite recursion.  Set them inside each method.
    properties
        Alt_alternate
        Range_alternate
        Loiter
    end

    methods(Static)

        function obj = Boxwing
            obj = BoxWing.cast.TLAR();
            obj.Alt_alternate   = 22e3  ./ SI.ft;
            obj.Range_alternate = 200   ./ SI.Nmile;
            obj.Loiter          = 30    ./ SI.min;

            obj.Range            = 4700  ./ SI.Nmile;
            obj.Payload          = 123000; % change for carry more payload
            obj.M_c              = 0.82;
            obj.Alt_cruise       = 39e3  ./ SI.ft;
            obj.Alt_max          = 41e3  ./ SI.ft;

            obj.GroundRun        = 2600;
            obj.GroundRunLanding = 1700;
            obj.V_app            = 135   ./ SI.knt;
            obj.V_ld             = 140   ./ SI.knt;
            obj.V_climb          = 250   ./ SI.knt;

            obj.Crew             = 3;
            obj.CrewMass         = (80+10) * obj.Crew;

            obj.H_to_screen           = 35    ./ SI.ft;
            obj.TOCG_AEO_gearUp       = 0.027;
            obj.TOCG_OEI_gearDown     = 0.030;
            obj.V_tocg_ref            = obj.V_climb;
            obj.ISA_deltaT_TO         = 15;

            obj.TTC_alt1              = 1500  ./ SI.ft;
            obj.TTC_alt2              = max(obj.Alt_cruise, 20e3./SI.ft);
            obj.TTC_time              = 26    ./ SI.min;
            obj.ROC_min_at_cruise     = 300;
            obj.Alt_speed_restriction = 10e3  ./ SI.ft;
            obj.Vmax_below_10k        = 250   ./ SI.knt;
            obj.ISA_deltaT_climb      = 0;
        end

        %function obj = B777F
        %    obj = BoxWing.cast.TLAR();
            % set alternate/loiter defaults safely inside a method
        %    obj.Alt_alternate   = 22e3  ./ SI.ft;
        %    obj.Range_alternate = 200   ./ SI.Nmile;
        %    obj.Loiter          = 30    ./ SI.min;

        %    obj.Range            = 4800  ./ SI.Nmile;
        %    obj.GroundRun        = 2830;
        %    obj.GroundRunLanding = 1500;
         %   obj.M_c              = 0.82;
         %   obj.Alt_max          = 39e3  ./ SI.ft;
        %    obj.Alt_cruise       = 31e3  ./ SI.ft;
        %    obj.Crew             = 4;
        %    obj.Payload          = 140000; % changed 
        %    obj.CrewMass         = (80+10) * obj.Crew;
        %    obj.V_app            = 145   ./ SI.knt;
        %    obj.V_ld             = 150   ./ SI.knt;
        %    obj.V_climb          = 250   ./ SI.knt;

         %   obj.H_to_screen           = 35    ./ SI.ft;
        %    obj.TOCG_AEO_gearUp       = 0.03;
        %    obj.TOCG_OEI_gearDown     = 0.005;
        %    obj.V_tocg_ref            = obj.V_climb;
        %    obj.ISA_deltaT_TO         = 15;

        %    obj.TTC_alt1              = 1500  ./ SI.ft;
        %    obj.TTC_alt2              = max(obj.Alt_cruise, 20e3./SI.ft);
        %    obj.TTC_time              = 30    ./ SI.min;
        %    obj.ROC_min_at_cruise     = 300;
        %    obj.Alt_speed_restriction = 10e3  ./ SI.ft;
        %    obj.Vmax_below_10k        = 250   ./ SI.knt;
         %   obj.ISA_deltaT_climb      = 0;
        %end

        %function obj = A350F
        %    obj = cast.TLAR();
        %    obj.Alt_alternate   = 22e3  ./ SI.ft;
        %    obj.Range_alternate = 200   ./ SI.Nmile;
        %    obj.Loiter          = 30    ./ SI.min;

        %    obj.Range            = 5000  ./ SI.Nmile;%changed today
        %    obj.Payload          = 109000;
         %   obj.M_c              = 0.85;
          %  obj.Alt_cruise       = 41e3  ./ SI.ft;
         %   obj.Alt_max          = 43e3  ./ SI.ft;

         %   obj.GroundRun        = 2750;
        %    obj.GroundRunLanding = 1800;
       %     obj.V_app            = 140   ./ SI.knt;
       %     obj.V_ld             = 145   ./ SI.knt;
        %    obj.V_climb          = 250   ./ SI.knt;

        %    obj.Crew             = 3;
        %    obj.CrewMass         = (80+10) * obj.Crew;

        %    obj.H_to_screen           = 35    ./ SI.ft;
        %    obj.TOCG_AEO_gearUp       = 0.024;
        %    obj.TOCG_OEI_gearDown     = 0.027;
        %    obj.V_tocg_ref            = obj.V_climb;
         %   obj.ISA_deltaT_TO         = 15;

        %    obj.TTC_alt1              = 1500  ./ SI.ft;
         %   obj.TTC_alt2              = max(obj.Alt_cruise, 20e3./SI.ft);
         %   obj.TTC_time              = 25    ./ SI.min;
         %   obj.ROC_min_at_cruise     = 300;
         %   obj.Alt_speed_restriction = 10e3  ./ SI.ft;
        %    obj.Vmax_below_10k        = 250   ./ SI.knt;
        %    obj.ISA_deltaT_climb      = 0;
       % end

    end
end
