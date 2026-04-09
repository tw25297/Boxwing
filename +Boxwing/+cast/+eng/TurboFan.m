classdef TurboFan
    %TURBOFAN Simple turbofan engine model

    properties
        Length
        Diameter
        Mass
        SFC_TO
        SFC_cruise
        T_Static
        BPR
        SFC_A = 0.45;
        SFC_B = 0.54;
    end

    methods
        function obj = TurboFan(T_Static,L,D,M,SFC_TO,SFC_cruise,BPR,alt_cruise,M_cruise)
            arguments
                T_Static
                L
                D
                M
                SFC_TO
                SFC_cruise
                BPR
                alt_cruise = 35e3 ./ SI.ft
                M_cruise   = 0.85
            end
            obj.T_Static   = T_Static;
            obj.Length     = L;
            obj.Diameter   = D;
            obj.Mass       = M;
            obj.SFC_TO     = SFC_TO;
            obj.SFC_cruise = SFC_cruise;
            obj.BPR        = BPR;

            obj.SFC_A = obj.SFC_TO;
            T  = Boxwing.cast.atmosT(alt_cruise);
            T0 = Boxwing.cast.atmosT(0);
            obj.SFC_B = (obj.SFC_cruise ./ sqrt(T/T0) - obj.SFC_A) / M_cruise;
        end

        function eng_new = Rubberise(obj, T_new)
            f = T_new / obj.T_Static;
            eng_new = Boxwing.cast.eng.TurboFan(T_new, ...
                obj.Length   * f^0.4, ...
                obj.Diameter * f^0.5, ...
                obj.Mass     * f^1.1, ...
                obj.SFC_TO, obj.SFC_cruise, obj.BPR);
            eng_new.SFC_A = obj.SFC_A;
            eng_new.SFC_B = obj.SFC_B;
        end

        function TSFC = TSFC(obj, M, alt)
            T  = Boxwing.cast.atmosT(alt);
            T0 = Boxwing.cast.atmosT(0);
            TSFC = (obj.SFC_A + obj.SFC_B .* M) .* sqrt(T ./ T0);
        end
    end

    methods(Static)

        function obj = CFM_LEAP_1A(sfc_scaling, alt_cruise, M_cruise)
            arguments
                sfc_scaling = 1;
                alt_cruise  = 35e3 ./ SI.ft
                M_cruise    = 0.85
            end
            BPR        = 9.6;
            SFC_T0     = 19*exp(-0.12*BPR)*1e-6 * sfc_scaling;
            SFC_cruise = 25*exp(-0.05*BPR)*1e-6 * sfc_scaling;
            obj = Boxwing.cast.eng.TurboFan(143050,3.328,2.4,3008, ...
                                    SFC_T0,SFC_cruise,BPR,alt_cruise,M_cruise);
        end

        function obj = GE90(sfc_scaling, alt_cruise, M_cruise)
            arguments
                sfc_scaling = 1;
                alt_cruise  = 32e3 ./ SI.ft
                M_cruise    = 0.84
            end
            BPR        = 8.5;
            SFC_T0     = 19*exp(-0.12*BPR)*1e-6 * sfc_scaling;
            SFC_cruise = 25*exp(-0.05*BPR)*1e-6 * sfc_scaling;
            obj = Boxwing.cast.eng.TurboFan(513e3,7.28,3.85,8762, ...
                                    SFC_T0,SFC_cruise,BPR,alt_cruise,M_cruise);
        end

        function obj = CFM56_5()
            f   = 1 ./ (SI.lb/(SI.lbf*SI.hr));
            obj = BoxWing.cast.eng.TurboFan(107e3,2.422,2.00,2331, ...
                                    0.3316*f, 0.596*f, 6);
        end

        function obj = UltraFan(sfc_scaling, alt_cruise, M_cruise)
            %ULTRAFAN  Rolls-Royce UltraFan for boxwing freighter.
            %   Ultra-high bypass ratio geared turbofan (~2030 EIS).
            arguments
                sfc_scaling = 1;
                alt_cruise  = 39e3 ./ SI.ft
                M_cruise    = 0.82
            end
            BPR        = 15;
            SFC_T0     = 18*exp(-0.12*BPR)*1e-6 * sfc_scaling;
            SFC_cruise = 22*exp(-0.05*BPR)*1e-6 * sfc_scaling;
            % Physical specs scaled for 319 t aircraft
            obj = BoxWing.cast.eng.TurboFan(380e3, 6.5, 4.5, 6500, ...
                                    SFC_T0, SFC_cruise, BPR, ...
                                    alt_cruise, M_cruise);
        end

    end
end
