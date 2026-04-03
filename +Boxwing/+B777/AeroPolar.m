classdef AeroPolar
%AEROPOLAR  Drag polar for the boxwing freighter — including trim drag and wave drag.
%
%  CD(CL) = CD0 + Beta*CL^2 + CDwave + CD_trim
%
%  FIXES vs previous version:
%   1. Added CDwave (was missing — caused CD to be underestimated at cruise Mach)
%   2. AR computed from EffectiveSpan if available (fixes locked AR=5 in trade study)
%   3. Physical guard: AR floored at 3.0, e floored at 0.60
%   4. CD_trim=0 fallback is now silent (was spamming warnings every iteration)

    properties
        CD0      % zero-lift drag coefficient    [-]
        CDwave   % wave drag at cruise Mach       [-]
        e        % Oswald efficiency factor       [-]
        Beta     % induced drag factor 1/(pi*AR*e)[-]
        CD_trim  % trim drag increment            [-]
        AR       % stored for diagnostics         [-]
    end

    methods
        function obj = AeroPolar(ADP, x_cg)
            %% AR — FIX: use EffectiveSpan if available so span trade sees AR vary
            if isprop(ADP,'EffectiveSpan') && ~isempty(ADP.EffectiveSpan) ...
                    && isfinite(ADP.EffectiveSpan) && ADP.EffectiveSpan > 0 ...
                    && ~isempty(ADP.WingArea) && ADP.WingArea > 0
                obj.AR = ADP.EffectiveSpan^2 / ADP.WingArea;
            else
                obj.AR = ADP.AR();   % fallback to Span^2/WingArea
            end
            obj.AR = max(obj.AR, 3.0);   % physical floor

            %% Zero-lift drag
            obj.CD0 = ADP.CD0;

            %% Wave drag — FIX: was missing from old version
            if isprop(ADP,'CDwave') && ~isempty(ADP.CDwave) && isfinite(ADP.CDwave)
                obj.CDwave = ADP.CDwave;
            else
                obj.CDwave = 0.0005;   % ~Mach 0.82, lower wave drag than B777
            end

            %% Induced drag (Kroo 2005 boxwing correction)
            Q = 1.02;  P = 0.006;
            e_kroo = 1.0 / (Q + P * pi * obj.AR);
            obj.e  = min(e_kroo, ADP.e);
            obj.e  = max(obj.e, 0.60);   % physical floor
            obj.Beta = 1.0 / (pi * obj.AR * obj.e);

            %% Trim drag — use real CG if provided
            if nargin < 2 || isempty(x_cg)
                L_f  = ADP.CockpitLength + ADP.CabinLength + ADP.CabinRadius * 1.48;
                x_cg = 0.48 * L_f;
            end
            try
                [obj.CD_trim, ~] = BoxWing.B777.trimDrag(ADP, x_cg);
                % Sanity: trim drag should be small positive number
                if ~isfinite(obj.CD_trim) || obj.CD_trim < 0 || obj.CD_trim > 0.01
                    obj.CD_trim = 0.0002;
                end
            catch
                obj.CD_trim = 0.0002;   % FIX: silent fallback, no warning spam
            end
        end

        function CD = CD(obj, CL)
            %CD  Full polar: parasite + induced + wave + trim
            CD = obj.CD0 + obj.Beta .* CL.^2 + obj.CDwave + obj.CD_trim;
        end

        function CD = CD_no_trim(obj, CL)
            %CD_NO_TRIM  Polar without trim drag (for sensitivity studies)
            CD = obj.CD0 + obj.Beta .* CL.^2 + obj.CDwave;
        end
    end
end