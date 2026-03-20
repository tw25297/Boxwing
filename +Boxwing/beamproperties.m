classdef beamproperties
        properties
        % identification
        name

        % geometry
        beamlength          % half-span [m]

        % aircraft / loading
        MTOM                % [kg]
        load_factor         % scalar, e.g. 1 or 2.5
        lift_split          % fraction of half-aircraft lift carried by this beam

        % material / stiffness
        E                   % Young's modulus [Pa]
        I                   % second moment of area [m^4]

        % triangular load description: w(x) = gradient*x + intercept
        gradient                        % [N/m^2]
        intercept                       % root load intensity w0 [N/m]
        effective_triangular_force      % [N]
        x_effective_force               % [m] from root

        % tip interaction force
        Rt                  % magnitude [N]
        Rt_sign             % +1 = upward on this beam, -1 = downward on this beam
        x_Rt                %

        % root reactions
        S_r                 % root shear [N]
        M_r                 % root moment [N m]
    end


    
    methods
        function obj = beamproperties(ADP, beam_name, lift_split, E, I)
            obj.name = beam_name;
            obj.beamlength = ADP.Span_max / 2;
            obj.MTOM = ADP.MTOM;

            obj.lift_split = lift_split;
            obj.E = E;
            obj.I = I;

            obj.load_factor = 1.0;
            obj.gradient = 0.0;
            obj.intercept = 0.0;
            obj.effective_triangular_force = 0.0;
            obj.x_effective_force = obj.beamlength / 3;

            obj.Rt = 0.0;
            obj.Rt_sign = 0.0;

            obj.S_r = 0.0;
            obj.M_r = 0.0;
        end 

        function obj = calcTriangularLoad(obj, n)
            g = 9.81;
            total_weight = obj.MTOM * g;
            safe_total_weight = total_weight * n;

            % total lift on one aircraft side
            half_aircraft_lift = safe_total_weight / 2;
        
            % beam takes only its assigned fraction of that side's lift
            beam_lift = obj.lift_split * half_aircraft_lift;
        
            % area under triangular load = beam_lift
            obj.intercept = (2 * beam_lift) / obj.beamlength;


            %equation of the triangdistrload: 
            obj.gradient = -obj.intercept/obj.beamlength; %this equation only works if the TDL is over the whole half span (which it is in this case)

            %area of triangle represents the 'effective' force 
            obj.effective_triangular_force = 0.5*obj.intercept*obj.beamlength;

            obj.x_effective_force = obj.beamlength / 3; %the point at which this force acts: 1/3 from the root of beam or cut

            obj.load_factor = n;
        end

        %this function shows the tip force that is used to represent the coupled deflection between the top and bottom wing tips.
        function obj = setTipForce(obj, Rt, Rt_sign)
            obj.Rt = Rt;
            obj.Rt_sign = Rt_sign;
        end

        %this funciton calcultes the reaciton shear and moment
        function obj = reactionLoads(obj)
            obj.S_r = -(obj.effective_triangular_force + obj.Rt_sign * obj.Rt);
            obj.M_r = obj.effective_triangular_force * obj.x_effective_force + ...
                      obj.Rt_sign * obj.Rt * obj.beamlength;
        end
        
%% take a cut
        %when a cut is taken it will take that x value and put into the
        %triangular load equation to calculate the highest load value in
        %that triangular distributed load
        function w = loadAt(obj, x)
            w = obj.gradient .* x + obj.intercept;
        end
        

        %this calculates the area of the triangle that remains after the
        %cut
        function Ftri = remainingTriangularForce(obj, x)
            Lr = obj.beamlength - x;
            wc = obj.loadAt(x);
            Ftri = 0.5 .* wc .* Lr;
        end
        
        %calcultes the point at which the cut triangular distributed
        %force's effective force acts
        function xbar = remainingTriangularCentroid(obj, x)
            xbar = (obj.beamlength - x) ./ 3;
        end
        

        % Calcultes shear equation of the cut section
        function S = shearAt(obj, x)
            Ftri = obj.remainingTriangularForce(x);
            S = -(Ftri + obj.Rt_sign * obj.Rt);
        end
        

        % Calcultes moment equation of the cut section
        function M = momentAt(obj, x)
            Ftri = obj.remainingTriangularForce(x);
            xbar = obj.remainingTriangularCentroid(x);
            Lr = obj.beamlength - x;
            M = Ftri .* xbar + obj.Rt_sign * obj.Rt .* Lr;
        end

        function delta_tri = tipDeflectionTriangular(obj)
            delta_tri = obj.intercept * obj.beamlength^4 / (30 * obj.E * obj.I);
        end

        function delta_tip = tipDeflectionTipForce(obj)
            delta_tip = obj.Rt_sign * obj.Rt * obj.beamlength^3 / (3 * obj.E * obj.I);
        end

        function delta_total = tipDeflectionTotal(obj)
            delta_total = obj.tipDeflectionTriangular() + obj.tipDeflectionTipForce();
        end

        function [x, S, M] = diagrams(obj, npts)
            if nargin < 2
                npts = 200;
            end
            x = linspace(0, obj.beamlength, npts);
            S = arrayfun(@(xx) obj.shearAt(xx), x);
            M = arrayfun(@(xx) obj.momentAt(xx), x);
        end

        function printSummary(obj)
            fprintf('\n--- %s beam ---\n', obj.name);
            fprintf('Half-span L              = %.3f m\n', obj.beamlength);
            fprintf('Load factor n            = %.3f\n', obj.load_factor);
            fprintf('Lift split               = %.3f\n', obj.lift_split);
            fprintf('w0 at root               = %.3f kN/m\n', obj.intercept / 1000);
            fprintf('Gradient                 = %.6f kN/m^2\n', obj.gradient / 1000);
            fprintf('Equivalent triangular F  = %.3f MN\n', obj.effective_triangular_force / 1e6);
            fprintf('Rt on beam               = %.3f MN\n', obj.Rt / 1e6);
            fprintf('Root shear S_r           = %.3f MN\n', obj.S_r / 1e6);
            fprintf('Root moment M_r          = %.3f MNm\n', obj.M_r / 1e6);
            fprintf('Tip deflection           = %.4f m\n', obj.tipDeflectionTotal());
        end

    end
end
