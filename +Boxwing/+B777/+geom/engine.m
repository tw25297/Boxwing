function [GeomObj,massObj] = engine(obj)
% ENGINE  Build engine geometry and mass for the boxwing freighter.

%         Engines mounted under the REAR wing (user specification).
%         Based on a rubberised GE90-115B scaled to required thrust.
%
% Rubberising is done inline here so this function has no dependency on
% obj.Engine having a .Rubberise method (which is not reliably preserved
% across handle-class property assignments in iterative sizing loops).

% WITH THIS (UltraFan):
T_ref = 380e3;   % [N]   UltraFan SLS thrust per engine
D_ref = 4.50;    % [m]   fan diameter
L_ref = 6.50;    % [m]   overall length
M_ref = 6500;    % [kg]  dry mass per engine
%% ── Required thrust per engine ───────────────────────────────────────
% obj.Thrust is total aircraft thrust [N]; two engines assumed
T_req = max(obj.Thrust / 2, 50e3);   % floor at 50 kN

%% ── Rubber scaling laws (Raymer Ch.10 / Mattingly) ───────────────────
%    D, L  ∝  T^(1/3)   (fan area ∝ thrust)
%    Mass  ∝  T^0.9     (empirical high-BPR turbofan)
ratio = T_req / T_ref;
D = D_ref * ratio^(1/3);
L = L_ref * ratio^(1/3);
M = M_ref * ratio^0.9;

%% ── Override with obj.Engine geometry if already rubberised ──────────
% If the caller provided an engine object with explicit geometry, use it
% (allows the runner to supply a custom engine deck).
if ~isempty(obj.Engine)
    if isfield(obj.Engine,'Diameter') && ~isempty(obj.Engine.Diameter) && isfinite(obj.Engine.Diameter)
        % Re-rubberise from the stored baseline values if available
        if isfield(obj.Engine,'T_ref') && ~isempty(obj.Engine.T_ref)
            ratio2 = T_req / obj.Engine.T_ref;
            D = obj.Engine.D_ref * ratio2^(1/3);
            L = obj.Engine.L_ref * ratio2^(1/3);
            M = obj.Engine.M_ref * ratio2^0.9;
        end
    end
end

% Store rubberised values back so other functions (CD0, landingGear) see them
obj.Engine.Diameter = D;
obj.Engine.Length   = L;
obj.Engine.Mass     = M;

%% ── Geometry ─────────────────────────────────────────────────────────
Xs = [-0.5, 0.5; 0.5, 0.5; 0.5, -0.5; -0.5, -0.5];
Xs = Xs .* repmat([L D], size(Xs,1), 1);

% ENGINE PLACEMENT: under REAR wing (matching user version)
% X = RearWingPos set back by 0.3*L (under rear wing leading edge area)
% Y = cabin radius + 1.60*D (under-wing ground clearance)
offsetEng  = [obj.RearWingPos - L*0.3,  obj.CabinRadius + 1.60*D];
GeomObj    = Boxwing.cast.GeomObj(Name="EngineRight", Xs = Xs + offsetEng);
GeomObj(2) = Boxwing.cast.GeomObj(Name="EngineLeft",  Xs = Xs + offsetEng.*[1 -1]);

%% ── Mass ─────────────────────────────────────────────────────────────
mEng   = double(M);
% Engine installation / pylon mass (Raymer eq 15.52)
m_engi = 1.1 * (2.575 * (mEng * SI.lb)^0.922) / SI.lb - mEng;
m_engi = double(m_engi(1));

offsetPylon = offsetEng + [L/2, 0];

massObj        = Boxwing.cast.MassObj(Name="Engine Right",       m=mEng,   X=offsetEng);
massObj(end+1) = Boxwing.cast.MassObj(Name="Engine Pylon Right", m=m_engi, X=offsetPylon);
massObj(end+1) = Boxwing.cast.MassObj(Name="Engine Left",        m=mEng,   X=offsetEng.*[1 -1]);
massObj(end+1) = Boxwing.cast.MassObj(Name="Engine Pylon Left",  m=m_engi, X=offsetPylon.*[1 -1]);
end