function [GeomObj,massObj] = engine(obj)
% engine - this function build the engines for a B777 like aircraft
% engine is based upon a rubberised version of the GE90

% Ensure baseline engine exists (geometry + TSFC model)
if isempty(obj.Engine)
    obj.Engine = BoxWing.cast.eng.TurboFan.CFM_LEAP_1A(1, obj.TLAR.Alt_cruise, obj.TLAR.M_c);   %from YUke
end

Eng0 = obj.Engine;  % keep original geometry fields

% Rubberise to match required thrust
obj.Engine = obj.Engine.Rubberise(obj.Thrust/2);

% If Rubberise nuked key properties, restore from baseline
if isempty(obj.Engine.Length),   obj.Engine.Length   = Eng0.Length;   end
if isempty(obj.Engine.Diameter), obj.Engine.Diameter = Eng0.Diameter; end
if isempty(obj.Engine.Mass),     obj.Engine.Mass     = Eng0.Mass;     end

% --------------------------- Create Geometry ----------------------------
Xs = [-0.5,0.5; 0.5,0.5; 0.5,-0.5; -0.5,-0.5];

L = obj.Engine.Length;
D = obj.Engine.Diameter;

Xs = Xs .* repmat([L D], size(Xs,1), 1);

offsetEng = [obj.x_ac - obj.c_ac*0.6, obj.CabinRadius + 1.75*D];
GeomObj = BoxWing.cast.GeomObj(Name="EngineRight", Xs = Xs + offsetEng);
GeomObj(2) = BoxWing.cast.GeomObj(Name="EngineLeft", Xs = Xs + offsetEng.*[1 -1]);

% ------------------------- Create Mass Objects --------------------------
mEng = obj.Engine.Mass;
mEng = double(mEng);
mEng = mEng(1);   % now safe because we ensured not empty

% engine installation mass (Raymer 15.52) -> also force scalar
m_engi = 1.1*(2.575*(mEng*SI.lb)^0.922)./SI.lb - mEng;
m_engi = double(m_engi); 
m_engi = m_engi(1);

offsetPylon = offsetEng + [L/2, 0];

massObj         = BoxWing.cast.MassObj(Name="Engine Right",       m=mEng,  X=offsetEng);
massObj(end+1)  = BoxWing.cast.MassObj(Name="Engine Pylon Right", m=m_engi, X=offsetPylon);
massObj(end+1)  = BoxWing.cast.MassObj(Name="Engine Left",        m=mEng,  X=offsetEng.*[1 -1]);
massObj(end+1)  = BoxWing.cast.MassObj(Name="Engine Pylon Left",  m=m_engi, X=offsetPylon.*[1 -1]);
end