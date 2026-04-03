function [GeomObj, massObj] = BuildGeometry(obj, opts)
%BUILDGEOMETRY  Assemble all geometry and mass objects for the boxwing.

arguments
    obj
    opts.FuelFraction    = 1;
    opts.PayloadFraction = 1;
end

GeomObj = struct.empty;
massObj = struct.empty;

components = ["frontwing", "rearwing", "verticalconnector", ...
              "verticaltail","fuselage","engine", "landingGear"];
             

for i = 1:length(components)
    [gTmp, mTmp] = BoxWing.B777.geom.(components(i))(obj);
    GeomObj = [GeomObj, gTmp];
    massObj = [massObj, mTmp];
end

BoxWing.B777.liftingSurfaceAC(obj, 'verbose', false); %lifting surface forr both surfaces

% This is to make the aircraft nose heavy 
% Ballast for CG control
m_ballast = 0;  % kg (3 tonnes)
x_ballast = 3.0;   % m (nose gear bay)
massObj(end+1) = BoxWing.cast.MassObj(Name="Ballast", m=m_ballast, X=[x_ballast; 0]);

%% Fuel: 60% front wing, 40% rear wing and we can chnage this according to the CG
fuel = obj.MTOM * obj.Mf_Fuel * opts.FuelFraction;
% WARNING
fuel = mean(fuel); % ASSUMPTION TO DEBBUG CODE
massObj(end+1) = BoxWing.cast.MassObj(Name="Fuel Front Wing", ...
    m=fuel*0.80, X=[obj.FrontWingPos + obj.c_ac*0.15; 0]);
massObj(end+1) = BoxWing.cast.MassObj(Name="Fuel Rear Wing", ...
    m=fuel*0.20, X=[obj.RearWingPos  + 2.0; 0]);

%% Payload
m_payload = obj.TLAR.Payload * opts.PayloadFraction;
% WARNING
m_payload = mean(m_payload); % ASSUMPTION TO DEBBUG CODE
massObj(end+1) = BoxWing.cast.MassObj(Name="Payload", ...
    m=m_payload, ...
    X=[obj.CockpitLength + obj.CabinLength*0.50; 0]);
end
