function [GeomObj, massObj] = fuselage(obj)
%FUSELAGE  Fuselage structure + all aircraft systems for boxwing freighter.
%  Mass fractions calibrated against A350F/B777F published OEM data.
%  Reference: Torenbeek App.C, Raymer Ch.15, Jane's All The World's Aircraft.
%  Target: OEM ~ 40-46% of MTOW (A350F=46%, B777F=44%).
%
%  All items named to avoid OEM filter exclusion:
%  (Do NOT name anything starting with "Fuel " or "Payload")

L_f   = obj.CockpitLength + obj.CabinLength + obj.CabinRadius * 1.48;
MTOM  = obj.MTOM;
M_dg  = MTOM * obj.Mf_TOC * SI.lb;    % design gross weight [lb]
n_z   = 2.5 * 1.5;                     % ultimate load factor
D     = (2 * obj.CabinRadius) * SI.ft; % fuselage diameter [ft]
L_ft  = L_f * SI.ft;                   % fuselage length [ft]

%% Geometry (top-view outline)
theta = linspace(0,pi,101)';
Xs = [-sin(theta)*obj.CockpitLength, cos(theta)*obj.CabinRadius];
theta = linspace(pi,0,101)';
Xs = [Xs;
      sin(theta)*obj.CabinRadius*2*2.48 + (obj.CabinLength-obj.CabinRadius*2), ...
      cos(theta)*obj.CabinRadius];
Xs(:,1) = Xs(:,1) + obj.CockpitLength;
GeomObj = BoxWing.cast.GeomObj(Name="Fuselage", Xs=Xs);

%%  1. Fuselage Structure  (Raymer 15.7 and CFRP factor 0.78) 
K_d = 1.04;   K_Lg = 1.12;
S_f = pi*D*(obj.CabinLength*SI.ft) ...
    + pi*(obj.CabinLength*SI.ft)*(obj.CabinRadius*SI.ft) ...
    + pi*(1.48*obj.CabinRadius*SI.ft)*(obj.CabinRadius*SI.ft);
W_fus = 0.3280 * K_d * K_Lg * sqrt(M_dg*n_z) ...
      * L_f^0.25 * S_f^0.302 * (L_f/D)^0.10;
m_fus = (W_fus / SI.lb) * 0.78;    % assuming 22% CFRP saving
% WARNING
m_fus = mean(m_fus);  % ASSUMPTION MADE TO DEBUG
massObj = BoxWing.cast.MassObj(Name="Fuselage Structure", m=m_fus, X=[L_f/2; 0]);

%% 2. Flight Controls  
% Fly-by-wire system: actuators, control surfaces, sensors, computers.
% Fraction method (Raymer 15.6 over-predicts for large aircraft):
%   B777:  ~8000 kg = 2.7% MTOW
%   A350:  ~6000 kg = 1.9% MTOW  (more-electric, lighter actuators)
%   Boxwing: extra surfaces (8 control surfaces vs 4) -> 2.8%
m_fc = 0.028 * MTOM;
% WARNING
m_fc = mean(m_fc);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Flight Controls", m=m_fc, X=[L_f/2; 0]);

%%  3. Air Conditioning & Pressurisation  
% Environmental Control System (ECS):
%   B777:  ~7500 kg = 2.5% MTOW
%   A350:  ~6000 kg = 1.9% MTOW  (electric ECS, no bleed)
%   Boxwing: conventional bleed-air -> 2.2%
m_ac = 0.022 * MTOM;
% WARNING
m_ac = mean(m_ac);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Air Conditioning", m=m_ac, X=[L_f*0.45; 0]);

%%  4. Ice Protection  
% Hot-air wing LE + electrothermal tail LE:
%   Typical wide-body: 0.2% MTOW
m_ice = 0.002 * MTOM;
% WARNING
m_ice = mean(m_ice);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Ice Protection", m=m_ice, X=[L_f*0.3; 0]);

%%  5. Fire Protection  
% Engine bay, APU bay, cargo hold suppression systems
m_fire = 0.003 * MTOM;    % ~960 kg for 319t
% WARNING
m_fire = mean(m_fire);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Fire Protection", m=m_fire, X=[L_f/2; 0]);

%%  6. Avionics  (Raymer eq 15.8) 
% W_av = 2.117 * W_uav^0.933  [lb]
W_uav   = 800 * SI.lb;
m_avion = 2.117 * W_uav^0.933 / SI.lb;   % ~1026 kg
% WARNING
m_avion = mean(m_avion);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Avionics", m=m_avion, X=[obj.CockpitLength*0.5; 0]);

%%  7. Hydraulics  
% 3 independent circuits (normal / alternate / emergency)
%   B777: ~6000 kg = 2.0% MTOW
%   A350: ~3500 kg (partial electric backup) = 1.1%
%   Boxwing (un-conventional): 1.8%
m_hyd = 0.018 * MTOM;
% WARNING
m_hyd = mean(m_hyd);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Hydraulics", m=m_hyd, X=[L_f/2; 0]);

%% 8. Electrical Systems  (Raymer eq 15.12) 
% W_el = 7.291 * R_kva^0.782 * L_ft^0.346 * N_gen^0.1  [lb]
R_kva = 400;   N_gen = 4;
m_elec = 7.291 * R_kva^0.782 * L_ft^0.346 * N_gen^0.1 / SI.lb;  % ~2642 kg
% WARNING
m_elec = mean(m_elec);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Electrical Systems", m=m_elec, X=[L_f/2; 0]);

%% 9. APU  
m_apu = 1200;   % kg — Honeywell HGT1700 class
massObj(end+1) = BoxWing.cast.MassObj(Name="APU", m=m_apu, X=[L_f*0.92; 0]);

%%  10. Cargo Handling System  
% Floor rollers, ball mats, net rails, tie-downs, Cargo Loading System unit
%   B777F: ~5500 kg = 1.8% MTOW
%   A350F: ~5000 kg = 1.6% MTOW
m_cargo = 0.017 * MTOM;
% WARNING
m_cargo = mean(m_cargo);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Cargo Handling", m=m_cargo, X=[L_f*0.50; 0]);

%% 11. Oxygen & Safety Equipment  
m_oxy = 400 + obj.TLAR.Crew * 15;
% WARNING
m_oxy = mean(m_oxy);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Oxygen & Safety", m=m_oxy, X=[obj.CockpitLength; 0]);

%%  12. Interior Finish  
% Freighter: acoustic lining, floor panels, cargo compartment walls
%   ~1% MTOW
m_int = 0.010 * MTOM;
% WARNING
m_int = mean(m_int);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Interior Finish", m=m_int, X=[L_f*0.5; 0]);

%% 13. Unusable Fuel & Trapped Oil  
m_unusable = 0.010 * MTOM * obj.Mf_Fuel;
% WARNING
m_unusable = mean(m_unusable);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Unusable Fuel & Oil", m=m_unusable, X=[L_f/2; 0]);

%% 14. Paint  
m_paint = 0.004 * MTOM;   
% WARNING
m_paint = mean(m_paint);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Paint", m=m_paint, X=[L_f/2; 0]);

%%  15. Tank Systems  (Raymer eq 15.11) 
% W_fs = 2.405 * V_t^0.606 * N_t^0.5 * N_eng^0.5  [lb]
FuelMass = MTOM * obj.Mf_Fuel;
V_t_gal  = (FuelMass / BoxWing.cast.eng.Fuel.JA1.Density * SI.litre) / 3.785;
m_tanks  = 2.405 * V_t_gal^0.606 * 4^0.5 * 2^0.5 / SI.lb;
% WARNING
m_tanks = mean(m_tanks);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Tank Systems", m=m_tanks, X=[L_f/2; 0]);

%%  16. Wiring Harness  
% Electrical wiring throughout aircraft body and wings:
%   B777: ~8000 kg = 2.7% MTOW (320 km of wire!)
%   A350: ~5000 kg = 1.6% MTOW (CFRP structure, shorter runs)
%   Boxwing: 2.0% MTOW
m_wiring = 0.020 * MTOM;
% WARNING
m_wiring = mean(m_wiring);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Wiring Harness", m=m_wiring, X=[L_f/2; 0]);

%%  17. Manufacturing & Weight Growth Margin  
% Standard 2% structural mass margin for weight growth during development.
% Accounts for: fasteners, sealants, shimming, as-built vs design mass.
m_margin = 0.020 * MTOM;
% WARNING
m_margin = mean(m_margin);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Weight Margin (2%)", m=m_margin, X=[L_f/2; 0]);

%%  18. Operator Items  
m_oper = obj.TLAR.CrewMass + 800;   % assuming crew mass + 800 kg misc
% WARNING
m_oper = mean(m_oper);  % ASSUMPTION MADE TO DEBUG
massObj(end+1) = BoxWing.cast.MassObj(Name="Operator Items", m=m_oper, X=[obj.CockpitLength*0.5; 0]);

end