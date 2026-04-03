function [GeomObj, massObj] = verticaltail(obj)
%VERTICALTAIL  Conventional vertical stabilizer at fuselage tail
%   Integrates with rear wing

%%  Geometry parameters 
S_vt = 90;          % m² vertical tail area
b_vt = 10;          % m  tail height
c_r_vt = 11;        % m  root chord
c_t_vt = 3.5;       % m  tip chord
sweep_LE = 42;      % deg leading edge sweep

% Position at tail
L_f = obj.CockpitLength + obj.CabinLength + obj.CabinRadius*1.48;
x_tail_root = obj.RearWingPos - 2.0;

%%  Planform coordinates 
% For top-down view visualization (project onto x-y plane)
Xs = [x_tail_root, 0; 
      x_tail_root + c_r_vt, 0;
      x_tail_root + c_r_vt + tand(sweep_LE)*b_vt - c_t_vt, b_vt/10;
      x_tail_root + tand(sweep_LE)*b_vt, b_vt/10];

GeomObj = BoxWing.cast.GeomObj(Name="Vertical Tail", Xs=Xs);

%%  Mass (Raymer VTP equation) 
[rho, a] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
q_c = 0.5 * rho * (obj.TLAR.M_c * a)^2;

M_dg = obj.MTOM * obj.Mf_TOC * SI.lb;
AR_vt = b_vt^2 / S_vt;
taper_vt = c_t_vt / c_r_vt;
t_c_avg = 0.11;

% Raymer Eq. 15.31
m_vt = 0.073 * (1 + 0.2*0) * (1.5*2.5*M_dg)^0.376 ...
     * (SI.lb/SI.ft^2 * q_c)^0.122 ...
     * (S_vt*SI.ft^2)^0.873 ...
     * (100*t_c_avg/cosd(sweep_LE))^-0.49 ...
     * (AR_vt/cosd(sweep_LE)^2)^0.357 * taper_vt^0.039;

% Material factor
composite_factor = 0.70;  % CFRP
integration_factor = 0.95;  % Integrated with rear wing

m_vt = (m_vt / SI.lb) * composite_factor * integration_factor;
% WARNING
m_vt = mean(m_vt);  % ASSUMPTION MADE TO DEBUG
% CG location
x_cg = x_tail_root + c_r_vt*0.42 + tand(sweep_LE)*b_vt*0.40;

massObj = BoxWing.cast.MassObj(Name="Vertical Tail", m=m_vt, X=[x_cg; 0]);
end