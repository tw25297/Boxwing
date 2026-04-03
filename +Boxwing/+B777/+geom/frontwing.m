function [GeomObj, massObj] = frontwing(obj)
%FRONTWING  Forward lifting surface of the boxwing freighter.
%  Includes primary structure + control surfaces (flaps, ailerons, spoilers).
%  Composite factor 0.75 
%fprintf('DEBUG frontwing: FrontWingSpan=%s, FrontWingArea=%s\n', ...
%    mat2str(obj.FrontWingSpan), mat2str(obj.FrontWingArea));
%% Planform
SweepQtrChord = 25;
tr  = 0.35;
b   = obj.FrontWingSpan;
S   = obj.FrontWingArea;
R_f = obj.CabinRadius;

c_r = 2*S / (b*(1+tr));
c_t = tr * c_r;

%% Coordinates
ys = [-b/2, -R_f, 0, R_f, b/2]';
cs = [c_t,   c_r, c_r, c_r, c_t]';
sweepLE = atand( tand(SweepQtrChord) + (c_r-c_t)/(4*(b/2)) );
x_le    = [tand(sweepLE)*(b/2), 0, 0, 0, tand(sweepLE)*(b/2)]';
x_te    = x_le + cs;
Xs      = [x_le, ys; flipud(x_te), flipud(ys)];
Xs(:,1) = Xs(:,1) + obj.FrontWingPos;
GeomObj = BoxWing.cast.GeomObj(Name="Front Wing", Xs=Xs);

%% Store MAC / AC
obj.c_ac = (2/3) * c_r * (1 + tr + tr^2) / (1 + tr);
obj.x_ac = obj.FrontWingPos + obj.c_ac * 0.25;

%% Primary structure mass  (Raymer and 0.75 CFRP factor)
b_w    = b * SI.ft;
S_w    = S * SI.ft^2;
t_w    = 0.13 * c_r * SI.ft;
cosLam = cosd(SweepQtrChord);
Wdg_lb = obj.MTOM * obj.Mf_TOC * SI.lb;
n_z    = 2.5 * 1.5;

w_wing = 0.00125 * Wdg_lb * (b_w/cosLam)^0.75 * (1 + sqrt(6.3*cosLam/b_w)) * n_z^0.55 * (b_w*S_w ./ (t_w*Wdg_lb*cosLam))^0.3;

m_struct = (w_wing / SI.lb) * 0.75;   %  assuming 25% CFRP saving 

%% Control surfaces  (flaps + ailerons + spoilers = 9% of wing structure)
m_surfaces = m_struct * 0.09;
m_wing = m_struct + m_surfaces;
% WARNING
m_wing = mean(m_wing);  % average over spanwise stations THIS WAS ADDED TO FIX A BUG, CHECK IF IT'S OKAY
%m_wing = 7000; % kg

massObj = BoxWing.cast.MassObj(Name="Front Wing", m=m_wing, X=[obj.FrontWingPos + obj.c_ac*0.25; 0]);
end
