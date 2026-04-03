function [GeomObj, massObj] = rearwing(obj)
%REARWING  Aft lifting surface with FORWARD SWEEP (horizontal tail function)

%%  Planform parameters 
SweepQtrChord = -20;         % Assuming NEGATIVE = forward sweep [deg]
tr            = 0.38;        %  assuming taper ratio

b   = obj.RearWingSpan;
S   = obj.RearWingArea;
R_f = obj.CabinRadius;

c_r = 2*S / (b*(1+tr));
c_t = tr * c_r;

%%  Coordinates (FORWARD SWEEP) 
ys = [-b/2, -R_f, 0, R_f, b/2]';
cs = [ c_t,  c_r, c_r, c_r, c_t]';

% FORWARD sweep: leading edge moves FORWARD as you go outboard
sweepLE = atand( tand(SweepQtrChord) + (c_r - c_t)/(4*(b/2)) );
x_le    = [-tand(sweepLE)*(b/2), 0, 0, 0, -tand(sweepLE)*(b/2)]';
x_te    = x_le + cs;

Xs = [x_le, ys; flipud(x_te), flipud(ys)];
Xs(:,1) = Xs(:,1) + obj.RearWingPos;

GeomObj = BoxWing.cast.GeomObj(Name="Rear Wing", Xs=Xs);

%%  Mass (Raymer equation with forward sweep corrections) 
b_w       = b * SI.ft;
S_w       = S * SI.ft^2;
t_w       = 0.12 * c_r * SI.ft;
cosLambda = cosd(abs(SweepQtrChord));  % Use absolute value

Wdg_lb = obj.MTOM * obj.Mf_TOC * SI.lb;
n_z    = 2.0 * 1.5;  % Lower load factor (acts as tail)

w_wing = 0.00125 * Wdg_lb * (b_w/cosLambda)^0.75 * (1 + sqrt(6.3*cosLambda/b_w)) * n_z^0.55 * (b_w*S_w ./ (t_w*Wdg_lb*cosLambda))^0.3;
% WARNING
w_wing = mean(w_wing); % THIS IS DONE TO DEBUG THE CODE
% Forward sweep penalty and composite factor
forward_sweep_penalty = 1.12;  % Assuming +12% for aeroelastic concerns
composite_factor = 0.75;       % CFRP weight reduction

m_wing = (w_wing / SI.lb) * composite_factor * forward_sweep_penalty;

c_mac_rear = (2/3)*c_r*(1 + tr + tr^2)/(1 + tr);

massObj = BoxWing.cast.MassObj(Name="Rear Wing", m=m_wing, ...
                       X=[obj.RearWingPos + c_mac_rear*0.25; 0]);
end