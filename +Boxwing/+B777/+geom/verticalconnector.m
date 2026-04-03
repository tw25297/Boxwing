function [GeomObj, massObj] = verticalconnector(obj)
%VERTICALCONNECTOR  Wingtip fins that close the boxwing loop.

h   = obj.ConnectorHeight;   % [m] vertical gap
c   = 3.5;                   % [m] connector chord

x_front    = obj.FrontWingPos + obj.FrontWingArea / obj.FrontWingSpan;
x_rear     = obj.RearWingPos;
half_f     = obj.FrontWingSpan / 2;
half_r     = obj.RearWingSpan  / 2;

%% Geometry (top-down quad for each connector)
Xs_L = [x_front,   -half_f;
        x_rear,    -half_r;
        x_rear+c,  -half_r;
        x_front+c, -half_f];
Xs_R = Xs_L;  Xs_R(:,2) = -Xs_R(:,2);

GeomObj    = BoxWing.cast.GeomObj(Name="Left Connector",  Xs=Xs_L);
GeomObj(2) = BoxWing.cast.GeomObj(Name="Right Connector", Xs=Xs_R);

%% Mass  (VTP-style Raymer)
[rho, a] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
q_c      = 0.5 * rho * (obj.TLAR.M_c * a)^2;

S_conn  = h * c;
AR_conn = h / c;
tr_c    = 0.5;
tcr = 0.12;  tct = 0.10;
M_dg    = obj.MTOM * obj.Mf_TOC * SI.lb;

m_c = 0.073*(1+0.2*0)*(1.5*2.5*M_dg)^0.376 ...
    * (SI.lb/SI.ft^2 * q_c)^0.122 ...
    * (S_conn*SI.ft^2)^0.873 ...
    * (100*(tcr+tct)/2/cosd(15))^-0.49 ...
    * (AR_conn/cosd(15)^2)^0.357 * tr_c^0.039;
m_c = (m_c/SI.lb) * 1.30 * 0.70;   % assuming +30% attach, -30% CFRP
% WARNING
m_c = mean(m_c);  % ASSUMPTION MADE TO DEBUG
x_cg = (x_front + x_rear)/2 + c/2;

massObj    = BoxWing.cast.MassObj(Name="Left Connector",  m=m_c, X=[x_cg; -half_f]);
massObj(2) = BoxWing.cast.MassObj(Name="Right Connector", m=m_c, X=[x_cg;  half_r]);
end
