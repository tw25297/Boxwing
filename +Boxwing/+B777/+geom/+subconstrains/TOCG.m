function [TW_TOCG] = TOCG(obj, WS)
%TOCG  Take-off climb gradient (OEI) constraint  —  T/W vs W/S
%
%  FAR 25 second-segment climb gradient for twin engine: G_min = 0.024
%  OEI factor halves available thrust.
%  FIX: cast.atmos → BoxWing.cast.atmos

WS = WS(:).';   % force row vector

G_min = 0.024;   % FAR 25 twin OEI second segment

% Sea-level atmosphere
[rho, ~] = BoxWing.cast.atmos(0);

CLmax_TO = obj.CL_max + obj.Delta_Cl_to;
Vs_TO    = sqrt(2 .* WS ./ (rho .* CLmax_TO));
V        = 1.25 .* Vs_TO;   % V2 = 1.25 Vs
q        = 0.5 .* rho .* V.^2;

% Drag polar
if ~isempty(obj.AeroPolar)
    CL     = WS ./ q;
    CD     = obj.AeroPolar.CD(CL);
    DoverW = CD ./ CL;
else
    CD0    = 0.03;   % high-lift + gear
    AR     = obj.AR();
    e      = 0.80;
    DoverW = q .* CD0 ./ WS + WS ./ (q * pi * AR * e);
end

% OEI for twin
OEI_factor = (2 - 1) / 2;   % = 0.5

TW_TOCG = (DoverW + G_min) ./ OEI_factor;
TW_TOCG = TW_TOCG(:).';
end