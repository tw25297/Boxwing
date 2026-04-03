function [TW_ROC] = ROC(obj, WS)
%ROC  Rate-of-climb constraint at cruise altitude  —  T/W vs W/S
%
%  Based on Corke eq 3.13:  T/W = D/W + ROC/V
%  FIX: cast.atmos → BoxWing.cast.atmos

WS = WS(:).';   % force row vector

% Requirement
ROC_req = (obj.TLAR.ROC_min_at_cruise / 3.28084) / 60;   % ft/min → m/s

% Atmosphere at cruise altitude
[rho, a] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
V = obj.TLAR.M_c * a;
q = 0.5 * rho * V^2;

% Drag polar
if ~isempty(obj.AeroPolar)
    CL   = WS ./ q;
    CD   = obj.AeroPolar.CD(CL);
    DtoW = CD ./ CL;
else
    CD0 = obj.CD0;
    AR  = obj.AR();
    e   = obj.e;
    DtoW = q .* CD0 ./ WS + WS ./ (q * pi * AR * e);
end

TW_ROC = DtoW + ROC_req ./ V;
TW_ROC = TW_ROC(:).';
end