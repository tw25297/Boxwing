function TW_alt = MACS(obj, WS_SI)
%MACS  Cruise drag constraint  —  T/W vs W/S at cruise altitude
%
%  T/W = q*CD0/WS + WS/(q*pi*AR*e)   (Corke eq 3.13 at cruise)
%  FIX: cast.atmos → BoxWing.cast.atmos

WS_SI = WS_SI(:).';   % force row vector

% Cruise condition
[rho, a] = BoxWing.cast.atmos(obj.TLAR.Alt_cruise);
V = obj.TLAR.M_c * a;
q = 0.5 * rho * V^2;

% Aero parameters
CD0 = obj.CD0;
e   = obj.e;

% Aspect ratio
AR = obj.AR();
if ~isfinite(AR) || AR <= 0
    if ~isempty(obj.AeroPolar) && isprop(obj.AeroPolar, 'AR')
        AR = obj.AeroPolar.AR;
    else
        AR = 9.5;
        warning('MACS: AR not available, using fallback AR=%.1f', AR);
    end
end

TW_alt = (q * CD0 ./ WS_SI) + (WS_SI ./ (q * pi * AR * e));
TW_alt = TW_alt(:).';
end