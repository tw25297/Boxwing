function [TW_TO] = TOL(adp, WS_TO)
%TOL  Take-off field length constraint  —  T/W vs W/S
%
%  Uses Corke cubic correlation (imperial internally, SI in/out).
%  REF: Corke "Design of Aircraft", Ch.3
%
%  FIX: cast.atmos → BoxWing.cast.atmos

% Required ground run
sTO_req_ft = adp.TLAR.GroundRun * 3.28084;   % m → ft

% Airport atmosphere (SL, ISA + hot-day offset)
delta_T = adp.TLAR.ISA_deltaT_TO;
[~, ~, ~, ~, ~, ~, sigma] = BoxWing.cast.atmos(0, delta_T);

% CLmax at take-off
CL_max_TO = adp.CL_max + adp.Delta_Cl_to;

% W/S to imperial
WS_lbft2 = WS_TO * 0.020885;   % N/m² → lb/ft²

% TOP* (take-off parameter)
A = WS_lbft2 ./ (sigma .* CL_max_TO);

% Solve Corke cubic:  87*sqrt(A)*x^3 - sTO*x^2 + 20.9*A = 0,  x = sqrt(T/W)
TW_TO = NaN(size(WS_TO));
for i = 1:numel(WS_TO)
    Ai = A(i);
    if ~(isfinite(Ai) && Ai > 0), continue; end
    coeffs = [87*sqrt(Ai), -sTO_req_ft, 0, 20.9*Ai];
    r = roots(coeffs);
    r = r(imag(r) == 0 & real(r) > 0);
    if isempty(r), continue; end
    TW_TO(i) = min(real(r))^2;
end

TW_TO = TW_TO(:).';   % ensure row vector
end