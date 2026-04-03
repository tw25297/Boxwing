function WSmax_ceiling_lbft2 = Ceiling(obj)
%CEILING  Cruise ceiling constraint  —  returns max W/S in lb/ft²
%
%  Level flight at ceiling altitude: WS = CL * q
%  FIX: cast.atmos → BoxWing.cast.atmos

% Altitude: use Alt_max if defined, else cruise altitude
if isfield(obj.TLAR, 'Alt_max') && ~isempty(obj.TLAR.Alt_max)
    h = obj.TLAR.Alt_max;
else
    h = obj.TLAR.Alt_cruise;
end

M         = obj.TLAR.M_c;
CL_ceiling = 1.0;   % Corke: CL~1.0 for ceiling sizing

[rho, a] = BoxWing.cast.atmos(h);
q = 0.5 * rho * (M * a)^2;

WSmax_SI            = CL_ceiling * q;            % [N/m²]
WSmax_ceiling_lbft2 = WSmax_SI   * 0.020885;     % N/m² → lb/ft²
end