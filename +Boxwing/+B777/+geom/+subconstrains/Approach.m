function WS_Max_Vapp = Approach(obj)
%APPROACH  Approach speed (stall) constraint  —  returns max W/S in lb/ft²
%
%  FAR 25: V_app = 1.3 * V_stall  →  V_stall = V_app / 1.3
%  WS = 0.5 * rho * Vstall² * CLmax_land
%  FIX: cast.atmos → BoxWing.cast.atmos

Vstall = obj.TLAR.V_app / 1.3;   % [m/s]

CL_max_landing = obj.CL_max + obj.Delta_Cl_ld;

% Sea-level ISA
[rho, ~] = BoxWing.cast.atmos(0);

WS_SI = 0.5 * rho * Vstall^2 * CL_max_landing;   % [N/m²]

% Convert to lb/ft² (to match LFL and Ceiling outputs for the main function)
WS_Max_Vapp = WS_SI * 0.020885;   % N/m² → lb/ft²
end