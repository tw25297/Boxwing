function WS_Max_LFL = LFL(obj)
%LFL  Landing field length constraint  —  returns max W/S in lb/ft²
%
%  Corke landing parameter:  LP = (s_land_ft - 400) / 118
%  WS_max = sigma * CL_max_land * LP
%  FIX: cast.atmos → BoxWing.cast.atmos

sL_ft = obj.TLAR.GroundRunLanding * 3.28084;   % m → ft

CL_max_landing = obj.CL_max + obj.Delta_Cl_ld;

% Sea-level ISA density ratio
[~, ~, ~, ~, ~, ~, sigma] = BoxWing.cast.atmos(0);

LP = (sL_ft - 400) / 118;   % landing parameter [lb/ft²] (Corke)
WS_Max_LFL = sigma * CL_max_landing * LP;   % [lb/ft²]
end