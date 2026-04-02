function [GeomObj,massObj] = fuselage(obj)
% fuselage - Builds the fuselage geometry and high-fidelity segmented mass
%            model for a B777F-like Box-Wing aircraft.
%
% Mass model overview:
%   1. Raymer correlation gives an aluminium-equivalent baseline mass.
%   2. Fuselage is split into 6 functional segments (nose, cockpit, forward
%      barrel, wing-box bay, aft barrel, tailcone) with calibrated per-metre
%      mass multipliers that rescale to match the Raymer total exactly.
%   3. Each segment is further split into skin+stringers vs frames+bulkheads
%      using the fus_ratio_skin / fus_ratio_frames parameters from ADP_BW.
%   4. A CFRP fraction and Kmat weight-saving factor are applied independently
%      to skins and frames in every segment.
%   5. A nose cargo-door structural penalty is added as a separate MassObj.
%
% Output massObj entries (in order):
%   Fuselage Nose | Fuselage Cockpit | Fuselage Fwd Barrel |
%   Fuselage WingBox | Fuselage Aft Barrel | Fuselage Tailcone |
%   Nose Cargo Door | Systems | Fuel Systems


%% =====================================================================
%  1.  FUSELAGE GEOMETRY
% ======================================================================

% Total fuselage length  [m]
L_f = obj.CockpitLength + obj.CabinLength + obj.CabinRadius * 1.48;

% --- Nose / cockpit profile (half-ellipse swept back) ---
theta = linspace(0, pi, 101)';
Xs = [-sin(theta)*obj.CockpitLength, cos(theta)*obj.CabinRadius];

% --- Tailcone profile ---
theta = linspace(pi, 0, 101)';
Xs = [Xs; ...
    sin(theta)*obj.CabinRadius*2*2.48 + (obj.CabinLength - obj.CabinRadius*2), ...
    cos(theta)*obj.CabinRadius];

% Shift so x = 0 at nose tip
Xs(:,1) = Xs(:,1) + obj.CockpitLength;
GeomObj = cast.GeomObj(Name="Fuselage", Xs=Xs);


%% =====================================================================
%  2.  RAYMER ALUMINIUM BASELINE MASS
% ======================================================================

K_d  = 1.12;                                   % transport damage factor
K_Lg = 1.12;                                   % fuselage-mounted LG factor
M_dg = obj.MTOM * obj.Mf_TOC * SI.lb;         % design gross weight at TOC [lb]
n_z  = 2.5 * 1.5;                              % ultimate manoeuvre load factor
D    = (2 * obj.CabinRadius) * SI.ft;          % max fuselage diameter [ft]
b_w  = obj.Span * SI.ft;                       % wing span [ft]

% Quarter-chord sweep and taper ratio
SweepQtrChord = real(acosd(0.75 .* obj.Mstar ./ obj.TLAR.M_c));  % [deg]
tr = -0.0083 * SweepQtrChord + 0.4597;

% Raymer K_ws (wing-fuselage bending relief)
K_ws = 0.75 * ((1 + 2*tr) / (1 + tr)) * (b_w / L_f) * tand(SweepQtrChord);

% Box-wing corrections: distributed lift reduces peak fuselage bending
k_bw_n  = 0.80;    % reduced effective load factor (20% relief from rear wing)
k_bw_K  = 0.85;    % smaller wing-span influence term
n_z_eff = k_bw_n * n_z;
K_ws    = k_bw_K  * K_ws;

% First-order fuselage wetted area [ft^2]
S_f = pi*D*(obj.CabinLength*SI.ft) + ...
      pi*(obj.CabinLength*SI.ft)*(obj.CabinRadius*SI.ft) + ...
      pi*(1.48*obj.CabinRadius*SI.ft)*(obj.CabinRadius*SI.ft);

% Raymer transport fuselage weight [lb]
W_fus_lb = 0.3280 * K_d * K_Lg * sqrt(M_dg * n_z_eff) * ...
           (L_f^0.25) * (S_f^0.302) * ((1 + K_ws)^0.04) * ((L_f / D)^0.10);

% Aluminium-equivalent baseline mass [kg]  (x1.5 = Raymer lb-to-kg empirical scale)
m_fus_Al = W_fus_lb / SI.lb * 1.5;


%% =====================================================================
%  3.  SEGMENT LENGTHS
% ======================================================================

% Primary fuselage divisions  [m]
L_cockpit  = obj.CockpitLength;
L_cabin    = obj.CabinLength;
L_tailcone = obj.CabinRadius * 1.48;   % matches geometry definition above

% Split cockpit into unpressurised nose cone and pressurised cockpit shell
L_nose = 0.40 * L_cockpit;   % forward 40% -> ogive / nose cone
L_cpit = 0.60 * L_cockpit;   % aft 60%     -> pressurised cockpit bay

% Wing-box bay length approx 1.2 x fuselage diameter (spans wing carry-through frame)
L_wbox = 1.2 * (2 * obj.CabinRadius);

% Remaining cabin length split equally fore and aft of the wing box
L_fwdBar = max(0.0,  0.5 * (L_cabin - L_wbox));
L_aftBar = max(0.0,  0.5 * (L_cabin - L_wbox));


%% =====================================================================
%  4.  ALUMINIUM SEGMENT MASSES  (calibrated to Raymer total)
% ======================================================================
% Average mass per metre, then scale each segment by its structural
% intensity multiplier (kseg_* from ADP_BW).  The entire array is then
% renormalised so that the sum equals m_fus_Al exactly.

w0 = m_fus_Al / L_f;   % average fuselage mass per metre [kg/m]

% Segment multipliers from ADP properties
kN  = obj.kseg_nose;        % 0.8  - light ogive structure
kC  = obj.kseg_cockpit;     % 1.3  - complex cutouts, pressurised
kFB = obj.kseg_fwdBarrel;   % 1.0  - nominal barrel
kWB = obj.kseg_wingbox;     % 1.4  - heavy carry-through frames
kAB = obj.kseg_aftBarrel;   % 0.9  - aft barrel, slightly lighter
kTC = obj.kseg_tailcone;    % 0.7  - tapered, lightly loaded

% Raw segment masses before renormalisation
m_nose_Al     = kN  * w0 * L_nose;
m_cpit_Al     = kC  * w0 * L_cpit;
m_fwdBar_Al   = kFB * w0 * L_fwdBar;
m_wbox_Al     = kWB * w0 * L_wbox;
m_aftBar_Al   = kAB * w0 * L_aftBar;
m_tailcone_Al = kTC * w0 * L_tailcone;

% Renormalise so segments sum exactly to Raymer baseline
m_sum = m_nose_Al + m_cpit_Al + m_fwdBar_Al + m_wbox_Al + m_aftBar_Al + m_tailcone_Al;
scale = m_fus_Al / m_sum;

m_nose_Al     = m_nose_Al     * scale;
m_cpit_Al     = m_cpit_Al     * scale;
m_fwdBar_Al   = m_fwdBar_Al   * scale;
m_wbox_Al     = m_wbox_Al     * scale;
m_aftBar_Al   = m_aftBar_Al   * scale;
m_tailcone_Al = m_tailcone_Al * scale;


%% =====================================================================
%  5.  SKIN / FRAME SPLIT AND CFRP WEIGHT CORRECTION
% ======================================================================
% For each segment the corrected mass is:
%
%   m_corr = m_Al * [ rs*((1 - fC_s) + Ks*fC_s)
%                   + rf*((1 - fC_f) + Kf*fC_f) ]
%
%   rs, rf  = skin and frame mass fractions (0.45 / 0.55)
%   fC_s    = CFRP fraction within skins for this segment   (from ADP_BW)
%   fC_f    = CFRP fraction within frames for this segment  (from ADP_BW)
%   Ks      = Kmat for CFRP skins  (0.78 -> 22% mass saving vs Al)
%   Kf      = Kmat for CFRP frames (0.85 -> 15% mass saving vs Al)

rs = obj.fus_ratio_skin;    % 0.45
rf = obj.fus_ratio_frames;  % 0.55
Ks = obj.Kmat_skin;         % 0.78
Kf = obj.Kmat_frame;        % 0.85

% Anonymous helper keeps the per-segment lines concise
corrFn = @(m_al, fCs, fCf) m_al * ( rs*((1-fCs) + Ks*fCs) + ...
                                     rf*((1-fCf) + Kf*fCf) );

% --- Nose (ogive, unpressurised) ---
% Skin 30% CFRP | Frame 20% CFRP
% Simple ogive skin; minimal frames -> modest composite adoption
m_nose_corr     = corrFn(m_nose_Al,     obj.fCFRP_skin_nose,     obj.fCFRP_frame_nose);

% --- Cockpit shell (pressurised, complex window & door cutouts) ---
% Skin 40% CFRP | Frame 25% CFRP
% Matches A350 / B787 cockpit skin practice (~40% CFRP)
m_cpit_corr     = corrFn(m_cpit_Al,     obj.fCFRP_skin_cockpit,  obj.fCFRP_frame_cockpit);

% --- Forward barrel (main cargo bay, uniform pressure vessel) ---
% Skin 60% CFRP | Frame 35% CFRP
% Long uniform barrel suits CFRP panel manufacture (B777X barrel ~55-65%)
m_fwdBar_corr   = corrFn(m_fwdBar_Al,   obj.fCFRP_skin_barrel,   obj.fCFRP_frame_barrel);

% --- Wing-box bay (carry-through frames, heavy local reinforcement) ---
% Skin 60% CFRP | Frame 35% CFRP
% Wing carry-through frames are thick Al or Ti; only skin panels switch to CFRP
m_wbox_corr     = corrFn(m_wbox_Al,     obj.fCFRP_skin_wingbox,  obj.fCFRP_frame_wingbox);

% --- Aft barrel (mirror of forward barrel) ---
% Skin 60% CFRP | Frame 35% CFRP
m_aftBar_corr   = corrFn(m_aftBar_Al,   obj.fCFRP_skin_aft,      obj.fCFRP_frame_aft);

% --- Tailcone (tapered, lightly loaded) ---
% Skin 70% CFRP | Frame 30% CFRP
% Thin tapered shell is an ideal CFRP application; few frames
m_tailcone_corr = corrFn(m_tailcone_Al, obj.fCFRP_skin_tailcone, obj.fCFRP_frame_tailcone);

% Total corrected fuselage structural mass
m_fus_corr = m_nose_corr + m_cpit_corr + m_fwdBar_corr + ...
             m_wbox_corr + m_aftBar_corr + m_tailcone_corr;

% Diagnostic: print mass reduction factor (expected range 0.90 - 0.96)
FusMassReduction = m_fus_corr / m_fus_Al;
fprintf('  [fuselage] Al baseline: %.0f kg | CFRP-corrected: %.0f kg | reduction factor: %.3f\n', ...
        m_fus_Al, m_fus_corr, FusMassReduction);


%% =====================================================================
%  6.  NOSE CARGO-DOOR STRUCTURAL PENALTY
% ======================================================================
% A freighter nose door (hinge reinforcements, door skin, actuators,
% latches) adds approximately 1.5-3% of corrected fuselage mass.
% k_door = 0.02 (2%) is set in ADP_BW.

m_door = obj.k_door * m_fus_corr;


%% =====================================================================
%  7.  REPRESENTATIVE CG X-LOCATIONS FOR EACH SEGMENT
% ======================================================================

x_nose     = 0.20 * L_nose;
x_cpit     = L_nose + 0.50 * L_cpit;
x_fwdBar   = L_cockpit + 0.50 * L_fwdBar;
x_wbox     = obj.WingPos;                              % centre of wing carry-through bay
x_aftBar   = L_cockpit + L_fwdBar + L_wbox + 0.50 * L_aftBar;
x_tailcone = L_f - 0.30 * L_tailcone;
x_door     = L_nose + 0.50 * L_cpit;                  % door hinge at cockpit / nose junction


%% =====================================================================
%  8.  ASSEMBLE MASS OBJECTS
% ======================================================================

massObj        = cast.MassObj(Name="Fuselage Nose",       m=m_nose_corr,     X=[x_nose;     0]);
massObj(end+1) = cast.MassObj(Name="Fuselage Cockpit",    m=m_cpit_corr,     X=[x_cpit;     0]);
massObj(end+1) = cast.MassObj(Name="Fuselage Fwd Barrel", m=m_fwdBar_corr,   X=[x_fwdBar;   0]);
massObj(end+1) = cast.MassObj(Name="Fuselage WingBox",    m=m_wbox_corr,     X=[x_wbox;     0]);
massObj(end+1) = cast.MassObj(Name="Fuselage Aft Barrel", m=m_aftBar_corr,   X=[x_aftBar;   0]);
massObj(end+1) = cast.MassObj(Name="Fuselage Tailcone",   m=m_tailcone_corr, X=[x_tailcone; 0]);
massObj(end+1) = cast.MassObj(Name="Nose Cargo Door",     m=m_door,          X=[x_door;     0]);


%% =====================================================================
%  9.  SYSTEMS MASS  (unchanged from original formulation)
% ======================================================================

m_sys = (270*(2*obj.CabinRadius) + 150) * L_f / 9.81 * 2;
massObj(end+1) = cast.MassObj(Name="Systems", m=m_sys, X=[L_f/2; 0]);


%% =====================================================================
%  10. FUEL SYSTEM MASS  (Torenbeek, unchanged from original)
% ======================================================================

FuelMass   = obj.MTOM * obj.Mf_Fuel;
N_fuelTank = 3;    % two wing tanks + one centre tank
V_t        = FuelMass / cast.eng.Fuel.JA1.Density * SI.litre;  % total volume [litres]
N_eng      = 2;

m_fuelsys = 36.3*(N_eng + N_fuelTank - 1) + 4.366*N_fuelTank^0.5 * V_t^(1/3);  % [kg]
massObj(end+1) = cast.MassObj(Name="Fuel Systems", m=m_fuelsys, X=[L_f/2; 0]);

end
