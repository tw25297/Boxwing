clc; clear; close all;


%%  INPUTS
scripts.ExampleSizing_BW;





Span_max = ADP.Span_max;      % [m]
MTOM     = ADP.MTOM;    % [kg]

% Material / stiffness assumptions: Using Aluminium
% Replace these with your own values later
E_front = 70e9;           % [Pa]
E_rear  = 70e9;           % [Pa]

I_front = 0.08;           % [m^4] placeholder
I_rear  = 0.08;           % [m^4] placeholder



%% ----- box wing geometry assumptions -----
b = Span_max;                         % full span
S = (MTOM * 9.81) / ADP.WingLoading;  % total wing reference area


alpha  = ADP.alphaArea;   % Fraction of area for front wing (often = eta initially)


% ----- Split areas (still using total S) -----
Sf = alpha * S;         % Area of front wing [m^2]
Sr = (1 - alpha) * S;   % Area of rear wing [m^2]

bf = b;        % front full span
br = b;        % rear full span

tr = 0.3;

c_rf = 2 * Sf / (bf * (1 + tr));
c_rr = 2 * Sr / (br * (1 + tr));

c_tf = tr * c_rf;
c_tr = tr * c_rr;


% Lift split between front and rear wings
% Must sum to 1.0 for the half-aircraft lift
lift_split_front = ADP.etaLift;
lift_split_rear  = 1 - ADP.etaLift;





%% gust analysis stuff:
rho0 = 1.225;
Kg   = 0.75;
a_w = 5.5;   % [1/rad] temporary estimate for whole-wing lift curve slope !! double check with mond
WS   = ADP.WingLoading;

V_C = 350 * 0.514444;
V_D = 385 * 0.514444;

Ude_C = 16;
Ude_D = 8;

dn_C = rho0 * V_C * a_w * Kg * Ude_C / (2 * WS);
dn_D = rho0 * V_D * a_w * Kg * Ude_D / (2 * WS);

load_cases = struct( ...
    'name', {'1g','2p5g','minus1g','gustVCup','gustVCdown','gustVDup','gustVDdown'}, ...
    'n',    {1.0, 2.5, -1.0, 1+dn_C, 1-dn_C, 1+dn_D, 1-dn_D} );

%%
% Wing-box spar layout assumptions
x_fspar = 0.15;                         % front spar location as fraction of chord
x_rspar = 0.70;                         % rear spar location as fraction of chord
box_frac = x_rspar - x_fspar;           % = 0.55

% Aerodynamic lift line and structural torsion-axis assumptions
x_torsion_ax = 0.5 * (x_fspar + x_rspar); % = 0.425
x_lift_line = 0.25;                     % quarter-chord lift line


%%

results = struct([]);
 %material properties for Aluminium 7050-T7451
sigma_allow_cap = 469e6; %tensile yield strength is more defensible for caps/flange sizing than ultimate because you don't want the section reaching permanent deformation in normal loading
tau_allow_web   = 303e6; %shear strength for the web and skin as they are being sized from shera flow/ shear stress
rho_al          = 2800;

for k = 1:numel(load_cases)

    n_case = load_cases(k).n;

    front = B777.beamproperties(ADP, 'Front Wing', lift_split_front, E_front, I_front);
    rear  = B777.beamproperties(ADP, 'Rear Wing',  lift_split_rear,  E_rear,  I_rear);

    front = front.calcTriangularLoad(n_case);
    rear  = rear.calcTriangularLoad(n_case);

    % tip compatibility
    delta_front_0 = front.tipDeflectionTriangular();
    delta_rear_0  = rear.tipDeflectionTriangular();

    cf = front.beamlength^3 / (3 * front.E * front.I);
    cr = rear.beamlength^3  / (3 * rear.E  * rear.I);

    Rt = (delta_front_0 - delta_rear_0) / (cf + cr);

    front = front.setTipForce(Rt, -1);
    rear  = rear.setTipForce(Rt, +1);

    front = front.reactionLoads();
    rear  = rear.reactionLoads();

    % diagrams
    npts = 200;
    [xf, Vf, Mf] = front.diagrams(npts);
    [xr, Vr, Mr] = rear.diagrams(npts);

    % local geometry
    c_local_front = c_rf - ((c_rf - c_tf)/front.beamlength) .* xf;
    c_local_rear  = c_rr - ((c_rr - c_tr)/rear.beamlength)  .* xr;

    wing_box_width_front  = box_frac    .* c_local_front; % 15% L.E. and then 35% T.E. :. 
    wing_box_width_rear   = box_frac    .* c_local_rear;

    wing_box_height_front = 0.1086 .* c_local_front;
    wing_box_height_rear  = 0.1086 .* c_local_rear;

    % caps
    [A_cap_front, ~] = B777.structures_calculations(ADP, abs(Mf), wing_box_height_front, wing_box_width_front, sigma_allow_cap);
    [A_cap_rear,  ~] = B777.structures_calculations(ADP, abs(Mr), wing_box_height_rear,  wing_box_width_rear,  sigma_allow_cap);

    V_caps_half_front = trapz(xf, 2 .* A_cap_front);
    V_caps_half_rear  = trapz(xr, 2 .* A_cap_rear);

    % webs
    t_web_front = abs(Vf) ./ (2 .* tau_allow_web .* wing_box_height_front);
    t_web_rear  = abs(Vr) ./ (2 .* tau_allow_web .* wing_box_height_rear);

    A_web_total_front = 2 .* t_web_front .* wing_box_height_front;
    A_web_total_rear  = 2 .* t_web_rear  .* wing_box_height_rear;

    V_webs_half_front = trapz(xf, A_web_total_front);
    V_webs_half_rear  = trapz(xr, A_web_total_rear);

    mass_caps_total = 2 * rho_al * (V_caps_half_front + V_caps_half_rear);
    mass_webs_total = 2 * rho_al * (V_webs_half_front + V_webs_half_rear);

    results(k).name = load_cases(k).name;
    results(k).n    = n_case;
    results(k).Rt   = Rt;
    results(k).maxM = max([abs(Mf), abs(Mr)]);
    results(k).maxV = max([abs(Vf), abs(Vr)]);
    results(k).mass_primary = mass_caps_total + mass_webs_total;
end

%%  PRINT RESULTS

fprintf('\n============================================================\n');
fprintf('TIP COMPATIBILITY SOLUTION\n');
fprintf('============================================================\n');
fprintf('Rt magnitude = %.3f MN\n', Rt / 1e6);
fprintf('Front wing triangular-only tip deflection = %.4f m\n', delta_front_0);
fprintf('Rear  wing triangular-only tip deflection = %.4f m\n', delta_rear_0);
fprintf('Front wing final tip deflection           = %.4f m\n', front.tipDeflectionTotal());
fprintf('Rear  wing final tip deflection           = %.4f m\n', rear.tipDeflectionTotal());

front.printSummary();
rear.printSummary();
%% 

[~, idx] = max([results.mass_primary]);

n_gov    = results(idx).n;
name_gov = results(idx).name;

fprintf('\nGoverning load case = %s\n', name_gov);
fprintf('Load factor n       = %.3f\n', n_gov);
fprintf('Max moment          = %.3f MNm\n', results(idx).maxM / 1e6);
fprintf('Max shear           = %.3f MN\n',  results(idx).maxV / 1e6);
fprintf('Primary mass        = %.1f kg\n',  results(idx).mass_primary);

% -------------------------------------------------
% RERUN GOVERNING CASE FOR CLEAN OUTPUT / PLOTS
% -------------------------------------------------
front = B777.beamproperties(ADP, 'Front Wing', lift_split_front, E_front, I_front);
rear  = B777.beamproperties(ADP, 'Rear Wing',  lift_split_rear,  E_rear,  I_rear);

front = front.calcTriangularLoad(n_gov);
rear  = rear.calcTriangularLoad(n_gov);

delta_front_0 = front.tipDeflectionTriangular();
delta_rear_0  = rear.tipDeflectionTriangular();

cf = front.beamlength^3 / (3 * front.E * front.I);
cr = rear.beamlength^3  / (3 * rear.E  * rear.I);

Rt = (delta_front_0 - delta_rear_0) / (cf + cr);

front = front.setTipForce(Rt, -1);
rear  = rear.setTipForce(Rt, +1);

front = front.reactionLoads();
rear  = rear.reactionLoads();

%% ---------- ASSUMED INTERNAL LAYOUT (PRELIMINARY) ----------

%% ---------- PRELIMINARY RIB MASS MODEL ----------

% -------------------------------------------------------------------------
% USER INPUTS
% -------------------------------------------------------------------------
s_rib_front = 0.50;      % [m] assumed front rib spacing
s_rib_rear  = 0.50;      % [m] assumed rear rib spacing

t_rib = 0.003;           % [m] rib thickness
%rho_al was here before

% Elliptical lightening-hole geometry factors
alpha_hole = 0.70;       % major diameter fraction of local wing-box width
beta_hole  = 0.60;       % minor diameter fraction of local wing-box height

% -------------------------------------------------------------------------
% CALCULATE NUMBER OF RIBS FROM SPACING
% -------------------------------------------------------------------------
N_ribs_front_half = ceil(front.beamlength / s_rib_front) + 1;
N_ribs_rear_half  = ceil(rear.beamlength  / s_rib_rear ) + 1;

% rib station locations
x_rib_front = linspace(0, front.beamlength, N_ribs_front_half);
x_rib_rear  = linspace(0, rear.beamlength,  N_ribs_rear_half);

% -------------------------------------------------------------------------
% LOCAL RIB GEOMETRY
% -------------------------------------------------------------------------
c_rib_front = c_rf - ((c_rf - c_tf)/front.beamlength) .* x_rib_front;
c_rib_rear  = c_rr - ((c_rr - c_tr)/rear.beamlength)  .* x_rib_rear;

b_box_rib_front = box_frac  .* c_rib_front; %0.5 because the wing box bredth should cover the sama area as the wingbox area
b_box_rib_rear  = box_frac  .* c_rib_rear;

h_box_rib_front = 0.1086 .* c_rib_front;
h_box_rib_rear  = 0.1086 .* c_rib_rear;

% -------------------------------------------------------------------------
% GROSS RIB AREA
% -------------------------------------------------------------------------
A_rib_gross_front = b_box_rib_front .* h_box_rib_front;
A_rib_gross_rear  = b_box_rib_rear  .* h_box_rib_rear;

% -------------------------------------------------------------------------
% ELLIPTICAL LIGHTENING HOLE
% -------------------------------------------------------------------------
D_hole_front = alpha_hole .* b_box_rib_front;   % major diameter
D_hole_rear  = alpha_hole .* b_box_rib_rear;

H_hole_front = beta_hole  .* h_box_rib_front;   % minor diameter
H_hole_rear  = beta_hole  .* h_box_rib_rear;

A_hole_front = (pi/4) .* D_hole_front .* H_hole_front;
A_hole_rear  = (pi/4) .* D_hole_rear  .* H_hole_rear;

% Remaining rib material area
A_rib_mat_front = A_rib_gross_front - A_hole_front;
A_rib_mat_rear  = A_rib_gross_rear  - A_hole_rear;

% Optional retained fractions for reporting
phi_rib_front = A_rib_mat_front ./ A_rib_gross_front;
phi_rib_rear  = A_rib_mat_rear  ./ A_rib_gross_rear;

% -------------------------------------------------------------------------
% RIB VOLUME AND MASS
% -------------------------------------------------------------------------
V_ribs_half_front = sum(A_rib_mat_front .* t_rib);
V_ribs_half_rear  = sum(A_rib_mat_rear  .* t_rib);

mass_ribs_total = 2 * rho_al * (V_ribs_half_front + V_ribs_half_rear);

% -------------------------------------------------------------------------
% PRINT RESULTS
% -------------------------------------------------------------------------
fprintf('\n================ PRELIMINARY RIB MASS MODEL ================\n');
fprintf('Front rib spacing                = %.3f m\n', s_rib_front);
fprintf('Rear  rib spacing                = %.3f m\n', s_rib_rear);
fprintf('Front ribs / half-wing           = %d\n', N_ribs_front_half);
fprintf('Rear  ribs / half-wing           = %d\n', N_ribs_rear_half);
fprintf('Front mean retained fraction     = %.3f\n', mean(phi_rib_front));
fprintf('Rear  mean retained fraction     = %.3f\n', mean(phi_rib_rear));
fprintf('Estimated total rib mass         = %.1f kg\n', mass_ribs_total);

%% ============================================================
% TORQUE DISTRIBUTION FROM LIFT OFFSET ABOUT TORSION AXIS
% ============================================================

npts_torque = 200;
[xf, ~, ~] = front.diagrams(npts_torque);
[xr, ~, ~] = rear.diagrams(npts_torque);

% Force column vectors
xf = xf(:);
xr = xr(:);

% Rebuild local chord distributions at governing-case stations
c_local_front = c_rf - ((c_rf - c_tf) / front.beamlength) .* xf;
c_local_rear  = c_rr - ((c_rr - c_tr) / rear.beamlength)  .* xr;




% Chordwise eccentricity between lift line and torsion axis
e_front = (x_torsion_ax - x_lift_line) .* c_local_front;   % [m]
e_rear  = (x_torsion_ax - x_lift_line) .* c_local_rear;    % [m]

% Distributed lift intensity at each span station from existing beam model
w_front = arrayfun(@(xx) front.loadAt(xx), xf);
w_rear  = arrayfun(@(xx) rear.loadAt(xx),  xr);

w_front = w_front(:);
w_rear  = w_rear(:);

% Distributed twisting moment per unit span
m_t_front = w_front .* e_front;   % [N]
m_t_rear  = w_rear  .* e_rear;    % [N]

% Torque from cut to tip using cumulative integration on flipped arrays
T_front = flipud(cumtrapz(flipud(xf), flipud(m_t_front)));
T_rear  = flipud(cumtrapz(flipud(xr), flipud(m_t_rear)));

%% TORQUE DIAGRAMS

% figure('Name','Front Wing Torque');
% plot(xf, T_front/1e6, 'LineWidth', 1.8)
% grid on
% xlabel('x [m]')
% ylabel('Torque [MN m]')
% title('Front Wing Torque Distribution')
% 
% figure('Name','Rear Wing Torque');
% plot(xr, T_rear/1e6, 'LineWidth', 1.8)
% grid on
% xlabel('x [m]')
% ylabel('Torque [MN m]')
% title('Rear Wing Torque Distribution')

%% ============================================================
% SKIN SIZING FROM TORQUE
% ============================================================

tau_allow_skin = 303e6;   % [Pa] shear strength basis for 7050-T7451

% Use the same governing-case box geometry arrays on the torque grid
wing_box_width_front_torque  = box_frac .* c_local_front;
wing_box_width_rear_torque   = box_frac .* c_local_rear;

wing_box_height_front_torque = 0.1086 .* c_local_front;
wing_box_height_rear_torque  = 0.1086 .* c_local_rear;

% Enclosed box area (spanwise array, not a single scalar)
A_m_front = wing_box_width_front_torque .* wing_box_height_front_torque;
A_m_rear  = wing_box_width_rear_torque  .* wing_box_height_rear_torque;

% Shear flow from closed-section torsion: 2*A*q = T
q_front = abs(T_front) ./ (2 .* A_m_front);
q_rear  = abs(T_rear)  ./ (2 .* A_m_rear);

% Skin thickness from shear flow
t_skin_front = q_front ./ tau_allow_skin;
t_skin_rear  = q_rear  ./ tau_allow_skin;

% Approximate box perimeter
P_front = 2 .* wing_box_width_front_torque + 2 .* wing_box_height_front_torque;
P_rear  = 2 .* wing_box_width_rear_torque  + 2 .* wing_box_height_rear_torque;

% Skin material area per span station
A_skin_front = P_front .* t_skin_front;
A_skin_rear  = P_rear  .* t_skin_rear;

% Half-wing skin volumes
V_skin_half_front = trapz(xf, A_skin_front);
V_skin_half_rear  = trapz(xr, A_skin_rear);

% Total skin mass
mass_skin_total = 2 * rho_al * (V_skin_half_front + V_skin_half_rear);

fprintf('\n================ SKIN SIZING FROM TORQUE ================\n');
fprintf('Front root skin thickness         = %.3f mm\n', 1e3*t_skin_front(1));
fprintf('Rear  root skin thickness         = %.3f mm\n', 1e3*t_skin_rear(1));
fprintf('Estimated total skin mass         = %.1f kg\n', mass_skin_total);

figure('Name','Skin thickness from torsion');
plot(xf, 1e3*t_skin_front, 'LineWidth', 1.8); hold on
plot(xr, 1e3*t_skin_rear,  'LineWidth', 1.8);
grid on
xlabel('x [m]')
ylabel('Skin thickness [mm]')
legend('Front wing','Rear wing','Location','best')
title('Skin thickness distribution from torsion')





%% % --------- FUEL TANK / GEAR BAY EXCLUSION ---------
rho_fuel   = 800;    % kg/m^3, placeholder % convert into kg/l
eta_tank   = 1;   % tank usability factor
eta_gearbay = 0.5;  % how much of excluded root-box volume is really unusable (changed to 0.5 from 0.8 to increase fuel volume)
x_gear_end = 0.08 * front.beamlength;   % first 12% (changed to 8% to increase fuel volume) of semi-span excluded

A_box_front = wing_box_width_front .* wing_box_height_front; %
A_box_rear  = wing_box_width_rear  .* wing_box_height_rear;

%don't need to halve area here because xf and xr are already half-span
%arrays, so integrating box area along them arelady gives you the half-wing
%box volume

V_box_half_front = trapz(xf, A_box_front);
V_box_half_rear  = trapz(xr, A_box_rear);

idx_gear = xf <= x_gear_end;
V_gearbay_half_front = eta_gearbay * trapz(xf(idx_gear), A_box_front(idx_gear));

V_free_half_front = V_box_half_front ...
                  - V_caps_half_front ...
                  - V_webs_half_front ...
                  - V_ribs_half_front ...
                  - V_skin_half_front ...
                  - V_gearbay_half_front;

V_free_half_rear = V_box_half_rear ...
                 - V_caps_half_rear ...
                 - V_webs_half_rear ...
                 - V_ribs_half_rear ...
                 - V_skin_half_rear;

V_fuel_total = 2* eta_tank * (V_free_half_front + V_free_half_rear);

m_fuel_capacity = rho_fuel * V_fuel_total;

fprintf('\nUsable fuel volume = %.2f m^3\n', V_fuel_total);
fprintf('Fuel mass capacity = %.1f kg\n', m_fuel_capacity);



%% 


%% ============================================================
% TORNBEEK SECONDARY STRUCTURE MASS
% Fixed Leading Edge (Eq. 11.63)
% Fixed Trailing Edge (Eq. 11.65)
% ============================================================

g = 9.81;

% Save area variables clearly so they are not confused later with shear S
S_front_area = Sf;     % front wing reference area [m^2]
S_rear_area  = Sr;     % rear wing reference area  [m^2]

% ------------------------------------------------------------
% Torenbeek reference constants
% ------------------------------------------------------------
Omega_ref = 56;      % [N/m^2] from Torenbeek Sec. 11.6
q_ref     = 30e3;    % [N/m^2] = 30 kN/m^2
W_ref     = 1e6;     % [N]
b_ref     = 50;      % [m]

% ------------------------------------------------------------
% Aircraft values already available in your script
% ------------------------------------------------------------
W_MTO = MTOM * g;              % [N]
q_D   = 0.5 * rho0 * V_D^2;    % [N/m^2]

% ------------------------------------------------------------
% User assumptions
% ------------------------------------------------------------

k_fle   = 1.0;     % no slats / Krueger flaps

% Fixed trailing-edge strip fractions
cfixTE_front = 0.08;   % fixed TE strip = 8% of local front-wing chord
cfixTE_rear  = 0.08;   % fixed TE strip = 8% of local rear-wing chord

% Extra specific-weight penalty for flap-support structure:
% 0   -> no extra penalty
% 40  -> supports single-slotted Fowler / double-slotted flaps
% 100 -> supports double-slotted Fowler / triple-slotted flaps
dOmega_fte_support = 40;   % [N/m^2]

% ------------------------------------------------------------
% Rebuild governing-case chord distributions so this block is self-contained
% ------------------------------------------------------------
npts_sec = 300;
[xf_sec, ~, ~] = front.diagrams(npts_sec);
[xr_sec, ~, ~] = rear.diagrams(npts_sec);

c_local_front_sec = c_rf - ((c_rf - c_tf) / front.beamlength) .* xf_sec;
c_local_rear_sec  = c_rr - ((c_rr - c_tr) / rear.beamlength)  .* xr_sec;

% ============================================================
% 1) FIXED LEADING EDGE
% ============================================================
% Approximate planform area as area ahead of front spar
S_fle_front = x_fspar * S_front_area;   % [m^2]
S_fle_rear  = x_fspar * S_rear_area;    % [m^2]

Omega_fle_front = 3.15 * k_fle * Omega_ref ...
                * (q_D / q_ref)^0.25 ...
                * ((W_MTO * bf) / (W_ref * b_ref))^0.145;    % [N/m^2]

Omega_fle_rear  = 3.15 * k_fle * Omega_ref ...
                * (q_D / q_ref)^0.25 ...
                * ((W_MTO * br) / (W_ref * b_ref))^0.145;    % [N/m^2]

W_fle_front = Omega_fle_front * S_fle_front;   % [N]
W_fle_rear  = Omega_fle_rear  * S_fle_rear;    % [N]

mass_fle_total = (W_fle_front + W_fle_rear) / g;   % [kg]

% ============================================================
% 2) FIXED TRAILING EDGE
% ============================================================
% Estimate planform area as a fixed strip aft of the rear spar
S_fte_half_front = trapz(xf_sec, cfixTE_front .* c_local_front_sec);   % [m^2]
S_fte_half_rear  = trapz(xr_sec, cfixTE_rear  .* c_local_rear_sec);    % [m^2]

S_fte_front = 2 * S_fte_half_front;   % [m^2]
S_fte_rear  = 2 * S_fte_half_rear;    % [m^2]

Omega_fte_front = 2.6 * Omega_ref ...
                * ((W_MTO * bf) / (W_ref * b_ref))^0.0544 ...
                + dOmega_fte_support;                                  % [N/m^2]

Omega_fte_rear  = 2.6 * Omega_ref ...
                * ((W_MTO * br) / (W_ref * b_ref))^0.0544 ...
                + dOmega_fte_support;                                  % [N/m^2]

W_fte_front = Omega_fte_front * S_fte_front;   % [N]
W_fte_rear  = Omega_fte_rear  * S_fte_rear;    % [N]

mass_fte_total = (W_fte_front + W_fte_rear) / g;   % [kg]

% ============================================================
% TOTAL FROM THESE TWO TORNBEEK TERMS
% ============================================================
mass_secondary_fixed_total = mass_fle_total + mass_fte_total;

fprintf('\n================ TORNBEEK SECONDARY STRUCTURES ================\n');
fprintf('q_D                                = %.2f kPa\n', q_D / 1000);
fprintf('Omega_ref                          = %.1f N/m^2\n', Omega_ref);

fprintf('\n--- Fixed Leading Edge ---\n');
fprintf('S_fle_front                        = %.2f m^2\n', S_fle_front);
fprintf('S_fle_rear                         = %.2f m^2\n', S_fle_rear);
fprintf('Omega_fle_front                    = %.2f N/m^2\n', Omega_fle_front);
fprintf('Omega_fle_rear                     = %.2f N/m^2\n', Omega_fle_rear);
fprintf('Fixed LE mass total                = %.1f kg\n', mass_fle_total);

fprintf('\n--- Fixed Trailing Edge ---\n');
fprintf('S_fte_front                        = %.2f m^2\n', S_fte_front);
fprintf('S_fte_rear                         = %.2f m^2\n', S_fte_rear);
fprintf('Omega_fte_front                    = %.2f N/m^2\n', Omega_fte_front);
fprintf('Omega_fte_rear                     = %.2f N/m^2\n', Omega_fte_rear);
fprintf('Fixed TE mass total                = %.1f kg\n', mass_fte_total);

fprintf('\nTotal fixed LE + fixed TE mass     = %.1f kg\n', mass_secondary_fixed_total);

% Optional running total
mass_structural_total_with_secondary = results(idx).mass_primary ...
                                    + mass_ribs_total ...
                                    + mass_skin_total ...
                                    + mass_secondary_fixed_total;
fprintf('Primary + ribs + skin + fixed LE/TE total = %.1f kg\n', mass_structural_total_with_secondary);


%% ============================================================
% TORNBEEK MOVABLE SURFACES
% Conventional-equivalent total device area
% then split between front and rear wings
% ============================================================
% ------------------------------------------------------------
% Torenbeek reference constants
% ------------------------------------------------------------
Omega_ref = 56;      % [N/m^2]
S_ref     = 10;      % [m^2]
W_ref     = 1e6;     % [N]

W_MTO = MTOM * g;    % [N]

% ------------------------------------------------------------
% TOTAL CONVENTIONAL-EQUIVALENT DEVICE AREA FRACTIONS
% These are fractions of TOTAL wing area S, not front/rear separately
% ------------------------------------------------------------
flap_frac_total = 0.16;   % 0.16 to 0.20 is a sensible first pass
ail_frac_total  = 0.04;   % ailerons about 3% to 5% of wing area
sp_frac_total   = 0.04;   % spoilers/lift dumpers about 4% for large jetliners

% ------------------------------------------------------------
% SPLIT OF TOTAL DEVICE AREA BETWEEN FRONT AND REAR WINGS
% Must add to 1.0 for each device type
% ------------------------------------------------------------
flap_share_front = 0.60;
flap_share_rear  = 0.40;

ail_share_front  = 0.50;
ail_share_rear   = 0.50;

sp_share_front   = 0.50;
sp_share_rear    = 0.50;

% ------------------------------------------------------------
% CHECK SHARES
% ------------------------------------------------------------
if abs((flap_share_front + flap_share_rear) - 1.0) > 1e-8
    error('Flap shares must sum to 1.0');
end
if abs((ail_share_front + ail_share_rear) - 1.0) > 1e-8
    error('Aileron shares must sum to 1.0');
end
if abs((sp_share_front + sp_share_rear) - 1.0) > 1e-8
    error('Spoiler shares must sum to 1.0');
end

% ------------------------------------------------------------
% FLAP / AILERON / SPOILER SUPPORT ASSUMPTIONS
% ------------------------------------------------------------
% Flaps
k_sup_front  = 1.2;   % 1.0 simple hinge, 1.2 link/track end supports, 1.6 Fowler hooked track
k_sup_rear   = 1.2;

k_slot_front = 1.0;   % 1.0 single-slotted, 1.5 double-slotted fixed vanes, 2.0 double-slotted articulating vanes
k_slot_rear  = 1.0;

% Ailerons
k_bal_front = 1.0;    % 1.0 unbalanced, 1.3 aerodynamic-balanced, 1.54 mass-balanced
k_bal_rear  = 1.0;

% ------------------------------------------------------------
% TOTAL DEVICE AREAS BASED ON TOTAL WING AREA
% ------------------------------------------------------------
S_tef_total = flap_frac_total * S;   % [m^2]
S_ail_total = ail_frac_total  * S;   % [m^2]
S_sp_total  = sp_frac_total   * S;   % [m^2]

% Split between front and rear wings
S_tef_front = flap_share_front * S_tef_total;
S_tef_rear  = flap_share_rear  * S_tef_total;

S_ail_front = ail_share_front * S_ail_total;
S_ail_rear  = ail_share_rear  * S_ail_total;

S_sp_front  = sp_share_front * S_sp_total;
S_sp_rear   = sp_share_rear  * S_sp_total;

% Optional safety check against available wing area
if S_tef_front > Sf
    warning('Front flap area exceeds front wing area');
end
if S_tef_rear > Sr
    warning('Rear flap area exceeds rear wing area');
end
if S_ail_front > Sf
    warning('Front aileron area exceeds front wing area');
end
if S_ail_rear > Sr
    warning('Rear aileron area exceeds rear wing area');
end
if S_sp_front > Sf
    warning('Front spoiler area exceeds front wing area');
end
if S_sp_rear > Sr
    warning('Rear spoiler area exceeds rear wing area');
end

% ============================================================
% 1) TRAILING-EDGE FLAPS (Eq. 11.66)
% ============================================================
Omega_tef_front = 1.7 * k_sup_front * k_slot_front * Omega_ref ...
                * (1 + (W_MTO / W_ref)^0.35);     % [N/m^2]

Omega_tef_rear  = 1.7 * k_sup_rear  * k_slot_rear  * Omega_ref ...
                * (1 + (W_MTO / W_ref)^0.35);     % [N/m^2]

W_tef_front = Omega_tef_front * S_tef_front;      % [N]
W_tef_rear  = Omega_tef_rear  * S_tef_rear;       % [N]

mass_tef_total = (W_tef_front + W_tef_rear) / g;  % [kg]

% ============================================================
% 2) AILERONS (Eq. 11.67)
% ============================================================
Omega_ail_front = 3.0 * Omega_ref * k_bal_front * (max(S_ail_front,1e-9)/S_ref)^0.044;   % [N/m^2]
Omega_ail_rear  = 3.0 * Omega_ref * k_bal_rear  * (max(S_ail_rear ,1e-9)/S_ref)^0.044;   % [N/m^2]

W_ail_front = Omega_ail_front * S_ail_front;      % [N]
W_ail_rear  = Omega_ail_rear  * S_ail_rear;       % [N]

mass_ail_total = (W_ail_front + W_ail_rear) / g;  % [kg]

% ============================================================
% 3) SPOILERS / LIFT DUMPERS (Eq. 11.68)
% ============================================================
Omega_sp_front = 2.2 * Omega_ref * (max(S_sp_front,1e-9)/S_ref)^0.032;   % [N/m^2]
Omega_sp_rear  = 2.2 * Omega_ref * (max(S_sp_rear ,1e-9)/S_ref)^0.032;   % [N/m^2]

W_sp_front = Omega_sp_front * S_sp_front;         % [N]
W_sp_rear  = Omega_sp_rear  * S_sp_rear;          % [N]

mass_sp_total = (W_sp_front + W_sp_rear) / g;     % [kg]

% ============================================================
% TOTAL MOVABLE-SURFACE MASS
% ============================================================
mass_movable_surfaces_total = mass_tef_total + mass_ail_total + mass_sp_total;

fprintf('\n================ TORNBEEK MOVABLE SURFACES ================\n');

fprintf('\n--- Area assumptions ---\n');
fprintf('Total flap area                  = %.2f m^2\n', S_tef_total);
fprintf('  Front flap area                = %.2f m^2\n', S_tef_front);
fprintf('  Rear flap area                 = %.2f m^2\n', S_tef_rear);

fprintf('Total aileron area               = %.2f m^2\n', S_ail_total);
fprintf('  Front aileron area             = %.2f m^2\n', S_ail_front);
fprintf('  Rear aileron area              = %.2f m^2\n', S_ail_rear);

fprintf('Total spoiler area               = %.2f m^2\n', S_sp_total);
fprintf('  Front spoiler area             = %.2f m^2\n', S_sp_front);
fprintf('  Rear spoiler area              = %.2f m^2\n', S_sp_rear);

fprintf('\n--- Mass results ---\n');
fprintf('Flap mass total                  = %.1f kg\n', mass_tef_total);
fprintf('Aileron mass total               = %.1f kg\n', mass_ail_total);
fprintf('Spoiler mass total               = %.1f kg\n', mass_sp_total);
fprintf('Total movable-surface mass       = %.1f kg\n', mass_movable_surfaces_total);

% ------------------------------------------------------------
% OPTIONAL UPDATED TOTAL
% ------------------------------------------------------------
mass_total_structural_plus_movables = results(idx).mass_primary ...
                                    + mass_ribs_total ...
                                    + mass_skin_total ...
                                    + mass_movable_surfaces_total;

if exist('mass_secondary_fixed_total','var')
    mass_total_structural_plus_movables = mass_total_structural_plus_movables + mass_secondary_fixed_total;
end
%%  PLOT SHEAR AND MOMENT DIAGRAMS
 %!!!!!! uncomment these diagrams
% npts = 300;
% [xf, Vf_plot, Mf_plot] = front.diagrams(npts);
% [xr, Vr_plot, Mr_plot] = rear.diagrams(npts);
% 
% 
% figure('Name','Front Wing Shear / Moment');
% subplot(2,1,1)
% plot(xf, Vf_plot/1e6, 'LineWidth', 1.8)
% grid on
% xlabel('x [m]')
% ylabel('Shear [MN]')
% title('Front Wing Shear Force')
% 
% subplot(2,1,2)
% plot(xf, Mf_plot/1e6, 'LineWidth', 1.8)
% grid on
% xlabel('x [m]')
% ylabel('Moment [MN m]')
% title('Front Wing Bending Moment')
% 
% figure('Name','Rear Wing Shear / Moment');
% subplot(2,1,1)
% plot(xr, Vr_plot/1e6, 'LineWidth', 1.8)
% grid on
% xlabel('x [m]')
% ylabel('Shear [MN]')
% title('Rear Wing Shear Force')
% 
% subplot(2,1,2)
% plot(xr, Mr_plot/1e6, 'LineWidth', 1.8)
% grid on
% xlabel('x [m]')
% ylabel('Moment [MN m]')
% title('Rear Wing Bending Moment')





fprintf('\nUpdated wing total incl. skin + fixed + movables = %.1f kg\n', ...
    mass_total_structural_plus_movables);