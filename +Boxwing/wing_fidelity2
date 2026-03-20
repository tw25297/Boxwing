clc; clear; close all;


%%  INPUTS
scripts.ExampleSizing_BW;


% ADP.Span_max = 64.8;      % [m]
% ADP.MTOM     = 300000;    % [kg]


Span_max = ADP.Span_max;      % [m]
MTOM     = ADP.MTOM;    % [kg]

% Material / stiffness assumptions: Using CFRP
% Replace these with your own values later
E_front = 70e9;           % [Pa]
E_rear  = 70e9;           % [Pa]

I_front = 0.08;           % [m^4] placeholder
I_rear  = 0.08;           % [m^4] placeholder

% Lift split between front and rear wings
% Must sum to 1.0 for the half-aircraft lift
lift_split_front = ADP.etaLift;
lift_split_rear  = 1 - ADP.etaLift;

% 3 Load factors for 3 different load cases
%n = 1; %for straight and level flight
%n = 2; %for
n = 2.5; %for 


%%  CREATE FRONT AND REAR BEAM OBJECTS


front = B777.beamproperties(ADP, 'Front Wing', lift_split_front, E_front, I_front);

rear  = B777.beamproperties(ADP, 'Rear Wing',  lift_split_rear,  E_rear,  I_rear);

front = front.calcTriangularLoad(n);
rear  = rear.calcTriangularLoad(n);


%%  SOLVE TIP COMPATIBILITY FOR Rt


% First: triangular-load-only tip deflections
delta_front_0 = front.tipDeflectionTriangular();
delta_rear_0  = rear.tipDeflectionTriangular();

% Flexibility coefficients due to unit tip force
cf = front.beamlength^3 / (3 * front.E * front.I);
    cr = rear.beamlength^3  / (3 * rear.E  * rear.I);

% Assume:
% - front wing gets a downward tip force  => Rt_sign = -1
% - rear wing gets an upward tip force    => Rt_sign = +1
%
% Then:
% delta_front = delta_front_0 - Rt*cf
% delta_rear  = delta_rear_0  + Rt*cr
%
% Compatibility:
% delta_front = delta_rear

Rt = (delta_front_0 - delta_rear_0) / (cf + cr);   % [N]

front = front.setTipForce(Rt, -1);
rear  = rear.setTipForce(Rt, +1);

front = front.reactionLoads();
rear  = rear.reactionLoads();



npts = 50;

xf = linspace(0, front.beamlength, npts);
Mf = arrayfun(@(x) front.momentAt(x), xf);

xr = linspace(0, rear.beamlength, npts);
Mr = arrayfun(@(x) rear.momentAt(x), xr);

disp(table(xf', Mf'/1e6, 'VariableNames', {'x_front_m','M_front_MNm'}))
disp(table(xr', Mr'/1e6, 'VariableNames', {'x_rear_m','M_rear_MNm'}))

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

%% 

%% ---------- sizing arrays ----------
npts = 200;
[xf, Vf, Mf] = front.diagrams(npts);
[xr, Vr, Mr] = rear.diagrams(npts);

%% ---------- local geometry ----------
c_local_front = c_rf - ((c_rf - c_tf)/front.beamlength) .* xf;
c_local_rear  = c_rr - ((c_rr - c_tr)/rear.beamlength)  .* xr;

wing_box_width_front  = 0.5    .* c_local_front;
wing_box_width_rear   = 0.5    .* c_local_rear;

wing_box_height_front = 0.1086 .* c_local_front;
wing_box_height_rear  = 0.1086 .* c_local_rear;

%% ---------- cap sizing ----------
b_cap_front = wing_box_width_front;
b_cap_rear  = wing_box_width_rear;

sigma_allow_cap = 503e6;   % example placeholder only
[A_cap_front, t_cap_front] = B777.structures_calculations(ADP, abs(Mf), wing_box_height_front, b_cap_front, sigma_allow_cap);
[A_cap_rear,  t_cap_rear ] = B777.structures_calculations(ADP, abs(Mr), wing_box_height_rear,  b_cap_rear,  sigma_allow_cap);

V_caps_half_front = trapz(xf, 2 .* A_cap_front);
V_caps_half_rear  = trapz(xr, 2 .* A_cap_rear);

%% ---------- web sizing ----------
tau_allow_web = 331e6;   % example placeholder only

t_web_front = abs(Vf) ./ (2 .* tau_allow_web .* wing_box_height_front);
t_web_rear  = abs(Vr) ./ (2 .* tau_allow_web .* wing_box_height_rear);

A_web_total_front = 2 .* t_web_front .* wing_box_height_front;
A_web_total_rear  = 2 .* t_web_rear  .* wing_box_height_rear;

V_webs_half_front = trapz(xf, A_web_total_front);
V_webs_half_rear  = trapz(xr, A_web_total_rear);

%% ---------- masses ----------
rho_al = 2800; %density 

mass_caps_total = 2 * rho_al * (V_caps_half_front + V_caps_half_rear);
mass_webs_total = 2 * rho_al * (V_webs_half_front + V_webs_half_rear);

mass_primary_wingbox_total = mass_caps_total + mass_webs_total;

fprintf('Total cap mass           = %.1f kg\n', mass_caps_total);
fprintf('Total web mass           = %.1f kg\n', mass_webs_total);
fprintf('Primary wing-box mass    = %.1f kg\n', mass_primary_wingbox_total);


%% ---------- BUCKLING-BASED LAYOUT (FIXED FIRST PASS) ----------

E_al = 71.7e9;
nu   = 0.33;
k_c  = 4.0;

Cplate = (pi^2 * E_al) / (12 * (1 - nu^2));

% --- minimum practical gauges ---
t_panel_min = 0.0025;   % 2.5 mm
t_web_min   = 0.0020;   % 2.0 mm

t_panel_front = max(t_cap_front, t_panel_min);
t_panel_rear  = max(t_cap_rear,  t_panel_min);

t_web_front_eff = max(t_web_front, t_web_min);
t_web_rear_eff  = max(t_web_rear,  t_web_min);

% --- only let genuinely loaded regions govern ---
valid_panel_front = abs(Mf) > 0.10 * max(abs(Mf));
valid_panel_rear  = abs(Mr) > 0.10 * max(abs(Mr));

valid_web_front = abs(Vf) > 0.10 * max(abs(Vf));
valid_web_rear  = abs(Vr) > 0.10 * max(abs(Vr));

%% ----- TOP/BOTTOM PANEL COMPRESSION BUCKLING -----
sigma_panel_front = abs(Mf) ./ max(A_cap_front .* wing_box_height_front, eps);
sigma_panel_rear  = abs(Mr) ./ max(A_cap_rear  .* wing_box_height_rear,  eps);

p_stringer_max_front = t_panel_front .* sqrt((k_c * Cplate) ./ max(sigma_panel_front, eps));
p_stringer_max_rear  = t_panel_rear  .* sqrt((k_c * Cplate) ./ max(sigma_panel_rear,  eps));

% choose governing loaded-region value, but do not let it become silly
p_stringer_front = min(p_stringer_max_front(valid_panel_front));
p_stringer_rear  = min(p_stringer_max_rear(valid_panel_rear));

p_stringer_front = max(p_stringer_front, 0.15);   % practical lower bound
p_stringer_rear  = max(p_stringer_rear,  0.15);

n_stringers_front = max(0, ceil(max(wing_box_width_front) / p_stringer_front) - 1);
n_stringers_rear  = max(0, ceil(max(wing_box_width_rear)  / p_stringer_rear ) - 1);

%% ----- WEB SHEAR BUCKLING -----
tau_web_front = abs(Vf) ./ max(2 .* t_web_front_eff .* wing_box_height_front, eps);
tau_web_rear  = abs(Vr) ./ max(2 .* t_web_rear_eff  .* wing_box_height_rear,  eps);

Kreq_front = tau_web_front ./ max(Cplate .* (t_web_front_eff ./ wing_box_height_front).^2, eps);
Kreq_rear  = tau_web_rear  ./ max(Cplate .* (t_web_rear_eff  ./ wing_box_height_rear ).^2, eps);

aspect_max_front = inf(size(Kreq_front));
aspect_max_rear  = inf(size(Kreq_rear));

idxF = Kreq_front > 5.34;
idxR = Kreq_rear  > 5.34;

aspect_max_front(idxF) = sqrt(4 ./ (Kreq_front(idxF) - 5.34));
aspect_max_rear(idxR)  = sqrt(4 ./ (Kreq_rear(idxR)  - 5.34));

s_rib_max_front = aspect_max_front .* wing_box_height_front;
s_rib_max_rear  = aspect_max_rear  .* wing_box_height_rear;

front_candidates = s_rib_max_front(valid_web_front & isfinite(s_rib_max_front));
rear_candidates  = s_rib_max_rear(valid_web_rear  & isfinite(s_rib_max_rear));

if isempty(front_candidates)
    s_rib_front = 0.7;
else
    s_rib_front = min(front_candidates);
end

if isempty(rear_candidates)
    s_rib_rear = 0.7;
else
    s_rib_rear = min(rear_candidates);
end

s_rib_front = max(s_rib_front, 0.4);   % practical lower bound
s_rib_rear  = max(s_rib_rear,  0.4);

n_ribs_front_half = floor(front.beamlength / s_rib_front) + 1;
n_ribs_rear_half  = floor(rear.beamlength  / s_rib_rear ) + 1;

%% ----- SIMPLE RIB MASS ESTIMATE -----
t_rib   = 0.003;
phi_rib = 0.25;

x_rib_front = linspace(0, front.beamlength, n_ribs_front_half);
x_rib_rear  = linspace(0, rear.beamlength,  n_ribs_rear_half);

c_rib_front = c_rf - ((c_rf - c_tf)/front.beamlength) .* x_rib_front;
c_rib_rear  = c_rr - ((c_rr - c_tr)/rear.beamlength)  .* x_rib_rear;

b_box_rib_front = 0.5    .* c_rib_front;
b_box_rib_rear  = 0.5    .* c_rib_rear;

h_box_rib_front = 0.1086 .* c_rib_front;
h_box_rib_rear  = 0.1086 .* c_rib_rear;

A_rib_mat_front = phi_rib .* b_box_rib_front .* h_box_rib_front;
A_rib_mat_rear  = phi_rib .* b_box_rib_rear  .* h_box_rib_rear;

V_ribs_half_front = sum(A_rib_mat_front .* t_rib);
V_ribs_half_rear  = sum(A_rib_mat_rear  .* t_rib);

mass_ribs_total = 2 * rho_al * (V_ribs_half_front + V_ribs_half_rear);

%% ----- PRINT RESULTS -----
fprintf('\n================ BUCKLING LAYOUT RESULTS ================\n');
fprintf('Front stringer pitch     = %.3f m\n', p_stringer_front);
fprintf('Rear  stringer pitch     = %.3f m\n', p_stringer_rear);
fprintf('Front ribs / half-wing   = %d\n', n_ribs_front_half);
fprintf('Rear  ribs / half-wing   = %d\n', n_ribs_rear_half);
fprintf('Front rib spacing        = %.3f m\n', s_rib_front);
fprintf('Rear  rib spacing        = %.3f m\n', s_rib_rear);
fprintf('Estimated total rib mass = %.1f kg\n', mass_ribs_total);
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


%%  PLOT SHEAR AND MOMENT DIAGRAMS

npts = 300;
[xf, Sf, Mf] = front.diagrams(npts);
[xr, Sr, Mr] = rear.diagrams(npts);

figure('Name','Front Wing Shear / Moment');
subplot(2,1,1)
plot(xf, Sf/1e6, 'LineWidth', 1.8)
grid on
xlabel('x [m]')
ylabel('Shear [MN]')
title('Front Wing Shear Force')

subplot(2,1,2)
plot(xf, Mf/1e6, 'LineWidth', 1.8)
grid on
xlabel('x [m]')
ylabel('Moment [MN m]')
title('Front Wing Bending Moment')

figure('Name','Rear Wing Shear / Moment');
subplot(2,1,1)
plot(xr, Sr/1e6, 'LineWidth', 1.8)
grid on
xlabel('x [m]')
ylabel('Shear [MN]')
title('Rear Wing Shear Force')

subplot(2,1,2)
plot(xr, Mr/1e6, 'LineWidth', 1.8)
grid on
xlabel('x [m]')
ylabel('Moment [MN m]')
title('Rear Wing Bending Moment')
