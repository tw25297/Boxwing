function [DOC, breakdown, no_landings, total_init, labour, V_max] = DOC(MTOM_t, OEM_kg, BlockFuel_kg, fleet_size, SAF_ratio, M_c, T_max_K)
%% ── Fixed mission parameters (F1 freight schedule) ──────────────────────
leg_distances_km = [16847, 8018, 1457, 8052, 1272, 11621, 2264, ...
                     6129,  957, 4462, 6940, 15821, 1204,  7433, ...
                     9782, 13053,  321, 5454];

% [~,a,~,~,~,~,~] = atmos(10670);
sound_speed_kmh = 1062; % a * SI.km * SI.hr;  % convert m/s to km/h
cruise_speed_kmh   = M_c * sound_speed_kmh;   
block_overhead     = 1.12;
total_distance_km  = sum(leg_distances_km);
total_flight_hours = (total_distance_km / cruise_speed_kmh) * block_overhead;
refuel_stops       = 5;
no_landings        = length(leg_distances_km) + refuel_stops;
no_days_parking    = 193;
Ne                 = 2;    % number of engines
N_ft               = 2;    % flight test aircraft
N_ownership        = 1;
%% Derived parameters 
M_e    = OEM_kg;                     % [kg] empty mass for DAPCA
V_max  = cruise_speed_kmh;           % [km/h] max speed proxy
T_3    = T_max_K;                       % [K] turbine inlet temp (fixed)
eta_M  = 1.2;                        % CFRP material factor
eta_MRO = 1.3;                       % novelty maintenance multiplier
ICAO_taxi_configuration = 'E';       % ICAO taxiway configuration (C, D, E, F)
T_max_kN = 370;                     % [kN] max thrust (fixed)
%% Cash Operating Costs 
% Crew
crew_cost   = 150000 * 4 * fleet_size;

% Fuel  (convert kg → litres via density 0.8 kg/L)
fuel_L_per_flight  = BlockFuel_kg / 0.8;
fuel_L_per_season  = fuel_L_per_flight * no_landings;
fuel_price_per_L   = 1.00*(1-SAF_ratio) + 2.00*SAF_ratio;
fuel_cost          = fuel_price_per_L * fuel_L_per_season * fleet_size;

% Landing fees
land_cost   = 25 * MTOM_t * no_landings * fleet_size;

% Parking fees (ICAO Code E)
if ICAO_taxi_configuration == 'C'
    parking_fee_per_day = 1000; % $ per day for configuration C
elseif ICAO_taxi_configuration == 'D'
    parking_fee_per_day = 2000; % $ per day for configuration D
elseif ICAO_taxi_configuration == 'E'
    parking_fee_per_day = 4000; % $ per day for configuration D
elseif ICAO_taxi_configuration == 'F'
    parking_fee_per_day = 6000; % $ per day for configuration D
end
park_cost = parking_fee_per_day * no_days_parking * fleet_size; % $ per year

% Navigation charges (Eurocontrol)
nav_unit    = 90;
nav_weight  = sqrt(MTOM_t / 50);
nav_charges = sum(nav_unit * (leg_distances_km/100) * nav_weight) * fleet_size;

COC = crew_cost + fuel_cost + land_cost + park_cost + nav_charges;

%% DAPCA-IV acquisition cost 
R_E = 115; R_T = 118; R_M = 98; R_Q = 108;  % labour rates [$/hr]
eta_cpi = 1.43;   % CPI 2012 inflation factor

H_E = 5.18  * M_e^0.777 * V_max^0.894 * N_ownership^0.163;
H_T = 7.22  * M_e^0.777 * V_max^0.696 * N_ownership^0.263;
H_M = 10.5  * M_e^0.82  * V_max^0.484 * N_ownership^0.641;
H_Q = 0.076 * H_M;
C_m = 31.2  * M_e^0.921 * V_max^0.621 * N_ownership^0.799;
C_D = 67.4  * M_e^0.63  * V_max^1.3;
C_F = 1947  * M_e^0.325 * V_max^0.822 + N_ft^1.21;
C_E = 3112  * (9.66*T_max_kN + 243.25*M_c + 1.74*T_3 - 2228);

labour       = (H_E*R_E + H_T*R_T + H_M*R_M + H_Q*R_Q) * eta_M;
total_init   = (C_E + C_m + C_F + C_D + labour) * eta_cpi;
cost_per_ac  = total_init / fleet_size;

Ca = cost_per_ac - C_E*eta_cpi*N_ft;   % airframe cost ex-engines
Ce = C_E * eta_cpi;                     % per-engine cost

%% Hull value (Method B: DAPCA + 15% margin) 
V_hull = cost_per_ac * 1.15;

%% Maintenance (Raymer Eq 18.12/18.13) 
maint_per_FH    = 3.3*(Ca/1e6) + 14.2 + (58*(Ce/1e6) - 26.1)*Ne;
maint_per_cycle = 4.0*(Ca/1e6) + 9.3  + (7.5*(Ce/1e6) + 5.6)*Ne;
maint_cost = eta_MRO * (maint_per_FH*total_flight_hours + maint_per_cycle*no_landings) * fleet_size;

%%  Financial Costs 
dep_cost = total_init /(14*fleet_size*total_flight_hours); % aircraft_life_yr;
int_cost = 0.05 * total_init;
ins_cost = 0.006 * V_hull; % * fleet_size;
FC       = dep_cost + int_cost + ins_cost;

%%  Total DOC 
DOC = COC + maint_cost + FC;

%%  Breakdown struct 
breakdown = struct( ...
    'crew',        crew_cost, ...
    'fuel',        fuel_cost, ...
    'landing',     land_cost, ...
    'parking',     park_cost, ...
    'navigation',  nav_charges, ...
    'maintenance', maint_cost, ...
    'depreciation',dep_cost, ...
    'interest',    int_cost, ...
    'insurance',   ins_cost, ...
    'COC',         COC, ...
    'FC',          FC, ...
    'total',       DOC);
end