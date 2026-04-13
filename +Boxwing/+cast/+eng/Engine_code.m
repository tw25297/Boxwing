% function [ATR100, climate_bd] = Engine_code(MTOM, OEW, s_ref, ar, range_km,  mach)
function [ATR100, climate_bd] = Engine_code(season, per_leg)
    % % Global Parameters
    % m_tom = MTOM;             
    % m_oew = OEW;               
    % m_payload = m_tom - m_oew;            
    % %s_ref = s_ref;                
    % %ar = ar;                    
    % e_app = 1.80;                 
    % cd0 = 0.0221;                 
    % k = 1 / (pi * ar * e_app);    
    % g = 9.80665;                  
    % 
    % % Propulsion Profile
    % eng_fn0 = 362300;             
    % eng_sfc_a = 1.41e-5;          
    % eng_sfc_b = 0.48e-5;          
    % 
    % % Mission 
    % % range_km = range;        
    % alt_m = 10668;                
    % % mach = 0.80;                  
    % 
    % [T_cruise, a_cruise, p_cruise, rho_cruise] = atmosisa(alt_m);
    % [T_sl, a_sl, p_sl, rho_sl] = atmosisa(0);
    % 
    % v_cruise = mach * a_cruise;   
    % delta = p_cruise / p_sl;      
    % theta = T_cruise / T_sl;      
    % 
    % % Full Mission Profile
    % % Take-off
    % w0 = m_tom * g;
    % ff_to = 0.970;
    % w1 = w0 * ff_to;
    % fuel_to = (w0 - w1) / g;
    % 
    % % Climb
    % ff_climb = 0.985;
    % w2 = w1 * ff_climb;
    % fuel_climb = (w1 - w2) / g;
    % 
    % % Cruise
    % w_start_cruise = w2;
    % cl_cruise = w_start_cruise / (0.5 * rho_cruise * v_cruise^2 * s_ref);
    % cd_cruise = cd0 + k * cl_cruise^2;
    % ld_cruise = cl_cruise / cd_cruise;
    % 
    % % TSFC Refined for Altitude & Mach
    % sfc_cruise = (eng_sfc_a + eng_sfc_b * mach) * sqrt(theta);
    % 
    % range_m = range_km * 1000;
    % weight_ratio = exp((range_m * g * sfc_cruise) / (v_cruise * ld_cruise));
    % w3 = w_start_cruise / weight_ratio;
    % fuel_cruise = (w_start_cruise - w3) / g;
    % 
    % % Approach
    % ff_app = 0.992;
    % w4 = w3 * ff_app;
    % fuel_app = (w3 - w4) / g;
    
    
    total_fuel_burn(i) = per_leg(i).BlockFuel_kg;

    % Climate Impact 
    ei_co2 = 3.16;
    ei_h2o = 1.26;         
    ei_nox = 16.2;         
    ei_so4 = 0.2;       
    ei_soot = 0.04;         

    % ATR100 Factors 
    a_co2      = 1.01e-15;
    a_nox_o3s  = 5.16e-13;  
    a_nox_o3l  = -1.21e-13; 
    a_h2o      = 2.40e-15;
    a_contrail_km = 1.10e-14 / 1.852; 

    m_co2 = total_fuel_burn * ei_co2;
    m_nox = (total_fuel_burn / 1000) * ei_nox;
    m_h2o = total_fuel_burn * ei_h2o;

    atr100_co2      = m_co2 * a_co2;
    atr100_nox      = m_nox * (a_nox_o3s + a_nox_o3l); 
    atr100_h2o      = m_h2o * a_h2o;
    atr100_contrail = range_km * a_contrail_km; 
    
    atr100_total = atr100_co2 + atr100_nox + atr100_h2o + atr100_contrail;
    ATR100 = atr100_total;
    climate_bd.ATR_CO2 = atr100_co2;
    climate_bd.ATR_NOx = atr100_nox;
    climate_bd.ATR_H2O = atr100_h2o;    
    climate_bd.ATR_AIC = atr100_contrail;
    % Propulsion Scaling
    t_ref = 374500;
    w_ref = 7277;
    l_ref = 4.50;
    d_ref = 3.00;
    tw_req = 0.226;
    
    t_total_req = (m_tom * g) * tw_req;
    t_req_per_eng = t_total_req / 2;
    sf = t_req_per_eng / t_ref;
    
    w_eng_scaled = w_ref * (sf)^1.1;
    l_eng_scaled = l_ref * (sf)^0.4;
    d_eng_scaled = d_ref * (sf)^0.5;
    w_propulsion = w_eng_scaled * 1.30; 

    % OEI Verification
    t_oei = 363027; 
    cl_to = 1.2;
    delta_cd0_to = 0.02;
    
    cd_to = (cd0 + delta_cd0_to) + (cl_to^2 / (pi * ar * e_app));
    ld_to = cl_to / cd_to;
    gamma_oei = (t_oei * 0.95 / (m_tom * g)) - (1 / ld_to);
    
    ld_cruise_fixed = 22.89; 
    sigma_oei = ( ((m_tom * g) / ld_cruise_fixed) / t_oei )^(1 / 0.7);

    % Result
    fprintf('MISSION PERFORMANCE (Metric Units):\n');
    fprintf('  Range: %.2f km | Cruise: %d m (M%.2f)\n', range_km, alt_m, mach);
    fprintf('  Initial Cruise L/D: %.2f\n', ld_cruise);
    fprintf('  Refined TSFC: %.4e kg/N/s\n', sfc_cruise);
    fprintf('  Total Mission Fuel: %.2f kg\n', total_fuel_burn);
    fprintf('CLIMATE IMPACT:\n');
    fprintf('  CO2 Mass: %.2f kg | NOx Mass: %.2f kg\n', m_co2, m_nox);
    fprintf('  ATR100 Breakdown: CO2=%.2e, NOx=%.2e, H2O=%.2e, Contrails=%.2e\n', ...
            atr100_co2, atr100_nox, atr100_h2o, atr100_contrail);
    fprintf('  ATR100 Total: %.4e K\n', atr100_total);
    fprintf('PROPULSION SCALING:\n');
    fprintf('  Required Thrust/Eng: %.2f kN\n', t_req_per_eng/1000);
    fprintf('  Total Propulsion System Mass: %.2f kg\n', w_propulsion * 2);
    fprintf('OEI VERIFICATION:\n');
    fprintf('  Take-off Gradient: %.2f%%\n', gamma_oei * 100);
end