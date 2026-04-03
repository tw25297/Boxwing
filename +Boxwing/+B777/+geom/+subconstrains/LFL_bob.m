% A350F data define
rho_sl = 1.225;          
rho_cruise = 0.380;      % 35,000ft (kg/m^3)
g = 9.81;

% A350F 
Target_LFL = 2200;       
Mach_Cruise = 0.85;      
V_sound_cruise = 295;   
V_cruise = Mach_Cruise * V_sound_cruise;

% aerodynamic data
CL_max_landing = 2.4;   
CD0 = 0.018;            
AR = 9.5;                
e = 0.85;                
Lapse_Rate = 0.25;       %  T_alt / T_sls at 35000ft

% calculation
WS_range = 200:10:1000;  

% LFL
WS_max_LFL = Target_LFL * (0.5 * rho_sl * CL_max_landing * 0.11);

% MACS
q = 0.5 * rho_cruise * V_cruise^2;
TW_cruise_alt = (q * CD0 ./ WS_range) + (WS_range ./ (q * pi * AR * e));

% SLS
TW_cruise_sls = TW_cruise_alt ./ Lapse_Rate;

% OEI 2.1%
Gradient = 0.021;
LD_landing = 9.0;         
N = 2;                    
Weight_Ratio = 0.85;    
TW_missed_maneuver = (Gradient + 1/LD_landing) * (N / (N-1));
TW_missed_sls = ones(size(WS_range)) * TW_missed_maneuver * Weight_Ratio;
