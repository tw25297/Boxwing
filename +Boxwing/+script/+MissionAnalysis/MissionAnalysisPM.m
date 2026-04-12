function [results] = MissionAnalysisPM(fleet, M_c, cruise_altitude, MTOW, target_range)
%% PARAMETERS 
[~,a,~,~,~,~,~] = Boxwing.cast.atmos(cruise_altitude,0);
p = struct();

p.num_aircraft          = fleet;            % [-]    Number of aircraft in fleet
p.cruise_speed_kmh      = M_c * a * 3600/1000;   % [km/h] Cruise ground speed
p.cruise_altitude_ft    = cruise_altitude;  % [ft]   Cruise altitude
p.actual_payload_t      = 736/fleet;        % [t]    Assumed payload per loaded leg
% p.fuel_burn_kg_hr       = fuel_burn_kg_hr;  % [kg/h] Fuel burn rate         *** SET ***
p.MTOW_kg               = MTOW;             % [kg]   Max Take-Off Weight    *** SET ***
% p.OEW_kg                = OEW;              % [kg]   Operating Empty Weight *** SET ***
% p.max_payload_t         = MTOW - OEW;       % [t]    Max structural payload

% Mission Rules and Assumptions
p.refuel_range_km       = target_range;     % [km]   Distance threshold → refuel stop (ADJUSTABLE)
p.refuel_stop_hr        = 2.0;              % [hr]   Duration of each refuel stop
p.warn_margin_hr        = 6.0;              % [hr]   Warn if time margin falls below this


%% TIMEZONE MAP 
tz = containers.Map( ...
    {'EGLL','YMML','ZSPD','RJGG','OBBI','OEJN', ...
     'KMIA','CYUL','LFMN','LEMD','UBBB','WSSS', ...
     'KAUS','MMMX','SBGR','KLAS','OTHH','OMAA'}, ...
    {'Europe/London',       'Australia/Melbourne', 'Asia/Shanghai',   'Asia/Tokyo', ...
     'Asia/Bahrain',        'Asia/Riyadh', ...
     'America/New_York',    'America/Toronto',     'Europe/Paris',    'Europe/Madrid', ...
     'Asia/Baku',           'Asia/Singapore', ...
     'America/Chicago',     'America/Mexico_City', 'America/Sao_Paulo','America/Los_Angeles', ...
     'Asia/Qatar',          'Asia/Dubai'} ...
);


RS = @(icao, y, mo, d, h, mi) datetime(y, mo, d, h, mi, 0, 'TimeZone', tz(icao)); % Race start datetime in local venue time
FD = @(rs) rs + hours(40); % Freight Departure = Race Start + 40 hr  (spreadsheet: E_n + 40/24)
FL = @(rs) dateshift(rs, 'start', 'day') - days(4) + hours(8); % Freight Delivery Limit = midnight of race day − 4 days + 8 hr (local)
UKT = @(dt) datetime(posixtime(dt), 'ConvertFrom', 'posixtime', 'TimeZone', 'Europe/London'); % Convert any timezone-aware datetime to UK Local Time (GMT/BST)

%% RACE START TIMES 
r1  = RS('YMML', 2026,  3,  8, 15, 0);   %  1  Melbourne
r2  = RS('ZSPD', 2026,  3, 15, 15, 0);   %  2  Shanghai
r3  = RS('RJGG', 2026,  3, 29, 14, 0);   %  3  Suzuka
r4  = RS('OBBI', 2026,  4, 12, 18, 0);   %  4  Bahrain
r5  = RS('OEJN', 2026,  4, 19, 20, 0);   %  5  Jeddah
r6  = RS('KMIA', 2026,  5,  3, 16, 0);   %  6  Miami
r7  = RS('CYUL', 2026,  5, 24, 14, 0);   %  7  Montreal
r8  = RS('LFMN', 2026,  6,  7, 15, 0);   %  8  Monaco  
% NO AIR FREIGHT r9 –r15: Barcelona, Austria, Silverstone, Belgium, Hungary, Zandvoort, Monza
r16 = RS('LEMD', 2026,  9, 13, 15, 0);   % 16  Madrid
r17 = RS('UBBB', 2026,  9, 27, 15, 0);   % 17  Baku
r18 = RS('WSSS', 2026, 10, 11, 20, 0);   % 18  Singapore
r19 = RS('KAUS', 2026, 10, 25, 14, 0);   % 19  Austin
r20 = RS('MMMX', 2026, 11,  1, 14, 0);   % 20  Mexico City
r21 = RS('SBGR', 2026, 11,  8, 15, 0);   % 21  São Paulo
r22 = RS('KLAS', 2026, 11, 21, 22, 0);   % 22  Las Vegas
r23 = RS('OTHH', 2026, 11, 29, 20, 0);   % 23  Qatar / Doha
r24 = RS('OMAA', 2026, 12,  6, 17, 0);   % 24  Abu Dhabi


%% LEG DEFINITIONS 

% Pre-compute block time for Leg 00 (LHR→MEL) to derive latest departure
d_L00 = 16847;
stops_L00 = max(0, ceil(d_L00 / p.refuel_range_km) - 1);
bt_L00 = d_L00 / p.cruise_speed_kmh + stops_L00 * p.refuel_stop_hr;

% Pre-compute block time for Leg 17 (AUH→LHR) for estimated arrival
d_L17 = 5454;
stops_L17 = max(0, ceil(d_L17 / p.refuel_range_km) - 1);
bt_L17 = d_L17 / p.cruise_speed_kmh + stops_L17 * p.refuel_stop_hr;

% Empty ferry flight times (used to estimate arrivals at parking airports)
flt_nce_mad = 957  / p.cruise_speed_kmh;   % NCE→MAD
flt_aus_mex = 1204 / p.cruise_speed_kmh;   % AUS→MEX

%  Column order: {label, from, to, dist_km, is_empty, dep_UK, del_UK, notes}
L = { ...
{ 'Leg 00 | LHR → MEL  [Season Start]', 'EGLL','YMML', 16847, false, ...
  UKT(FL(r1)) - hours(bt_L00), ...      % Latest departure from LHR
  UKT(FL(r1)), ...                       % Delivery deadline at MEL
  'Pre-season fleet positioning from UK; departure = latest possible' }; ...
{ 'Leg 01 | MEL → PVG', 'YMML','ZSPD',  8018, false, UKT(FD(r1)), UKT(FL(r2)), '' }; ...
{ 'Leg 02 | PVG → NGO', 'ZSPD','RJGG',  1457, false, UKT(FD(r2)), UKT(FL(r3)), '' }; ...
{ 'Leg 03 | NGO → BAH', 'RJGG','OBBI',  8052, false, UKT(FD(r3)), UKT(FL(r4)), '' }; ...
{ 'Leg 04 | BAH → JED', 'OBBI','OEJN',  1272, false, UKT(FD(r4)), UKT(FL(r5)), '' }; ...
{ 'Leg 05 | JED → MIA', 'OEJN','KMIA', 11621, false, UKT(FD(r5)), UKT(FL(r6)), '' }; ...
{ 'Leg 06 | MIA → YUL', 'KMIA','CYUL',  2264, false, UKT(FD(r6)), UKT(FL(r7)), '' }; ...
{ 'Leg 07 | YUL → NCE', 'CYUL','LFMN',  6129, false, UKT(FD(r7)), UKT(FL(r8)), '' }; ...
{ 'Leg 08 | NCE → MAD  [EMPTY — PARK START]', 'LFMN','LEMD',   957, true,  UKT(FD(r8)), ...
  UKT(FD(r8)) + hours(flt_nce_mad), ...  % Arrival at LEMD (no freight deadline)
  'Fleet flies empty to Madrid. PARKING BEGINS. Races 9-15 covered by road/sea.' }; %  No freight block (parking)
{ 'Leg 09 | MAD → GYD  [PARK END]', 'LEMD','UBBB',  4462, false, UKT(FD(r16)), UKT(FL(r17)), 'PARKING ENDS. Departs after Race 16 Madrid.' }; ...
{ 'Leg 10 | GYD → SIN', 'UBBB','WSSS',  6940, false, UKT(FD(r17)), UKT(FL(r18)), '' }; ...
{ 'Leg 11 | SIN → AUS', 'WSSS','KAUS', 15821, false, UKT(FD(r18)), UKT(FL(r19)), '' }; ...
{ 'Leg 12 | AUS → MEX  [EMPTY — PARK START]', 'KAUS','MMMX',  1204, true, UKT(FD(r19)), UKT(FD(r19)) + hours(flt_aus_mex), ...  % Arrival at MMMX
  'Fleet flies empty to Mexico City. PARKING BEGINS until Race 20.' }; ...
{ 'Leg 13 | MEX → GRU  [PARK END]', 'MMMX','SBGR',  7433, false, UKT(FD(r20)), UKT(FL(r21)), 'PARKING ENDS. Departs after Race 20 Mexico City.' }; ...
{ 'Leg 14 | GRU → LAS', 'SBGR','KLAS',  9782, false, UKT(FD(r21)), UKT(FL(r22)), '' }; ...
{ 'Leg 15 | LAS → DOH', 'KLAS','OTHH', 13053, false, UKT(FD(r22)), UKT(FL(r23)), '' }; ...
{ 'Leg 16 | DOH → AUH', 'OTHH','OMAA',   321, false, UKT(FD(r23)), UKT(FL(r24)), '' }; ...
{ 'Leg 17 | AUH → LHR  [Season End]', 'OMAA','EGLL',  5454, false, UKT(FD(r24)), UKT(FD(r24)) + hours(bt_L17), ...  % Estimated arrival at LHR
  'Post-season fleet return to UK' }; ...
};

N = numel(L);

%%  MISSION CALCULATIONS  

results = struct( ...
    'label', {}, 'from_icao', {}, 'to_icao', {}, 'dist_km', {}, 'is_empty', {}, 'payload_t', {}, ...
    'dep_UK', {}, 'del_UK', {}, 'window_hr', {}, 'flight_hr', {}, 'num_refuel_stops',{}, 'refuel_time_hr', {}, ...
    'block_time_hr', {}, 'num_landings', {}, 'arrival_UK', {}, 'turnaround_hr', {}, 'time_margin_hr',{}, 'notes', {});

for i = 1:N
    leg      = L{i};
    lbl      = leg{1};
    from     = leg{2};
    to       = leg{3};
    dist     = leg{4};
    is_emp   = leg{5};
    dep_uk   = leg{6};
    del_uk   = leg{7};
    notes    = leg{8};

     
    flt_hr  = dist / p.cruise_speed_kmh; %  Flight time (pure air time, no stops)
    n_stops = max(0, ceil(dist / p.refuel_range_km) - 1);
    ref_hr  = n_stops * p.refuel_stop_hr; %  Refuelling stops (if distance exceeds range threshold)
    blk_hr  = flt_hr + ref_hr; %  Block time = flight time + total refuel stop time
    n_land  = 1 + n_stops; %  Landings: 1 at destination + 1 per intermediate refuel stop    
    arr_uk  = dep_uk + hours(blk_hr); %  Arrival at destination (UK time)    
    win_hr  = hours(del_uk - dep_uk); %  Time window [hr]: from freight departure to delivery deadline 
    margin  = win_hr - blk_hr; %  Time margin [hr]: how much slack remains after block time 
    ta_hr   = hours(del_uk - arr_uk);   % Turnaround time [hr]: time between arrival and delivery deadline 
                                        % = margin (same as margin for loaded legs)
    %  Payload 
    if is_emp
        payload = 0;
    else
        payload = p.actual_payload_t;
    end

    %  Store results
    results(i).label            = lbl;
    results(i).from_icao        = from;
    results(i).to_icao          = to;
    results(i).dist_km          = dist;
    results(i).is_empty         = is_emp;
    results(i).payload_t        = payload;
    results(i).dep_UK           = dep_uk;
    results(i).del_UK           = del_uk;
    results(i).window_hr        = win_hr;
    results(i).flight_hr        = flt_hr;
    results(i).num_refuel_stops = n_stops;
    results(i).refuel_time_hr   = ref_hr;
    results(i).block_time_hr    = blk_hr;
    results(i).num_landings     = n_land;
    results(i).arrival_UK       = arr_uk;
    results(i).turnaround_hr    = ta_hr;
    results(i).time_margin_hr   = margin;
    results(i).notes            = notes;
end

%% PARKING CALCULATIONS

park(1).name      = 'Block 1 — LEMD (Madrid)';
park(1).races     = 'Races 9-15: BCN, AUT, SIL, BEL, HUN, ZAN, MON';
park(1).start_UK  = results(9).arrival_UK;    % Leg 08 arrival at MAD
park(1).end_UK    = results(10).dep_UK;        % Leg 09 departure from MAD
park(1).days      = days(park(1).end_UK - park(1).start_UK);
park(1).n_ac      = p.num_aircraft;
park(1).ac_days   = park(1).days * park(1).n_ac;

park(2).name      = 'Block 2 — MMMX (Mexico City)';
park(2).races     = 'Race 20: MEX (positioning wait after Austin empty ferry)';
park(2).start_UK  = results(13).arrival_UK;   % Leg 12 arrival at MEX
park(2).end_UK    = results(14).dep_UK;         % Leg 13 departure from MEX
park(2).days      = days(park(2).end_UK - park(2).start_UK);
park(2).n_ac      = p.num_aircraft;
park(2).ac_days   = park(2).days * park(2).n_ac;

total_ac_days = sum([park.ac_days]);

%%  SEASON TOTALS
tot_dist      = sum([results.dist_km]);
tot_flt_hr    = sum([results.flight_hr]);
tot_blk_hr    = sum([results.block_time_hr]);
tot_lands     = sum([results.num_landings]);
tot_refuels   = sum([results.num_refuel_stops]);
empty_mask    = logical([results.is_empty]);
empty_dist    = sum([results(empty_mask).dist_km]);
loaded_dist   = tot_dist - empty_dist;
deadhead_pct  = 100 * empty_dist / tot_dist;
uniq_airports = numel(unique([{results.from_icao}, {results.to_icao}]));
legs_refueled = find([results.num_refuel_stops] > 0); % Legs that trigger refuel stops
tight_mask    = ([results.time_margin_hr] < p.warn_margin_hr) & ~empty_mask; % Legs with tight time margin (loaded legs only)
tight_legs    = find(tight_mask);


%% OUTPUTS
dt_fmt = 'dd/MM HH:mm';

% fprintf('\n________________________________________________________\n');
% fprintf(' PER-LEG MISSION SUMMARY  (all times UK Local = GMT/BST)\n');
% fprintf('________________________________________________________\n');
% fprintf('%-4s %-30s %7s %3s  %-13s %-13s %6s %5s %5s %7s %5s %5s\n', 'Leg','Route','Dist km','Stp', ...
%     'Dep  (UK)','Del  (UK)', 'Win h','Flt h','Blk h','Margin h','TA h','Land');
% fprintf('%s\n', repmat('-', 1, 104));

for i = 1:N
    r   = results(i);
    lbl = r.label;
    if length(lbl) > 30, lbl = lbl(1:30); 
    end
    flags = '';
    if r.is_empty
        flags = ' [EMPTY]';
    end
    if r.num_refuel_stops > 0
        flags = [flags sprintf(' [+%dR]', r.num_refuel_stops)]; 
    end
    if r.time_margin_hr < p.warn_margin_hr && ~r.is_empty
        flags = [flags '  ** TIGHT **']; 
    end

    % fprintf('%-4d %-30s %7.0f %3d  %-13s %-13s %6.1f %5.1f %5.1f %7.1f %5.1f %5d%s\n', ...
      %   i-1, lbl, r.dist_km, r.num_refuel_stops, ...
      %  char(r.dep_UK, dt_fmt), char(r.del_UK, dt_fmt), ...
      %   r.window_hr, r.flight_hr, r.block_time_hr, ...
      %   r.time_margin_hr, r.turnaround_hr, r.num_landings, flags);
end

% fprintf('\n________________________________________________________\n');
% fprintf(' PARKING BLOCKS  (aircraft at last air-freight airport)\n');
% fprintf('________________________________________________________\n');
% for k = 1:numel(park)
%     fprintf('\n%s\n', park(k).name);
%     fprintf('  Covers  : %s\n',  park(k).races);
%     fprintf('  Start   : %s (UK local)\n', char(park(k).start_UK, 'dd-MMM-yyyy HH:mm z'));
%     fprintf('  End     : %s (UK local)\n', char(park(k).end_UK,   'dd-MMM-yyyy HH:mm z'));
%     fprintf('  Duration: %.1f days\n',     park(k).days);
%     fprintf('  Fleet   : %d aircraft  →  %.0f aircraft-days\n', park(k).n_ac, park(k).ac_days);
% end
% fprintf('\n  TOTAL parking: %.0f aircraft-days across %d blocks\n', total_ac_days, numel(park));

% fprintf('\n________________________________________________________\n');
% fprintf(' SEASON TOTALS\n');
% fprintf('________________________________________________________\n');
% fprintf('  Total legs defined          : %d\n',        N);
% fprintf('  Total season distance       : %8.0f km\n',  tot_dist);
% fprintf('  Total pure flight time      : %8.1f hr   (%.1f days)\n', tot_flt_hr, tot_flt_hr/24);
% fprintf('  Total block time            : %8.1f hr   (flight + refuel stops)\n', tot_blk_hr);
% fprintf('  Total landings (all acft.)  : %8d\n',       tot_lands);
% fprintf('  Legs requiring refuel stop  : %8d   ', numel(legs_refueled));
% if ~isempty(legs_refueled)
%     fprintf('(Legs: %s)', strjoin(arrayfun(@(x) sprintf('%02d',x-1), ...
%         legs_refueled, 'UniformOutput', false), ', '));
% end
% fprintf('\n');
% fprintf('  Total refuel stops          : %8d\n',   tot_refuels);
% fprintf('  Loaded distance             : %8.0f km  (%.0f%%)\n', loaded_dist, 100*loaded_dist/tot_dist);
% fprintf('  Empty ferry distance        : %8.0f km  (%.0f%% deadhead)\n', empty_dist, deadhead_pct);
% fprintf('  Unique airports visited     : %8d\n',   uniq_airports);
% fprintf('  Parking (total acft-days)   : %8.0f\n', total_ac_days);
% if ~isempty(tight_legs)
%     fprintf('\n  ** WARNING: %d leg(s) with time margin < %.0f hr (legs: %s)\n', ...
%         numel(tight_legs), p.warn_margin_hr, ...
%         strjoin(arrayfun(@(x) sprintf('%02d',x-1), tight_legs, 'UniformOutput', false), ', '));
%     fprintf('     Review schedule or increase cruise speed for these legs.\n');
% end
% fprintf('________________________________________________________\n\n');


%% EXPORT TO CSV
T              = table();
T.Leg          = (0 : N-1)';
T.Route        = {results.label}';
T.From_ICAO    = {results.from_icao}';
T.To_ICAO      = {results.to_icao}';
T.Distance_km  = [results.dist_km]';
T.IsEmptyFerry = [results.is_empty]';
T.Payload_t    = [results.payload_t]';
T.Dep_UK       = arrayfun(@(r) char(r.dep_UK, 'dd-MMM-yyyy HH:mm'), ...
                           results, 'UniformOutput', false)';
T.Del_UK       = arrayfun(@(r) char(r.del_UK, 'dd-MMM-yyyy HH:mm'), ...
                           results, 'UniformOutput', false)';
T.Window_hr    = [results.window_hr]';
T.Flight_hr    = [results.flight_hr]';
T.RefuelStops  = [results.num_refuel_stops]';
T.BlockTime_hr = [results.block_time_hr]';
T.Landings     = [results.num_landings]';
T.Margin_hr    = [results.time_margin_hr]';
T.Turnaround_hr= [results.turnaround_hr]';
T.Notes        = {results.notes}';

writetable(T, 'F1_2026_MissionAnalysis.csv');
end

