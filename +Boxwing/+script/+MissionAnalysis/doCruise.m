function [mf, t, dx, h_fin, W_out, warns] = doCruise(W_in, h_start, h_ceil, R_total, dh_step, ...
    M_cruise, S_ref, T_SLS, eng, polar, ROC_min, G, rho_SL)
%  At each chunk boundary, checks if stepping up dh_step is feasible:
%    ROC at (h + dh_step) ≥ ROC_min  →  perform step climb.
%  Step climbs use 500 ft internal segments for fuel accuracy.

CHUNK = 200e3;   % [m] Breguet chunk size

W         = W_in;
h         = h_start;
mf        = 0; t = 0; dx = 0;
remaining = R_total;
warns     = {};

% Verify ROC ≥ 300 ft/min at initial cruise altitude
[rho0, a0,~,~,~,~,sig0] = Boxwing.cast.atmos(h);
TAS0 = M_cruise * a0;
q0   = 0.5 * rho0 * TAS0^2;
CL0  = Boxwing.script.MissionAnalysis.clamp(W / (q0 * S_ref), 0.15, 1.20);
D0   = q0 * S_ref * polar.CD(CL0);
ROC0 = (T_SLS * sig0^0.75 - D0) * TAS0 / W;
if ROC0 < ROC_min
    warns{end+1} = sprintf('SPEC VIOLATION: ROC = %.0f ft/min < 300 at FL%.0f at start of cruise', ...
                            ROC0 * 196.85, h/SI.ft/100);
end

while remaining > 500
    % Check step-climb feasibility
    h_next = h + dh_step;
    if h_next <= h_ceil
        [rho_n, a_n,~,~,~,~,sig_n] = Boxwing.cast.atmos(h_next);
        TAS_n = M_cruise * a_n;
        q_n   = 0.5 * rho_n * TAS_n^2;
        CL_n  = Boxwing.script.MissionAnalysis.clamp(W / (q_n * S_ref), 0.15, 1.20);
        D_n   = q_n * S_ref * polar.CD(CL_n);
        ROC_n = (T_SLS * sig_n^0.75 - D_n) * TAS_n / W;

        if ROC_n >= ROC_min
            % Perform step climb with 500 ft sub-segments
            dh_sub = SI.ft * 500;
            [mf_s, t_s, dx_s, W, ~] = doClimb( ...
                W, h, h_next, dh_sub, ...
                M_cruise, S_ref, T_SLS, eng, polar, ...
                250*0.51444, 3048, G, rho_SL);  % no CAS limit during cruise step

            mf        = mf + mf_s;
            t         = t  + t_s;
            dx        = dx + dx_s;
            remaining = remaining - dx_s;
            h         = h_next;
            continue   % re-evaluate at new altitude
        end
    end

    % Breguet cruise chunk at current altitude 
    [rho, a_s,~,~,~,~,~] = Boxwing.cast.atmos(h);
    TAS    = M_cruise * a_s;
    q      = 0.5 * rho * TAS^2;
    CL     = Boxwing.script.MissionAnalysis.clamp(W / (q * S_ref), 0.15, 1.20);
    LD     = CL / polar.CD(CL);
    TSFC_c = eng.TSFC(M_cruise, h);

    chunk  = min(remaining, CHUNK);
    frac   = exp(-(chunk * G * TSFC_c) / (TAS * LD));
    W_end  = W * frac;
    dmf    = (W - W_end) / G;
    dt     = chunk / TAS;

    mf        = mf + dmf;
    t         = t  + dt;
    dx        = dx + chunk;
    W         = W_end;
    remaining = remaining - chunk;
end

h_fin = h;
W_out = W;
end % doCruise