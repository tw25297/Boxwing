function [mf, t, dx, W_out, TTC_s] = doClimb(W_in, h_start, h_end, dh, ...
    M_cruise, S_ref, T_SLS, eng, polar, V_CAS_lim, H_spd_lim, G, rho_SL)
%  Speed: CAS ≤ V_CAS_lim below H_spd_lim; cruise Mach above.
%  Thrust: max available (T_SLS * sigma^0.75) at each segment midpoint.
%  ROC: energy method  ROC = (T - D) * V / W

W   = W_in;
mf  = 0; t = 0; dx = 0;
h   = h_start;

while h < h_end - 0.1
    h_top = min(h + dh, h_end);
    h_mid = 0.5 * (h + h_top);
    dh_i  = h_top - h;

    [rho, a_s,~,~,~,~,sigma] = Boxwing.cast.atmos(h_mid);

    % Speed schedule
    if h_mid < H_spd_lim
        % Below FL100: TAS from 250 kts CAS (simplified, ignores compressibility)
        TAS   = V_CAS_lim / sqrt(sigma);
        M_loc = TAS / a_s;
    else
        % Above FL100: cruise Mach
        M_loc = M_cruise;
        TAS   = M_loc * a_s;
    end

    % Aerodynamics
    q   = 0.5 * rho * TAS^2;
    CL  = Boxwing.script.MissionAnalysis.clamp(W / (q * S_ref), 0.15, 1.50);
    D   = q * S_ref * polar.CD(CL);

    % Available thrust (Mattingly turbofan lapse)
    T_av = T_SLS * sigma^0.75;

    % Rate of climb (energy method, constant TAS segment)
    F_net = max(T_av - D, 0.003 * W);   % floor at 0.3% W to avoid NaN
    ROC   = F_net * TAS / W;            % [m/s]

    % Segment time, distance, fuel
    dt  = dh_i / ROC;
    ddx = TAS * dt;
    dmf = eng.TSFC(M_loc, h_mid) * T_av * dt;

    mf = mf + dmf;
    t  = t  + dt;
    dx = dx + ddx;
    W  = W  - dmf * G;
    h  = h_top;
end

TTC_s = t;
W_out = W;
end % doClimb