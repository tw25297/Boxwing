function [mf, t, dx, W_out] = doDescent(W_in, h_start, h_end, dh, ...
    M_cruise, S_ref, T_SLS, idle_frac, tsfc_mult, eng, polar, ...
    ROD_target, V_CAS_lim, H_spd_lim, G, rho_SL)

%  Speed schedule: mirrors climb (250 kts CAS below FL100, cruise Mach above).
%  Idle thrust: idle_frac * T_SLS * sigma^0.75.
%  ROD: prescribed at ROD_target; TSFC multiplied by tsfc_mult for idle.

W   = W_in;
mf  = 0; t = 0; dx = 0;
h   = h_start;

while h > h_end + 0.1
    h_bot = max(h - dh, h_end);
    h_mid = 0.5 * (h + h_bot);
    dh_i  = h - h_bot;

    [rho, a_s,~,~,~,~,sigma] = Boxwing.cast.atmos(h_mid);

    % Speed schedule (mirror of climb)
    if h_mid < H_spd_lim
        TAS   = V_CAS_lim / sqrt(sigma);
        M_loc = TAS / a_s;
    else
        M_loc = M_cruise;
        TAS   = M_loc * a_s;
    end

    % Idle thrust and aerodynamics
    T_idle = idle_frac * T_SLS * sigma^0.75;
    q      = 0.5 * rho * TAS^2;
    CL     = Boxwing.script.MissionAnalysis.clamp(W / (q * S_ref), 0.05, 1.20);
    D      = q * S_ref * polar.CD(CL);

    % Natural ROD from force balance at idle:
    %   T_idle + W*sin(γ) = D  → ROD_nat = TAS*(D - T_idle)/W
    ROD_nat = TAS * (D - T_idle) / W;
    ROD_nat = max(ROD_nat, 0.5);   % floor at 0.5 m/s

    % Use prescribed ROD (capped at natural to remain physical)
    ROD = min(ROD_target, ROD_nat * 1.2);
    ROD = max(ROD, 0.5);

    dt  = dh_i / ROD;
    ddx = TAS * dt;

    % Idle fuel burn
    TSFC_d = eng.TSFC(M_loc, h_mid) * tsfc_mult;
    dmf    = max(TSFC_d * T_idle * dt, 0);

    mf = mf + dmf;
    t  = t  + dt;
    dx = dx + ddx;
    W  = W  - dmf * G;
    h  = h_bot;
end

W_out = W;
end % doDescent