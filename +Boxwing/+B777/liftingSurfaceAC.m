function ac = liftingSurfaceAC(ADP, options)
%LIFTINGSURFACEAC  Compute aerodynamic centres for both boxwing lifting surfaces.
% ... (keep all existing comments) ...

arguments
    ADP
    options.verbose (1,1) logical = true
end

%% ── PLANFORM CONSTANTS ─────────────────────────────────────────────────────
SWEEP_QC_FRONT =  25;
TAPER_FRONT    =  0.35;
SWEEP_QC_REAR  = -20;
TAPER_REAR     =  0.38;

%% ── FRONT WING ─────────────────────────────────────────────────────────────
b_f  = ADP.FrontWingSpan;
S_f  = ADP.FrontWingArea;
lam_f = TAPER_FRONT;
c_r_f = 2 * S_f / (b_f * (1 + lam_f));
c_t_f = lam_f * c_r_f;
MAC_f = (2/3) * c_r_f * (1 + lam_f + lam_f^2) / (1 + lam_f);
y_mac_f = (b_f / 6) * (1 + 2*lam_f) / (1 + lam_f);
sweepLE_f = atand( tand(SWEEP_QC_FRONT) + (c_r_f - c_t_f) / (4 * (b_f/2)) );
x_le_root_f = ADP.FrontWingPos - 0.25 * c_r_f;
x_le_at_ymac_f = x_le_root_f + tand(sweepLE_f) * y_mac_f;
x_ac_f = x_le_at_ymac_f + 0.25 * MAC_f;

%% ── REAR WING ──────────────────────────────────────────────────────────────
b_r  = ADP.RearWingSpan;
S_r  = ADP.RearWingArea;
lam_r = TAPER_REAR;
c_r_r = 2 * S_r / (b_r * (1 + lam_r));
c_t_r = lam_r * c_r_r;
MAC_r = (2/3) * c_r_r * (1 + lam_r + lam_r^2) / (1 + lam_r);
y_mac_r = (b_r / 6) * (1 + 2*lam_r) / (1 + lam_r);
sweepLE_r = atand( tand(SWEEP_QC_REAR) + (c_r_r - c_t_r) / (4 * (b_r/2)) );
x_le_root_r = ADP.RearWingPos - 0.25 * c_r_r;
x_le_at_ymac_r = x_le_root_r + tand(sweepLE_r) * y_mac_r;
x_ac_r = x_le_at_ymac_r + 0.25 * MAC_r;

%% ── COMBINED SYSTEM AC ─────────────────────────────────────────────────────
S_total  = S_f + S_r;
x_ac_sys = (S_f * x_ac_f + S_r * x_ac_r) / S_total;
MAC_sys  = (S_f * MAC_f  + S_r * MAC_r)  / S_total;

%% ── WRITE BACK TO ADP ──────────────────────────────────────────────────────
ADP.c_ac = MAC_f;
ADP.x_ac = x_ac_f;

if isprop(ADP, 'c_ac_rear'); ADP.c_ac_rear = MAC_r;
else; try; addprop(ADP,'c_ac_rear'); ADP.c_ac_rear = MAC_r; catch; end; end

if isprop(ADP, 'x_ac_rear'); ADP.x_ac_rear = x_ac_r;
else; try; addprop(ADP,'x_ac_rear'); ADP.x_ac_rear = x_ac_r; catch; end; end

if isprop(ADP, 'x_ac_sys'); ADP.x_ac_sys = x_ac_sys;
else; try; addprop(ADP,'x_ac_sys'); ADP.x_ac_sys = x_ac_sys; catch; end; end

ADP.MAC = MAC_sys;

%% ── PACKAGE OUTPUT STRUCT ──────────────────────────────────────────────────
ac.front.MAC     = MAC_f;
ac.front.y_MAC   = y_mac_f;
ac.front.x_AC    = x_ac_f;
ac.front.c_r     = c_r_f;
ac.front.c_t     = c_t_f;
ac.front.lambda  = lam_f;
ac.front.sweepQC = SWEEP_QC_FRONT;
ac.front.sweepLE = sweepLE_f;

ac.rear.MAC      = MAC_r;
ac.rear.y_MAC    = y_mac_r;
ac.rear.x_AC     = x_ac_r;
ac.rear.c_r      = c_r_r;
ac.rear.c_t      = c_t_r;
ac.rear.lambda   = lam_r;
ac.rear.sweepQC  = SWEEP_QC_REAR;
ac.rear.sweepLE  = sweepLE_r;

ac.system.x_AC   = x_ac_sys;
ac.system.MAC    = MAC_sys;

%% ── CONSOLE REPORT (only when verbose=true) ────────────────────────────────
if options.verbose
    fprintf('\n══════════════════════════════════════════════════════════\n');
    fprintf('  BOXWING AERODYNAMIC CENTRE DEFINITIONS (Task 2.1)\n');
    fprintf('══════════════════════════════════════════════════════════\n');
    fprintf('\n  FRONT WING  (aft sweep, +%.0f deg QC)\n', SWEEP_QC_FRONT);
    fprintf('    Root chord c_r        : %6.3f m\n', c_r_f);
    fprintf('    Tip  chord c_t        : %6.3f m\n', c_t_f);
    fprintf('    Taper ratio lambda    : %6.3f\n',   lam_f);
    fprintf('    MAC                   : %6.3f m\n', MAC_f);
    fprintf('    Spanwise MAC station  : %6.3f m  (%.1f%% semi-span)\n', ...
            y_mac_f, 100*y_mac_f/(b_f/2));
    fprintf('    AC x-position         : %6.3f m  (from nose)\n', x_ac_f);
    fprintf('\n  REAR WING  (forward sweep, %.0f deg QC)\n', SWEEP_QC_REAR);
    fprintf('    Root chord c_r        : %6.3f m\n', c_r_r);
    fprintf('    Tip  chord c_t        : %6.3f m\n', c_t_r);
    fprintf('    Taper ratio lambda    : %6.3f\n',   lam_r);
    fprintf('    MAC                   : %6.3f m\n', MAC_r);
    fprintf('    Spanwise MAC station  : %6.3f m  (%.1f%% semi-span)\n', ...
            y_mac_r, 100*y_mac_r/(b_r/2));
    fprintf('    AC x-position         : %6.3f m  (from nose)\n', x_ac_r);
    fprintf('\n  COMBINED SYSTEM (area-weighted)\n');
    fprintf('    System AC x-position  : %6.3f m  (from nose)\n', x_ac_sys);
    fprintf('    System MAC            : %6.3f m\n', MAC_sys);
    fprintf('    Total lifting area    : %6.1f m^2\n', S_total);
    fprintf('    AC arm (front->rear)  : %6.3f m\n', x_ac_r - x_ac_f);
    fprintf('══════════════════════════════════════════════════════════\n\n');
end

end