%% Boxwing.B777.multiPhasePolar
% Multi-phase drag polar for all constraint diagram flight phases.
% All values pulled from obj (ADP or ADP_BW) where possible.
% Fallback assumptions are flagged with ***ASSUMPTION*** comments.
%
% FIX (2025-01): cast.atmos → Boxwing.cast.atmos throughout.
%   The error "Dot indexing into the result of a function call requires
%   parentheses" was caused by MATLAB parsing  cast.atmos(x)  as
%   cast().atmos  instead of  Boxwing.cast.atmos(x).  Using the full
%   package path resolves the ambiguity.
%
% REF: Raymer Ch12 (flap increments), Corke Ch3 (constraint equations)

function polars = multiPhasePolar(obj, B7Geom)
    if nargin < 2
        B7Geom = [];
    end

    % =========================================================
    % STEP 1 -- BASELINE CD0 (clean)
    % =========================================================
    if isempty(obj.CD0) || ~isfinite(obj.CD0) || obj.CD0 <= 0
        fprintf('multiPhasePolar: CD0 not set, computing from Boxwing.B777.CD0...\n');
        [obj.CD0, ~] = Boxwing.B777.CD0(obj, B7Geom);
        Boxwing.B777.UpdateAero(obj);
    end
    CD0_clean = obj.CD0;

    % =========================================================
    % STEP 2 -- FLAP CD0 INCREMENTS (Raymer Table 12.7 defaults)
    % =========================================================
    if isprop(obj,'Delta_CD0_TO') && ~isempty(obj.Delta_CD0_TO) && isfinite(obj.Delta_CD0_TO)
        Delta_CD0_TO = obj.Delta_CD0_TO;
    else
        Delta_CD0_TO = 0.015;
        warning('multiPhasePolar: Delta_CD0_TO not in ADP, using 0.0150 (Raymer Table 12.7)');
    end

    if isprop(obj,'Delta_CD0_LD') && ~isempty(obj.Delta_CD0_LD) && isfinite(obj.Delta_CD0_LD)
        Delta_CD0_LD = obj.Delta_CD0_LD;
    else
        Delta_CD0_LD = 0.055;
        warning('multiPhasePolar: Delta_CD0_LD not in ADP, using 0.0550 (Raymer Table 12.7)');
    end

    CD0_TO = CD0_clean + Delta_CD0_TO;
    CD0_LD = CD0_clean + Delta_CD0_LD;

    % =========================================================
    % STEP 3 -- OSWALD EFFICIENCY PER PHASE
    % =========================================================
    e_clean = obj.e;
    e_TO    = 0.80 * e_clean;
    e_LD    = 0.75 * e_clean;

    % =========================================================
    % STEP 4 -- GEOMETRY
    % =========================================================
    AR     = obj.Span^2 / obj.WingArea;
    CDwave = obj.CDwave;

    % =========================================================
    % STEP 5 -- CL LIMITS PER PHASE
    % =========================================================
    CL_max_clean = obj.CL_max;
    CL_max_TO    = obj.CL_max + obj.Delta_Cl_to;
    CL_max_LD    = obj.CL_max + obj.Delta_Cl_ld;

    % =========================================================
    % STEP 6 -- MACH NUMBERS PER PHASE
    % =========================================================
    M_cr = obj.TLAR.M_c;

    if isprop(obj.TLAR,'M_TO') && ~isempty(obj.TLAR.M_TO) && isfinite(obj.TLAR.M_TO)
        M_TO = obj.TLAR.M_TO;
    else
        M_TO = 0.25;
        warning('multiPhasePolar: TLAR.M_TO not set, using 0.25');
    end

    if isprop(obj.TLAR,'M_app') && ~isempty(obj.TLAR.M_app) && isfinite(obj.TLAR.M_app)
        M_app = obj.TLAR.M_app;
    else
        M_app = 0.20;
        warning('multiPhasePolar: TLAR.M_app not set, using 0.20');
    end

    % =========================================================
    % STEP 7 -- ATMOSPHERE + DYNAMIC PRESSURES
    % FIX: use Boxwing.cast.atmos (full package path) to avoid MATLAB
    %      parsing ambiguity that caused the "dot indexing" error.
    % =========================================================
    [rho_SL,   a_SL,  ~,~,~] = Boxwing.cast.atmos(0);
    [rho_cr,   a_cr,  ~,~,~] = Boxwing.cast.atmos(obj.TLAR.Alt_cruise);
    [rho_ceil, a_ceil,~,~,~] = Boxwing.cast.atmos(obj.TLAR.Alt_max);

    q_cr   = 0.5 * rho_cr   * (M_cr  * a_cr)^2;
    q_ceil = 0.5 * rho_ceil * (M_cr  * a_ceil)^2;
    q_TO   = 0.5 * rho_SL   * (M_TO  * a_SL)^2;
    q_app  = 0.5 * rho_SL   * (M_app * a_SL)^2;

    % =========================================================
    % STEP 8 -- DEFINE PHASES
    % =========================================================
    phases(1).name   = 'Cruise';
    phases(1).CD0    = CD0_clean;
    phases(1).e      = e_clean;
    phases(1).CDwave = CDwave;
    phases(1).CL_max = CL_max_clean;
    phases(1).q      = q_cr;
    phases(1).M      = M_cr;
    phases(1).alt_m  = obj.TLAR.Alt_cruise;
    phases(1).config = 'clean';

    phases(2).name   = 'Ceiling';
    phases(2).CD0    = CD0_clean;
    phases(2).e      = e_clean;
    phases(2).CDwave = CDwave;
    phases(2).CL_max = CL_max_clean;
    phases(2).q      = q_ceil;
    phases(2).M      = M_cr;
    phases(2).alt_m  = obj.TLAR.Alt_max;
    phases(2).config = 'clean';

    phases(3).name   = 'Climb (ROC)';
    phases(3).CD0    = CD0_clean;
    phases(3).e      = e_clean;
    phases(3).CDwave = 0;
    phases(3).CL_max = CL_max_clean;
    phases(3).q      = q_cr;
    phases(3).M      = M_cr;
    phases(3).alt_m  = obj.TLAR.Alt_cruise;
    phases(3).config = 'clean';

    phases(4).name   = 'Take-off (TOL/TOCG)';
    phases(4).CD0    = CD0_TO;
    phases(4).e      = e_TO;
    phases(4).CDwave = 0;
    phases(4).CL_max = CL_max_TO;
    phases(4).q      = q_TO;
    phases(4).M      = M_TO;
    phases(4).alt_m  = 0;
    phases(4).config = 'flaps TO';

    phases(5).name   = 'Approach / Landing';
    phases(5).CD0    = CD0_LD;
    phases(5).e      = e_LD;
    phases(5).CDwave = 0;
    phases(5).CL_max = CL_max_LD;
    phases(5).q      = q_app;
    phases(5).M      = M_app;
    phases(5).alt_m  = 0;
    phases(5).config = 'flaps LD';

    % =========================================================
    % STEP 9 -- EVALUATE POLARS
    % =========================================================
    n_phases = numel(phases);
    CL_vec   = linspace(0.1, 3.0, 300);
    WS_SI    = (obj.MTOM * 9.81) / obj.WingArea;

    for i = 1:n_phases
        Beta_i = 1 / (pi * AR * phases(i).e);
        CD_vec = phases(i).CD0 + Beta_i .* CL_vec.^2 + phases(i).CDwave;
        LD_vec = CL_vec ./ CD_vec;

        mask = CL_vec <= phases(i).CL_max;

        phases(i).CL_vec   = CL_vec(mask);
        phases(i).CD_vec   = CD_vec(mask);
        phases(i).LD_vec   = LD_vec(mask);
        phases(i).Beta     = Beta_i;
        phases(i).CD       = @(CL) phases(i).CD0 + Beta_i.*CL.^2 + phases(i).CDwave;

        [LD_max, idx_max]  = max(LD_vec(mask));
        phases(i).LD_max   = LD_max;
        CL_trimmed         = CL_vec(mask);
        phases(i).CL_LDmax = CL_trimmed(idx_max);

        phases(i).CL_op = WS_SI / phases(i).q;
        phases(i).CD_op = phases(i).CD0 + Beta_i * phases(i).CL_op^2 + phases(i).CDwave;
        phases(i).LD_op = phases(i).CL_op / phases(i).CD_op;
    end

    % =========================================================
    % STEP 10 -- PRINT SUMMARY
    % =========================================================
    fprintf('\n==============================================\n');
    fprintf('  MULTI-PHASE POLAR SUMMARY -- BOXWING\n');
    fprintf('==============================================\n');
    fprintf('  CD0 clean       = %.5f\n', CD0_clean);
    fprintf('  CD0 TO          = %.5f  (+%.4f)\n', CD0_TO, Delta_CD0_TO);
    fprintf('  CD0 LD          = %.5f  (+%.4f)\n', CD0_LD, Delta_CD0_LD);
    fprintf('  CL_max clean    = %.3f\n', CL_max_clean);
    fprintf('  e clean         = %.3f\n', e_clean);
    fprintf('  AR              = %.2f\n', AR);
    fprintf('----------------------------------------------\n');
    fprintf('  %-24s %5s %7s %7s %7s\n', 'Phase','M','Alt(ft)','CD0','L/Dmax');
    fprintf('----------------------------------------------\n');
    for i = 1:n_phases
        fprintf('  %-24s %5.3f %7.0f %7.4f %7.1f\n', ...
                phases(i).name, phases(i).M, ...
                phases(i).alt_m / 0.3048, ...
                phases(i).CD0, phases(i).LD_max);
    end
    fprintf('==============================================\n\n');

    polars = phases;
end