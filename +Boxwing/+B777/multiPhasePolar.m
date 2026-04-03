%% B777.multiPhasePolar
% Multi-phase drag polar for all constraint diagram flight phases.
% All values pulled from obj (ADP or ADP_BW) where possible.
% Fallback assumptions are flagged with ***ASSUMPTION*** comments.
%
% VALUES PULLED FROM ADP/ADP_BW:
%   obj.CD0               -- total clean CD0 (set by B777.CD0)
%   obj.e                 -- Oswald efficiency (set by sizing)
%   obj.CDwave            -- wave drag (set in ADP properties)
%   obj.CL_max            -- clean wing CL_max
%   obj.Delta_Cl_to       -- CL increment flaps TO (REFINEMENT in ADP)
%   obj.Delta_Cl_ld       -- CL increment flaps LD (REFINEMENT in ADP)
%   obj.Delta_CD0_TO      -- CD0 increment flaps TO (add to ADP -- see below)
%   obj.Delta_CD0_LD      -- CD0 increment flaps LD (add to ADP -- see below)
%   obj.TLAR.M_c          -- cruise Mach
%   obj.TLAR.Alt_cruise   -- cruise altitude [m]
%   obj.TLAR.Alt_max      -- ceiling altitude [m]
%   obj.TLAR.M_TO         -- take-off Mach (add to TLAR -- see below)
%   obj.TLAR.M_app        -- approach Mach (add to TLAR -- see below)
%   obj.Span, obj.WingArea, obj.MTOM
%
% VALUES TO ADD TO ADP.m / ADP_BW.m properties block:
%   Delta_CD0_TO = 0.015;  % ***REFINEMENT*** flap drag TO -- Raymer Table 12.7
%   Delta_CD0_LD = 0.055;  % ***REFINEMENT*** flap drag LD -- Raymer Table 12.7
%
% VALUES TO ADD TO TLAR properties block:
%   M_TO  = 0.25;   % ***REFINEMENT*** typical take-off Mach
%   M_app = 0.20;   % ***REFINEMENT*** typical approach Mach
%
% REF: Raymer Ch12 (flap increments), Corke Ch3 (constraint equations)
%
% USAGE:
%   polars = B777.multiPhasePolar(ADP)           % TubeWing
%   polars = B777.multiPhasePolar(ADP_BW, B7Geom) % BoxWing

function polars = multiPhasePolar(obj, B7Geom)
    if nargin < 2
        B7Geom = [];
    end

    % =========================================================
    % STEP 1 -- BASELINE CD0 (clean)
    % =========================================================
    if isempty(obj.CD0) || ~isfinite(obj.CD0) || obj.CD0 <= 0
        fprintf('multiPhasePolar: CD0 not set, computing from B777.CD0...\n');
        [obj.CD0, ~] = B777.CD0(obj, B7Geom);
        B777.UpdateAero(obj);
    end
    CD0_clean = obj.CD0;   % FROM ADP.CD0

    % =========================================================
    % STEP 2 -- FLAP CD0 INCREMENTS
    % Pull from ADP if available, else use Raymer Table 12.7 defaults
    % ADD to ADP.m/ADP_BW.m: Delta_CD0_TO = 0.015; Delta_CD0_LD = 0.055;
    % =========================================================
    if isprop(obj,'Delta_CD0_TO') && ~isempty(obj.Delta_CD0_TO) && isfinite(obj.Delta_CD0_TO)
        Delta_CD0_TO = obj.Delta_CD0_TO;   % FROM ADP.Delta_CD0_TO
        fprintf('  [CD0_TO] from ADP.Delta_CD0_TO = %.4f\n', Delta_CD0_TO);
    else
        Delta_CD0_TO = 0.015;              % ***ASSUMPTION*** Raymer Table 12.7 double-slotted TO
        warning('multiPhasePolar: Delta_CD0_TO not in ADP, using %.4f (Raymer Table 12.7)', Delta_CD0_TO);
    end

    if isprop(obj,'Delta_CD0_LD') && ~isempty(obj.Delta_CD0_LD) && isfinite(obj.Delta_CD0_LD)
        Delta_CD0_LD = obj.Delta_CD0_LD;   % FROM ADP.Delta_CD0_LD
        fprintf('  [CD0_LD] from ADP.Delta_CD0_LD = %.4f\n', Delta_CD0_LD);
    else
        Delta_CD0_LD = 0.055;              % ***ASSUMPTION*** Raymer Table 12.7 double-slotted LD
        warning('multiPhasePolar: Delta_CD0_LD not in ADP, using %.4f (Raymer Table 12.7)', Delta_CD0_LD);
    end

    CD0_TO = CD0_clean + Delta_CD0_TO;
    CD0_LD = CD0_clean + Delta_CD0_LD;

    % =========================================================
    % STEP 3 -- OSWALD EFFICIENCY PER PHASE
    % Clean: FROM ADP.e
    % Flaps: degraded -- ***ASSUMPTION*** Raymer guideline
    %   e_TO = 0.80*e_clean, e_LD = 0.75*e_clean
    % =========================================================
    e_clean = obj.e;   % FROM ADP.e
    e_TO    = 0.80 * e_clean;   % ***ASSUMPTION*** Raymer guideline flaps TO
    e_LD    = 0.75 * e_clean;   % ***ASSUMPTION*** Raymer guideline flaps LD

    % =========================================================
    % STEP 4 -- GEOMETRY
    % =========================================================
    AR     = obj.Span^2 / obj.WingArea;   % FROM ADP.Span, ADP.WingArea
    CDwave = obj.CDwave;                  % FROM ADP.CDwave

    % =========================================================
    % STEP 5 -- CL LIMITS PER PHASE
    % FROM ADP: CL_max, Delta_Cl_to, Delta_Cl_ld
    % =========================================================
    CL_max_clean = obj.CL_max;                        % FROM ADP.CL_max
    CL_max_TO    = obj.CL_max + obj.Delta_Cl_to;      % FROM ADP.Delta_Cl_to
    CL_max_LD    = obj.CL_max + obj.Delta_Cl_ld;      % FROM ADP.Delta_Cl_ld

    % =========================================================
    % STEP 6 -- MACH NUMBERS PER PHASE
    % Cruise: FROM ADP.TLAR.M_c
    % TO/App: FROM ADP.TLAR if available, else ***ASSUMPTION***
    % ADD to TLAR: M_TO = 0.25; M_app = 0.20;
    % =========================================================
    M_cr = obj.TLAR.M_c;   % FROM TLAR.M_c

    if isprop(obj.TLAR,'M_TO') && ~isempty(obj.TLAR.M_TO) && isfinite(obj.TLAR.M_TO)
        M_TO = obj.TLAR.M_TO;   % FROM TLAR.M_TO
    else
        M_TO = 0.25;            % ***ASSUMPTION*** typical widebody TO Mach
        warning('multiPhasePolar: TLAR.M_TO not set, using %.2f', M_TO);
    end

    if isprop(obj.TLAR,'M_app') && ~isempty(obj.TLAR.M_app) && isfinite(obj.TLAR.M_app)
        M_app = obj.TLAR.M_app;   % FROM TLAR.M_app
    else
        M_app = 0.20;             % ***ASSUMPTION*** typical widebody approach Mach
        warning('multiPhasePolar: TLAR.M_app not set, using %.2f', M_app);
    end

    % =========================================================
    % STEP 7 -- ATMOSPHERE + DYNAMIC PRESSURES
    % FROM TLAR.Alt_cruise, TLAR.Alt_max
    % =========================================================
    [rho_SL,   a_SL,  ~,~,~] = cast.atmos(0);
    [rho_cr,   a_cr,  ~,~,~] = cast.atmos(obj.TLAR.Alt_cruise);  % FROM TLAR
    [rho_ceil, a_ceil,~,~,~] = cast.atmos(obj.TLAR.Alt_max);     % FROM TLAR

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
    phases(3).CDwave = 0;        % no wave drag at climb -- ***ASSUMPTION***
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
    WS_SI    = (obj.MTOM * 9.81) / obj.WingArea;   % FROM ADP.MTOM, ADP.WingArea

    for i = 1:n_phases
        Beta_i = 1 / (pi * AR * phases(i).e);
        CD_vec = phases(i).CD0 + Beta_i .* CL_vec.^2 + phases(i).CDwave;
        LD_vec = CL_vec ./ CD_vec;

        mask = CL_vec <= phases(i).CL_max;

        phases(i).CL_vec   = CL_vec(mask);
        phases(i).CD_vec   = CD_vec(mask);
        phases(i).LD_vec   = LD_vec(mask);
        phases(i).Beta     = Beta_i;

        % L/D max
        [LD_max, idx_max]  = max(LD_vec(mask));
        phases(i).LD_max   = LD_max;
        CL_trimmed         = CL_vec(mask);
        phases(i).CL_LDmax = CL_trimmed(idx_max);

        % operating point at this phase q
        phases(i).CL_op = WS_SI / phases(i).q;
        phases(i).CD_op = phases(i).CD0 + Beta_i * phases(i).CL_op^2 + phases(i).CDwave;
        phases(i).LD_op = phases(i).CL_op / phases(i).CD_op;
    end

    % =========================================================
    % STEP 10 -- PRINT SUMMARY
    % =========================================================
    if isa(obj, 'B777.ADP_BW')
        config_name = 'BOXWING';
    else
        config_name = 'TUBEWING';
    end

    fprintf('\n==============================================\n');
    fprintf('  MULTI-PHASE POLAR SUMMARY -- %s\n', config_name);
    fprintf('==============================================\n');
    fprintf('  CD0 clean       = %.5f  [from ADP.CD0]\n',        CD0_clean);
    fprintf('  CD0 TO          = %.5f  (+%.4f Delta_CD0_TO)\n',  CD0_TO, Delta_CD0_TO);
    fprintf('  CD0 LD          = %.5f  (+%.4f Delta_CD0_LD)\n',  CD0_LD, Delta_CD0_LD);
    fprintf('  CL_max clean    = %.3f   [from ADP.CL_max]\n',    CL_max_clean);
    fprintf('  CL_max TO       = %.3f   [from ADP.Delta_Cl_to]\n', CL_max_TO);
    fprintf('  CL_max LD       = %.3f   [from ADP.Delta_Cl_ld]\n', CL_max_LD);
    fprintf('  e clean         = %.3f   [from ADP.e]\n',          e_clean);
    fprintf('  e TO            = %.3f   [***ASSUMPTION*** 0.80*e]\n', e_TO);
    fprintf('  e LD            = %.3f   [***ASSUMPTION*** 0.75*e]\n', e_LD);
    fprintf('  AR              = %.2f   [from ADP.Span/WingArea]\n', AR);
    fprintf('----------------------------------------------\n');
    fprintf('  %-24s %5s %7s %7s %7s %7s\n', ...
            'Phase','M','Alt(ft)','CD0','L/Dmax','CL_op');
    fprintf('----------------------------------------------\n');
    for i = 1:n_phases
        fprintf('  %-24s %5.3f %7.0f %7.4f %7.1f %7.3f\n', ...
                phases(i).name, phases(i).M, ...
                phases(i).alt_m / 0.3048, ...
                phases(i).CD0, phases(i).LD_max, phases(i).CL_op);
    end
    fprintf('==============================================\n\n');

    % =========================================================
    % STEP 11 -- PLOT (separate figure per config)
    % =========================================================
    colors = {'b','g','c','r','m'};
    styles = {'-','-','-','--','--'};

    % use high figure number to avoid clashing with constraint diagram figures
    if isa(obj, 'B777.ADP_BW')
        fig_num = 301;
    else
        fig_num = 300;
    end
    fh = figure(fig_num);
    clf;
    set(fh, 'Name',            sprintf('Multi-Phase Polar -- %s', config_name), ...
            'Position',        [150 150 1200 500], ...
            'WindowStyle',     'docked');   % docks into Figures group

    % CD vs CL
    subplot(1,2,1); hold on; grid on; box on;
    for i = 1:n_phases
        plot(phases(i).CL_vec, phases(i).CD_vec, ...
             [colors{i} styles{i}], 'LineWidth', 1.8, ...
             'DisplayName', sprintf('%s (M=%.2f)', phases(i).name, phases(i).M));
        if isfinite(phases(i).CL_op)
            plot(phases(i).CL_op, phases(i).CD_op, ...
                 [colors{i} 'o'], 'MarkerSize', 7, ...
                 'MarkerFaceColor', colors{i}, 'HandleVisibility','off');
        end
    end
    xlabel('C_L  [-]');
    ylabel('C_D  [-]');
    title(sprintf('Drag Polar  –  %s', config_name));
    legend('Location','northwest','FontSize',8);
    xlim([0 3.0]); ylim([0 0.30]);

    % L/D vs CL
    subplot(1,2,2); hold on; grid on; box on;
    for i = 1:n_phases
        plot(phases(i).CL_vec, phases(i).LD_vec, ...
             [colors{i} styles{i}], 'LineWidth', 1.8, ...
             'DisplayName', sprintf('%s (M=%.2f)', phases(i).name, phases(i).M));
        if isfinite(phases(i).CL_op)
            plot(phases(i).CL_op, phases(i).LD_op, ...
                 [colors{i} 'o'], 'MarkerSize', 7, ...
                 'MarkerFaceColor', colors{i}, 'HandleVisibility','off');
        end
        % triangle at L/D max
        plot(phases(i).CL_LDmax, phases(i).LD_max, ...
             [colors{i} '^'], 'MarkerSize', 6, 'HandleVisibility','off');
    end
    xlabel('C_L  [-]');
    ylabel('L/D  [-]');
    title(sprintf('L/D vs C_L  –  %s', config_name));
    legend('Location','northeast','FontSize',8);
    xlim([0 3.0]); ylim([0 50]);

    sgtitle(sprintf('%s  –  Multi-Phase Drag Polars', config_name), ...
            'FontSize', 12, 'FontWeight', 'bold');

    polars = phases;
end