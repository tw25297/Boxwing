function [CD_trim, out] = trimDrag(ADP, x_cg)
%TRIMDRAG  Trim drag from rear wing during cruise.
%
%  The rear wing must carry a trim load CL_h to balance pitching moment.
%  This generates additional induced drag = the trim drag.
%
%  USAGE:
%    [CD_trim, out] = trimDrag(ADP, x_cg)
%
%  If x_cg not supplied, uses front wing AC as conservative forward CG.

if nargin < 2 || isempty(x_cg)
    x_cg = ADP.x_ac;
end

%% Geometry from liftingSurfaceAC (already run, values on ADP)
x_ac_f   = ADP.x_ac;
x_ac_r   = ADP.x_ac_rear;
x_ac_sys = ADP.x_ac_sys;

S_ref = ADP.WingArea;
S_r   = ADP.RearWingArea;
b_r   = ADP.RearWingSpan;
AR_r  = b_r^2 / S_r;
e_r   = 0.85;   % conservative Oswald for fwd-swept rear wing

%% Trim lift on rear wing
% Moment balance about front wing AC:
%   CL_h = (x_ac_sys - x_cg) / (x_ac_r - x_ac_f) * CL_cruise * (S_ref/S_r)
l_ac     = x_ac_r - x_ac_f;
delta_x  = x_ac_sys - x_cg;
CL_h     = (delta_x / l_ac) * ADP.CL_cruise * (S_ref / S_r);
CL_h    = max(-1.2, min(1.2, CL_h));   % physical cap

%% Trim drag referenced to S_ref
CD_trim = (CL_h^2) / (pi * AR_r * e_r) * (S_r / S_ref);

%% Diagnostics
out.x_cg        = x_cg;
out.x_ac_sys    = x_ac_sys;
out.delta_x     = delta_x;
out.l_ac        = l_ac;
out.CL_h        = CL_h;
out.CD_trim     = CD_trim;

% fprintf('  Trim drag: delta_x=%.2fm  CL_h=%.4f  CD_trim=%.6f\n', ..., deal with it later if needed
%         delta_x, CL_h, CD_trim);
end