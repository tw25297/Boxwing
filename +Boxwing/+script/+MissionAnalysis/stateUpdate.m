function [W_out, mf_out, t_out] = stateUpdate(W, mf, t, dmf, dt, G)
    W_out  = W  - dmf * G;
    mf_out = mf + dmf;
    t_out  = t  + dt;
end