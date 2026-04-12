function v = clamp(v, lo, hi)
    v = max(lo, min(hi, v));
end