function segs = logSeg(segs, name, fuel, time, dist)
    segs(end+1) = struct('name', name, 'fuel', fuel, 'time', time, 'dist', dist);
end