function out = SpeedDown(in,dMax,aMax_down,d,D,ksai)
if d > dMax % 超出司机视野范围
    out = in;
else
    if rand() < ksai % 出错
        out = in; 
    else
        out = max(in-aMax_down,d+D);
    end
end
out = max(out,0);
end