function out = SpeedDown(in,dMax,aMax_down,d,D,ksai)
if d > dMax % ����˾����Ұ��Χ
    out = in;
else
    if rand() < ksai % ����
        out = in; 
    else
        out = max(in-aMax_down,d+D);
    end
end
out = max(out,0);
end