function out = SpeedUp(in,aMax_up,vMax,ksai)
if rand() < ksai % ³ö´í
    out = in;
else
    out = min(in+aMax_up,vMax);
end
end