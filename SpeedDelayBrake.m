function out = SpeedDelayBrake(in,aMax_down_manual,delta_manual)
if rand() < delta_manual
    out = max(in-aMax_down_manual,0);
else 
    out = in;
end
end