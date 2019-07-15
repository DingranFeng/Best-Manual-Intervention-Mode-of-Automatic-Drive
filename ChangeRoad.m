function iChange = ChangeRoad(i,nRoad,sigma)
if rand() < sigma
    if i == 1
        iChange = i+1;
    elseif i == nRoad
        iChange = i-1;
    else
        if rand() < 0.5
            iChange = i+1;
        else
            iChange = i-1;
        end
    end
else
    iChange = i;
end