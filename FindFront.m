function [front,vf] = FindFront(thisRoad,thisV,nLength,j)
judge = 0;
if j ~= nLength
    for k = (j+1):nLength
        if thisRoad(k) == 0
            judge = 1;
            front = k;
            vf = thisV(k);
            break;
        end
    end
    if judge == 0
        front = nLength;
        vf = thisV(j);
    end 
else
    front = nLength;
    vf = thisV(j);
end