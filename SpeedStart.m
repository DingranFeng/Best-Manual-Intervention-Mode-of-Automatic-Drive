function v = SpeedStart(road,vMax)
v = zeros(size(road,1),size(road,2));
for i = 1:size(road,1)
    for j = 1:size(road,2)
        if road(i,j) == 0
            v(i,j) = rand(1)*vMax; % ���ٶȷ�Χ��������ȷֲ�
        end
    end
end
end

