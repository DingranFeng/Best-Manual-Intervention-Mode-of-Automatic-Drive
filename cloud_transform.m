function [value,degree,Ex,En,He] = cloud_transform(y_spor,n)
% value:云滴 degree:钟形隶属度（度量倾向的稳定程度） 
% Ex:期望 En:熵 He:超熵
    value = zeros(1,n);
    degree = zeros(1,n);
    Ex = mean(y_spor);
    En = mean(abs(y_spor-Ex)).*sqrt(pi./2);
    He = sqrt(var(y_spor)-En.^2);
    
    for q = 1:n
        Enn = randn(1).*He+En;
        value(q) = randn(1).*Enn+Ex;
        degree(q) = exp(-(value(q)-Ex).^2./(2.*Enn.^2));
    end
end

