%% ��ģ������
clear all;close all;clc;
format compact;
warning off;
%% ���ݶ���
nPoint = 1500; % �Ƶθ���&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
ndata = 200;
data = zeros(ndata,2);% row:variables column:observation
for i= 1:ndata
    i
    [data(i,1),data(i,2)] = NNfun(0.163,0.039,0.073,7.582);
end

data = data'; % row:observation column:variables
%% ��ģ�ͼ���
figure;
for i = 1:2
    % value:�Ƶ���ֵ degree:���������ȣ�����������ȶ��̶ȣ�
    % Ex:���� En:�� He:����
    [value,degree,Ex,En,He] = cloud_transform(data(i,:),nPoint);
    Ex,En,He
    subplot(1,2,i);
    hold on;
    plot(value,degree,'b.','markersize',8);
    plot([Ex,Ex],[0,1],'r--','linewidth',1.8);
    xlabel('��ֵ�ֲ�','fontsize',14);
    ylabel('������','fontsize',14);
    if i==1
        title('\phi_{max}','fontsize',16);
    end
    if i==2
        title('\lambda','fontsize',16);
    end
    grid on;
    hold off
end