%% BP���������Ԥ��
%% ѵ��Ŀ�꣺����x,y�����x^2+y^2
clear all;close all;clc;
format compact;
warning off;
%% ���ݶ���
nTrain = 100;
% ѵ������
inTrain = [(0.4-0.01)*rand(1,nTrain)+0.01;(1.5-0.1)*rand(1,nTrain)+0.1;...
    (0.3-0.01)*rand(1,nTrain)+0.01;(12-2)*rand(1,nTrain)+2]; 
% ѵ�����
outTrain = zeros(1,nTrain);
for i =1:nTrain
    i
    [outTrain(1,i),~] = NNfun(inTrain(1,i),inTrain(2,i),inTrain(3,i),inTrain(4,i));
end
% ��������
nTest = 20;
inTest = [(0.4-0.01)*rand(1,nTest)+0.01;(1.5-0.1)*rand(1,nTest)+0.1;...
    (0.3-0.01)*rand(1,nTest)+0.01;(12-2)*rand(1,nTest)+2]; 
%% ���ݴ���
[inTrain_1,inMin,inMax,outTrain_1,outMin,outMax] = premnmx(inTrain,outTrain); % ѵ�����ݹ�һ��
inTest_1 = premnmx(inTest); % �������ݹ�һ��
% ������ֱ������㺬�и���Ԫ
% ������ڵ�ת�ƺ���ѡ�ö���S�ͺ����������ڵ�ת�ƺ���ѡ������S�ͺ���
% ѧϰѵ���������������½�BP�㷨
neuralNetwork = newff(minmax(inTrain),[20,20,1],{'logsig','logsig','tansig'},'traingd');
neuralNetwork.trainParam.goal = 1e-2; % ѵ������Ŀ����С���
neuralNetwork.trainParam.epochs = 5000; % ���ѵ������
neuralNetwork.trainparam.lr = 0.3; % ѧϰ����
neuralNetwork.trainparam.mc = 0.95; % ��������,��������ֲ���ֵ
neuralNetwork.layers{1}.initFcn = 'initwb';
neuralNetwork = init(neuralNetwork); % Ȩ�غ�ƫ�õĳ�ʼ��
[neuralNetwork,tr] = train(neuralNetwork,inTrain_1,outTrain_1); % ѵ��bp����
outTest_1 = sim(neuralNetwork,inTest_1);
% �������
outTest = postmnmx(outTest_1,outMin,outMax); % ���Խ������һ��
%% �������
x = linspace(0.01,0.4,10);
y = linspace(0.1,1.5,10);
[X,Y] = meshgrid(x,y);
Z = zeros(length(x),length(y));
for i =1:length(x)
    for j =1:length(y)
        i,j
        [tmp,~] = NNfun(x(i),y(j),0.1,8);
        Z(i,j) = tmp;
    end
end
Z = Z';
figure;
hold on;
mesh(X,Y,Z);
plot3(inTest(1,:),inTest(2,:),outTest,'r*','markersize',10);
grid on
hidden off
title('���Ч��','fontsize',14)
hold off