%% BP神经网络拟合预测
%% 训练目标：输入x,y，输出x^2+y^2
clear all;close all;clc;
format compact;
warning off;
%% 数据定义
nTrain = 100;
% 训练输入
inTrain = [(0.4-0.01)*rand(1,nTrain)+0.01;(1.5-0.1)*rand(1,nTrain)+0.1;...
    (0.3-0.01)*rand(1,nTrain)+0.01;(12-2)*rand(1,nTrain)+2]; 
% 训练输出
outTrain = zeros(1,nTrain);
for i =1:nTrain
    i
    [outTrain(1,i),~] = NNfun(inTrain(1,i),inTrain(2,i),inTrain(3,i),inTrain(4,i));
end
% 测试输入
nTest = 20;
inTest = [(0.4-0.01)*rand(1,nTest)+0.01;(1.5-0.1)*rand(1,nTest)+0.1;...
    (0.3-0.01)*rand(1,nTest)+0.01;(12-2)*rand(1,nTest)+2]; 
%% 数据处理
[inTrain_1,inMin,inMax,outTrain_1,outMin,outMax] = premnmx(inTrain,outTrain); % 训练数据归一化
inTest_1 = premnmx(inTest); % 测试数据归一化
% 隐含层分别隐含层含有个神经元
% 隐含层节点转移函数选用对数S型函数，输出层节点转移函数选用正切S型函数
% 学习训练函数采用最速下降BP算法
neuralNetwork = newff(minmax(inTrain),[20,20,1],{'logsig','logsig','tansig'},'traingd');
neuralNetwork.trainParam.goal = 1e-2; % 训练性能目标最小误差
neuralNetwork.trainParam.epochs = 5000; % 最大训练次数
neuralNetwork.trainparam.lr = 0.3; % 学习速率
neuralNetwork.trainparam.mc = 0.95; % 动量因子,避免陷入局部极值
neuralNetwork.layers{1}.initFcn = 'initwb';
neuralNetwork = init(neuralNetwork); % 权重和偏置的初始化
[neuralNetwork,tr] = train(neuralNetwork,inTrain_1,outTrain_1); % 训练bp网络
outTest_1 = sim(neuralNetwork,inTest_1);
% 测试输出
outTest = postmnmx(outTest_1,outMin,outMax); % 测试结果反归一化
%% 结果分析
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
title('拟合效果','fontsize',14)
hold off