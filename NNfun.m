function [phiMax,lambda] = NNfun(ratioManual,Tr,sigma,dMax_manual)
%% 参数设定
nRoad = 5; % 道路个数
nLength = 500; % 道路长度
rou0 = 0.3; % 初始车流密度

vMax = 5; % 最大速度
aMax_up_auto = 0.5; % 自动驾驶最大加速加速度
aMax_down_auto = 1; % 自动驾驶最大减速加速度
aMax_up_manual = 0.75*aMax_up_auto; % 手动驾驶最大加速加速度
aMax_down_manual = 0.75*aMax_down_auto; % 手动驾驶最大减速加速度
delta_manual = 0.25; % 手动驾驶随机慢化延迟刹车概率
ksai_manual = 0.01; % 手动驾驶出错概率-------------------------------
ksai_auto = 0.05; % 自动驾驶出错概率-------------------------------


dMax_auto = 14; % 雷达测距最大有效距离-------------------------------
Ttotal = 300; % 演化总时间
Twindow = 10; %出口概率滑窗跨度


img = round(0.5/rou0*rand(2*nRoad+1,nLength)); % 图像矩阵初始化
img = min(img,1);
road = zeros(nRoad,nLength); % 车道矩阵初始化
for i = 1:2*nRoad+1
    if mod(i,2) == 1
        img(i,:) = 1.2;
    else
        road(i/2,:) = img(i,:);
    end
end
v = SpeedStart(road,vMax); % 速度矩阵初始化
roadHist = zeros(Ttotal,nRoad,nLength); % 道路矩阵历史数据
vHist = zeros(Ttotal,nRoad,nLength); % 速度矩阵历史数据
nAccidentHist = zeros(1,Ttotal); % 事故数历史数据

run = 1;
%% 元胞自动机建立
t = 1; % 当前时间
while t <= Ttotal
    nAccident = 0; % 事故数
    if run == 1
        %% 新车辆进入观测区域
        if t > Twindow
            nOut = sum(sum(1-roadHist(t-Twindow:t-1,:,nLength))); % 驶出车辆数
            rOut = nOut/(Twindow*nRoad); % 驶出车辆率
            nIn = nOut; % 驶入车辆数
            rIn = rOut; % 驶入车辆率
            
            for k = 1:nRoad
                [front,vf] = FindFront(road(k,:),v(k,:),nLength,1); % 获取前一辆车的位置和速度
                if front == 1
                    continue;
                else
                    rIn = 1;
                    if rand() < rIn
                        road(k,1) = 0; % 生成新进入车辆
                        v(k,1) = v(k,front); % 新进入车辆速度默认与前车一致
                    end
                end
            end
        end
        %% 速度与位置变化
        for i = 1:nRoad
            for j = 1:nLength
                if road(i,j) ~= 0 % 此处无车
                    continue;
                end
                if v(i,j) < 0 % 处于事故处理中
                    v(i,j) = v(i,j)+1;
                    continue;
                end
                iChange = ChangeRoad(i,nRoad,sigma); % 车辆变道
                if rand(1) > ratioManual         % 自动驾驶速度变化
                    v(i,j) = SpeedUp(v(i,j),aMax_up_auto,vMax,ksai_auto); % 加速过程
                    v(iChange,j) = v(i,j);
                    if iChange ~= i
                        v(i,j) = 0;
                    end
                    [front,vf] = FindFront(road(iChange,:),v(iChange,:),nLength,j); % 获取前一辆车的位置和速度
                    d = front-j; % 车头间距
                    D = vMax/aMax_down_auto*(v(iChange,j)-vf); % 车头间距裕量
                    v(iChange,j) = SpeedDown(v(iChange,j),dMax_auto,aMax_down_auto,d,D,ksai_auto); % 减速过程
                else                             % 手动驾驶速度变化
                    v(i,j) = SpeedUp(v(i,j),aMax_up_manual,vMax,ksai_manual); % 加速过程
                    v(i,j) = SpeedDelayBrake(v(i,j),aMax_down_manual,delta_manual); % 随机慢化延迟刹车过程
                    v(iChange,j) = v(i,j);
                    if iChange ~= i
                        v(i,j) = 0;
                    end
                    [front,vf] = FindFront(road(iChange,:),v(iChange,:),nLength,j); % 获取前一辆车的位置和速度
                    d = front-j; % 车头间距
                    D = vMax/aMax_down_manual*(v(iChange,j)-vf+Tr*(aMax_up_manual+aMax_down_manual)); % 车头间距裕量
                    v(iChange,j) = SpeedDown(v(iChange,j),dMax_manual,aMax_down_manual,d,D,ksai_manual); % 减速过程
                end
            end
        end
        % 位置变化
        roadTmp = ones(nRoad,nLength); % 下一时刻道路矩阵
        for i = 1:nRoad
            for j = 1:nLength
                if road(i,j) ~= 0
                    continue;
                end
                if j+round(v(i,j)) <= nLength
                    if roadTmp(i,j+round(v(i,j))) == 0 % 此处已有车，因此发生追尾
                        nAccident = nAccident+1;
                    end
                    roadTmp(i,j+round(v(i,j))) = 0;
                end
            end
        end
        vHist(t,:,:) = v; % 速度历史记录存储
        roadHist(t,:,:) = roadTmp; % 位置历史记录存储
        nAccidentHist(1,t) = nAccident; % 事故数历史记录存储
        road = roadTmp; % 当前位置数据更新
        t = t+1;
        pause(0.001);
    end
end
flow = zeros(1,Ttotal);
for i = 1:Ttotal
    flow(i) = sum(sum(vHist(i,:,:)))/(nRoad*nLength); % 计算流量
end
phiMax = max(flow);
lambda = poissfit(nAccidentHist);
end

