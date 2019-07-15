%% Ԫ���Զ���ģ�⹫·��ͨ�е��Զ���ʻ/�˹���Ԥת��
clear all;close all;clc;
format compact;
warning off;
%% GUI���
%����
plotbutton = uicontrol('style','pushbutton',...
    'string','����', ...
    'fontsize',12, ...
    'position',[700,600,50,20], ...
    'callback', 'run = 1;');
%��ͣ
erasebutton = uicontrol('style','pushbutton',...
    'string','��ͣ', ...
    'fontsize',12, ...
    'position',[900,600,50,20], ...
    'callback','stop = 1;');
%�˳�
quitbutton = uicontrol('style','pushbutton',...
    'string','�˳�', ...
    'fontsize',12, ...
    'position',[1100,600,50,20], ...
    'callback','quit = 1; close;');
%ʱ�����
word = uicontrol('style','text', ...
    'string','ʱ��', ...
    'fontsize',12, ...
    'position',[850,430,50,20]);
time = uicontrol('style','text', ...
    'string','1', ...
    'fontsize',12, ...
    'position',[950,430,50,20]);
%% �����趨
nRoad = 5; % ��·����
nLength = 500; % ��·����

ratioManual = 0.05; % �˹���Ԥ���ֶ���ʻ��Ƶ��===========================
vMax = 5; % ����ٶ�
aMax_up_auto = 0.5; % �Զ���ʻ�����ټ��ٶ�
aMax_down_auto = 1; % �Զ���ʻ�����ټ��ٶ�
aMax_up_manual = 0.75*aMax_up_auto; % �ֶ���ʻ�����ټ��ٶ�
aMax_down_manual = 0.75*aMax_down_auto; % �ֶ���ʻ�����ټ��ٶ�
delta_manual = 0.25; % �ֶ���ʻ��������ӳ�ɲ������
ksai_manual = 0.01; % �ֶ���ʻ��������===========================
ksai_auto = 0.05; % �Զ���ʻ��������===========================
Tr = 0.5; % ˾����Ӧʱ��===========================
sigma = 0.1; % �����===========================
dMax_auto = 14; % �״��������Ч����===========================
dMax_manual = 8; % ˾�������Ч��Ұ��Χ===========================
Ttotal = 1e3; % �ݻ���ʱ��
Twindow = 10; %���ڸ��ʻ������


for rou0 = 0.1:0.05:0.9 % ��ʼ�����ܶ�========================
    
    img = round(0.5/rou0*rand(2*nRoad+1,nLength)); % ͼ������ʼ��
    img = min(img,1);
    road = zeros(nRoad,nLength); % ���������ʼ��
    for i = 1:2*nRoad+1
        if mod(i,2) == 1
            img(i,:) = 1.2;
        else
            road(i/2,:) = img(i,:);
        end
    end
    v = SpeedStart(road,vMax); % �ٶȾ����ʼ��
    roadHist = zeros(Ttotal,nRoad,nLength); % ��·������ʷ����
    vHist = zeros(Ttotal,nRoad,nLength); % �ٶȾ�����ʷ����
    nAccidentHist = zeros(1,Ttotal); % �¹�����ʷ����
    %��ʼ��ͼ�񣬰�ɫ��ʾ�г�����ɫ��ʾ��Ԫ��
    figure(1)
    imgh = imshow(img(:,:),[],'InitialMagnification','fit'); 
    set(imgh,'erasemode', 'none');
    run = 0; % ���м���ʼ��
    stop = 0; % ��ͣ����ʼ��
    quit = 0; % �˳�����ʼ��

    %% Ԫ���Զ�������
    t = 1; % ��ǰʱ��
    run=1;
    while quit == 0 && t <= Ttotal
        nAccident = 0; % �¹���
        if run == 1
            %% �³�������۲�����
            if t > Twindow 
                nOut = sum(sum(1-roadHist(t-Twindow:t-1,:,nLength))); % ʻ��������
                rOut = nOut/(Twindow*nRoad); % ʻ��������
                nIn = nOut; % ʻ�복����
                rIn = rOut; % ʻ�복����

                for k = 1:nRoad
                    [front,vf] = FindFront(road(k,:),v(k,:),nLength,1); % ��ȡǰһ������λ�ú��ٶ�
                    if front == 1
                        continue;
                    else
                        rIn = 1;
                        if rand() < rIn
                            road(k,1) = 0; % �����½��복��
                            v(k,1) = v(k,front); % �½��복���ٶ�Ĭ����ǰ��һ��
                        end
                    end
                end
            end
            %% �ٶ���λ�ñ仯 
            for i = 1:nRoad
                for j = 1:nLength
                    if road(i,j) ~= 0 % �˴��޳�
                        continue;
                    end
                    if v(i,j) < 0 % �����¹ʴ�����
                        v(i,j) = v(i,j)+1;
                        continue;
                    end
                    iChange = ChangeRoad(i,nRoad,sigma); % �������
                    if rand(1) > ratioManual         % �Զ���ʻ�ٶȱ仯
                        v(i,j) = SpeedUp(v(i,j),aMax_up_auto,vMax,ksai_auto); % ���ٹ���
                        v(iChange,j) = v(i,j);
                        if iChange ~= i
                            v(i,j) = 0;
                        end
                        [front,vf] = FindFront(road(iChange,:),v(iChange,:),nLength,j); % ��ȡǰһ������λ�ú��ٶ�
                        d = front-j; % ��ͷ���
                        D = vMax/aMax_down_auto*(v(iChange,j)-vf); % ��ͷ���ԣ��
                        v(iChange,j) = SpeedDown(v(iChange,j),dMax_auto,aMax_down_auto,d,D,ksai_auto); % ���ٹ���
                    else                             % �ֶ���ʻ�ٶȱ仯
                        v(i,j) = SpeedUp(v(i,j),aMax_up_manual,vMax,ksai_manual); % ���ٹ���
                        v(i,j) = SpeedDelayBrake(v(i,j),aMax_down_manual,delta_manual); % ��������ӳ�ɲ������
                        v(iChange,j) = v(i,j);
                        if iChange ~= i
                            v(i,j) = 0;
                        end
                        [front,vf] = FindFront(road(iChange,:),v(iChange,:),nLength,j); % ��ȡǰһ������λ�ú��ٶ�
                        d = front-j; % ��ͷ���
                        D = vMax/aMax_down_manual*(v(iChange,j)-vf+Tr*(aMax_up_manual+aMax_down_manual)); % ��ͷ���ԣ��
                        v(iChange,j) = SpeedDown(v(iChange,j),dMax_manual,aMax_down_manual,d,D,ksai_manual); % ���ٹ���
                    end
                end
            end
            % λ�ñ仯
            roadTmp = ones(nRoad,nLength); % ��һʱ�̵�·����
            for i = 1:nRoad
                for j = 1:nLength
                    if road(i,j) ~= 0
                        continue;
                    end
                    if j+round(v(i,j)) <= nLength
                        if roadTmp(i,j+round(v(i,j))) == 0 % �˴����г�����˷���׷β
                            nAccident = nAccident+1;
                        end
                        roadTmp(i,j+round(v(i,j))) = 0;
                    end
                end
            end
            vHist(t,:,:) = v; % �ٶ���ʷ��¼�洢
            roadHist(t,:,:) = roadTmp; % λ����ʷ��¼�洢
            nAccidentHist(1,t) = nAccident; % �¹�����ʷ��¼�洢
            road = roadTmp; % ��ǰλ�����ݸ���
            t = t+1;
            set(imgh,'cdata',road); % ����ͼ��
            pause(0.001);
            timeNew = 1+str2double(get(time,'string'));
            set(time,'string',num2str(timeNew)); % ����ʱ���������
        end
        if stop == 1
            run = 0;
            stop = 0;
        end
        drawnow;
    end

    density = zeros(1,Ttotal);
    flow = zeros(1,Ttotal);
    for i = 1:Ttotal
        density(i) = sum(sum(1-roadHist(i,:,:)))/(nRoad*nLength); % �����ܶ�
        flow(i) = sum(sum(vHist(i,:,:)))/(nRoad*nLength); % ��������
    end
    figure(2)
    hold on
    subplot(121)

    semilogy(1:300,density(1:300))
    hold off
    
    figure(2)
    hold on
    subplot(122)

    semilogy(1:300,flow(1:300))
    hold off
end
figure(2)
hold on
subplot(121)
grid on
xlabel('ʱ��/s','fontsize',14)
ylabel('�����ܶȶ���','fontsize',14)
title('��ͬ��ʼ�����ܶ��µĳ����ܶ�-ʱ����������','fontsize',17)
hold off

figure(2)
hold on
subplot(122)
grid on
xlabel('ʱ��/s','fontsize',14)
ylabel('����������','fontsize',14)
title('��ͬ��ʼ�����ܶ��µĳ�����-ʱ����������','fontsize',17)
hold off