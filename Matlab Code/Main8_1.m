clc
clear
close all

%% 
videoFlag = 1; % ����1�ᱣ����Ƶ
if videoFlag == 1
    writerObj=VideoWriter('test1.avi');  %������Ƶ�ļ������涯��
    open(writerObj);                    % �򿪸���Ƶ�ļ�
end

%% ��ͼ����
% mapOriginal=im2bw(imread('map205-velodyne1.pgm')); 
bw=imread(('map205-velodyne1.pgm'));
se1=strel('square',5);
mapOriginal = im2bw(imerode(bw,se1));

[aa,bb] = find(mapOriginal == 0);
map= mapOriginal(min(aa):max(aa),min(bb):max(bb));
for i = 1:size(map,1)
    for j = 1:size(map,2)
        if map(i,j) ~= 0
            map(i,j) = 0;
        else
            break;
        end
    end
    
    for j = size(map,2):-1:1
        if map(i,j) ~= 0
            map(i,j) = 0;
        else
            break;
        end
    end
end

for j = 1:size(map,2)
    for i = 1:size(map,1)
        if map(i,j) ~= 0
            map(i,j) = 0;
        else
            break;
        end
    end
    
    for j = size(map,2):-1:1
        if map(i,j) ~= 0
            map(i,j) = 0;
        else
            break;
        end
    end
end

%% ͼ����С
resolutionX=25;
resolutionY= 30;
map=imresize(map,[resolutionX resolutionY]);

%% ���Ƶ�ͼ
t0 = 1e-10;
row = size(map,1); col =size(map,2); % ����ߴ�
graph = ones(size(map));
h = figure(); %����ͼ�δ���
set(h, 'outerposition', get(0,'ScreenSize'));
colormap([0 0 0;1 1 1]),pcolor(graph)
hold on
set(gca,'xticklabel',[]);
set(gca,'yticklabel',[]);
pause(t0);
if videoFlag == 1
    frame = getframe;            %��ͼ�������Ƶ�ļ���
    writeVideo(writerObj,frame); %��֡д����Ƶ
end

%% �����ϰ���
[a,b] = find(map == 0);
barrNum = length(a);
barrLoc = [b,a];
for i = 1:barrNum
    temp = barrLoc(i,:);
    barr(i) = rectangle('Position',[temp(1),temp(2),1,1],'Curvature',[0,0],'linewidth',1,'EdgeColor', 'k','facecolor','k');
    
    if i == barrNum
        pause(t0);
        if videoFlag == 1
            frame = getframe;            %��ͼ�������Ƶ�ļ���
            writeVideo(writerObj,frame); %��֡д����Ƶ
        end
    end
end

barrLoc0 = barrLoc; % ��ʼ�ϰ�

%% �����ڽӾ���
adjMat = computeAdjMat(col,row);

%% �����ϰ����ڽӾ���
adjMat = barrAdjMat(barrLoc,adjMat,col,row,inf);

%% ������ڵ�Ķ�Ӧ��ϵ
Index = zeros(col,row);
count = 0;
for i = 1:row
    for j = 1:col
        count = count + 1;
        Index(j,i) = count;
    end
end

%% ���û����������������յ�
robotNum = 10;
Loc = zeros(robotNum,4); % ��Ż����˵������յ�
for i = 1:robotNum
    disp(num2str(i))
    while true
        % �������
        while true
            stopflag = 1;
            S = [randperm(col,1), randperm(row,1)];
            for j = 1:robotNum
                for k = 1:2
                    if j ~= i
                        if Loc(j,(k-1)*2+1) == S(1) && Loc(j,(k-1)*2+2) == S(2)
                            stopflag = 0;
                        end
                    end
                end
            end

            for j = 1:size(barrLoc,1)
                if barrLoc(j,1) == S(1) && barrLoc(j,2) == S(2)
                    stopflag = 0;
                    break;
                end
            end        

            if stopflag == 1
                break;
            end
        end

        % �յ�����
        while true
            E = [randperm(col,1), randperm(row,1)];
            stopflag = 1;
            for j = 1:robotNum
                for k = 1:2
                    if Loc(j,(k-1)*2+1) == E(1) && Loc(j,(k-1)*2+2) == E(2)
                        stopflag = 0;
                    end
                end
            end
            for j = 1:size(barrLoc,1)
                if barrLoc(j,1) == E(1) && barrLoc(j,2) == E(2)
                    stopflag = 0;
                    break;
                end
            end            
            if stopflag == 1
                break;
            end
        end
        [e,L0] = dijkstra(adjMat,Index(S(1),S(2)),Index(E(1),E(2)));
        if e ~= inf  && isSatisfyLimit(map,E, Loc) % ���Ե���
            Loc(i,:) = [S E];
            break
        end
    end
end
LocTemp = Loc;

%% �����յ��ͼ
c = rand(robotNum,3);
for i = 1:robotNum
    r(i) = rectangle('Position',[Loc(i,1)+0.2,Loc(i,2)+0.2,0.6,0.6],'Curvature',[1,1],'linewidth',1,'EdgeColor', 'k','facecolor',c(i,:));
    rectangle('Position',[Loc(i,3)+0.2,Loc(i,4)+0.2,0.6,0.6],'Curvature',[0,0],'linewidth',1,'EdgeColor', 'k','facecolor',c(i,:))
    pause(t0);
    if videoFlag == 1
        frame = getframe;            %��ͼ�������Ƶ�ļ���
        writeVideo(writerObj,frame); %��֡д����Ƶ
    end
end
        
%% ·������
pNum = 2; %һ�ι滮·���ڵ���
complete = zeros(1,robotNum);
planningDis = zeros(1,robotNum); % �滮����
planningTime = zeros(1,robotNum); % �滮ʱ��
planningUnitTime = 0.5; % ���������˹滮ʱ��

L = cell(1,robotNum);
planningNum = 0;
maxInfNum = 5;
cumNum = zeros(1,robotNum); % �ۼ��Ҳ���·���Ĵ���
while ~all(complete >= 1)
    planningNum = planningNum + 1;
    disp(['�滮����: ' num2str(planningNum)]);
    Moving  = cell(1,robotNum);
    NodeCluster = []; % ���ռ�õĽڵ�
    for i = 1:robotNum
        if complete(i) == 0
            % ����һ�������˶��ԣ���������˵�λ����·������ʱ����Ϊ�ϰ�
            adjMat1 = adjMat;
            for j = 1:robotNum               
                if i ~= j
                    adjMat1 = barrAdjMat(Loc(j,1:2),adjMat1,col,row,inf);
                end
            end          
            
            % ·������
            [e,L0] = dijkstra(adjMat1,Index(Loc(i,1),Loc(i,2)),Index(Loc(i,3),Loc(i,4)));
            L{i} = flip(L0); 
            if e == inf
                cumNum(i) = cumNum(i) + 1; % �ۼ��Ҳ���·���Ĵ���
                if (cumNum(i) >= maxInfNum)
                    complete(i) = 2;
                    disp(['robot ' num2str(i) ' can not find path!']);                  
                end
                continue;
            end
            
            % �ۼӹ滮ʱ��
            for j = 1:robotNum 
                if complete(j) == 0
                    planningTime(j) = planningTime(j) + planningUnitTime;
                end
            end
                                      
            % �ڵ�תΪ����
            loc = [];
            for j = 1:length(L{i})
                [x,y] = find(Index == L{i}(j));
                loc = [loc; [x+0.5,y+0.5]];
            end
            loc0 = loc;
            
            % ��·������ͼ
            p = plot(loc0(:,1),loc0(:,2),'--','color',c(i,:),'linewidth',2);
            if videoFlag == 1
                frame = getframe;            %��ͼ�������Ƶ�ļ���
                writeVideo(writerObj,frame); %��֡д����Ƶ
            end
            
            % ����
            if size(loc,1) <= pNum % ���Ե����յ�
                if i == 1 % ��һ��������
                    plot(loc(:,1),loc(:,2),'-','color',c(i,:),'linewidth',2)
                    pause(t0);
                    if videoFlag == 1
                        frame = getframe;            %��ͼ�������Ƶ�ļ���
                        writeVideo(writerObj,frame); %��֡д����Ƶ
                    end
                    complete(i) = 1;
                    disp(['robot ' num2str(i) ' complete!']);
                    Moving{i} = loc;
                    Loc(i,1:2) = Loc(i,3:4);
                    NodeCluster = [NodeCluster; loc];
                else
                    temp = [];
                    for num = 1:size(loc,1)
                        stopflag = 0;
                        node0 = loc(num,:);
                        for h = 1:size(NodeCluster,1)
                            if NodeCluster(h,1) == node0(1) && NodeCluster(h,2) == node0(2)
                                stopflag = 1;
                                break;
                            end
                        end
                        
                        if stopflag == 0
                            temp = [temp; node0];
                        else
                            break;
                        end
                    end
                    
                    if ~isempty(temp)                       
                        plot(temp(:,1),temp(:,2),'-','color',c(i,:),'linewidth',2)
                        pause(t0);
                        if videoFlag == 1
                            frame = getframe;            %��ͼ�������Ƶ�ļ���
                            writeVideo(writerObj,frame); %��֡д����Ƶ
                        end                         
                        Moving{i} = temp;
                        Loc(i,1:2) = temp(end,:)-0.5;
                        if size(loc,1) == size(temp,1)
                            complete(i) = 1;
                            disp(['robot ' num2str(i) ' complete!']);
                        end
                        NodeCluster = [NodeCluster; temp];                            
                    end 
                end                   
            else % ���ܵ����յ�
                if i == 1
                    plot(loc(1:pNum,1),loc(1:pNum,2),'-','color',c(i,:),'linewidth',2)
                    pause(t0);
                    if videoFlag == 1
                        frame = getframe;            %��ͼ�������Ƶ�ļ���
                        writeVideo(writerObj,frame); %��֡д����Ƶ
                    end
                    Moving{i} = loc(1:pNum,:);
                    Loc(i,1:2) = loc(pNum,:)-0.5;
                    NodeCluster = [NodeCluster; loc(1:pNum,:)];
                else
                    temp = [];
                    for num = 1:pNum
                        stopflag = 0;
                        node0 = loc(num,:);
                        for h = 1:size(NodeCluster,1)
                            if NodeCluster(h,1) == node0(1) && NodeCluster(h,2) == node0(2)
                                stopflag = 1;
                                break;
                            end
                        end
                        
                        if stopflag == 0
                            temp = [temp; node0];
                        else
                            break;
                        end
                    end
                    
                    if ~isempty(temp)                       
                        plot(temp(:,1),temp(:,2),'-','color',c(i,:),'linewidth',2)
                        pause(t0);
                        if videoFlag == 1
                            frame = getframe;            %��ͼ�������Ƶ�ļ���
                            writeVideo(writerObj,frame); %��֡д����Ƶ
                        end                         
                        Moving{i} = temp;
                        Loc(i,1:2) = temp(end,:)-0.5;
                        NodeCluster = [NodeCluster; temp];                            
                    end 
                end             
            end    
%             delete(p)
            
            % ��������Ϊ������
            if ~isempty(Moving{i})
                for k = size(Moving{i},1):-1:2
                    adjMat(Index(Moving{i}(k,1)-0.5, Moving{i}(k,2)-0.5),Index(Moving{i}(k-1,1)-0.5, Moving{i}(k-1,2)-0.5)) = inf;
                end    
            end
        end
    end
       
    %% �������ƶ�
    for i = 1:robotNum
        if ~isempty(Moving{i})
            delete(r(i))
             r(i) = rectangle('Position',[Moving{i}(end,1)-0.5+0.2,Moving{i}(end,2)-0.5+0.2,0.6,0.6],'Curvature',[1,1],'linewidth',1,'EdgeColor', 'k','facecolor',c(i,:));  
             pause(t0)
             if videoFlag == 1
                frame = getframe;            %��ͼ�������Ƶ�ļ���
                writeVideo(writerObj,frame); %��֡д����Ƶ
             end             
             planningDis(i) = planningDis(i) + size(Moving{i},1);
             planningTime(i) = planningTime(i) + size(Moving{i},1);
        end
    end
    
    %% �ͷ����򲻿��ñ�
    for i = 1:robotNum
        if ~isempty(Moving{i})
            for k = size(Moving{i},1):-1:2
                adjMat(Index(Moving{i}(k,1)-0.5, Moving{i}(k,2)-0.5),Index(Moving{i}(k-1,1)-0.5, Moving{i}(k-1,2)-0.5)) = 1;
            end    
        end
    end
    
%     %% ��������ϰ��ﶯ̬��
%     if rand < 0.3 && size(barrLoc,1) <= fix(row*col*0.03)
%         temp = [randperm(col,1),randperm(col,1)];
%         flag = 1;
%         for j = 1:robotNum
%             for k = 1:2
%                 if Loc(j,(k-1)*2+1) == temp(1) && Loc(j,(k-1)*2+2) == temp(2)
%                     flag = 0;
%                 end
%             end
%         end
%         if flag == 1
%             barrLoc = [barrLoc; temp];
%             barr(size(barrLoc,1)) = rectangle('Position',[temp(1),temp(2),1,1],'Curvature',[0,0],'linewidth',1,'EdgeColor', 'k','facecolor','k');
%             pause(t0);
%             if videoFlag == 1
%                 frame = getframe;            %��ͼ�������Ƶ�ļ���
%                 writeVideo(writerObj,frame); %��֡д����Ƶ
%             end
%             adjMat = barrAdjMat(temp,adjMat,col,row,inf);
%         end
%     end
    
%     %% ��������ϰ��ﶯ̬��
%     if rand < 0.3 && size(barrLoc,1) >= fix(row*col*0.01)
%         index = randperm(size(barrLoc,1),1);
%         flag = 1;
%         % ��֤�������ϰ���������ĵ�ͼ�ϰ�
%         for kk = 1:size(barrLoc0,1)
%             if barrLoc0(kk,1) == barrLoc(index,1) && barrLoc0(kk,2) == barrLoc(index,2)
%                 flag = 0;
%                 break;
%             end
%         end
%         if flag == 1
%             delete(barr(index));
%             adjMat = barrAdjMat(barrLoc(index,:),adjMat,col,row,1);
%             barrLoc(index,:) = [];
%             pause(t0);
%             if videoFlag == 1
%                 frame = getframe;            %��ͼ�������Ƶ�ļ���
%                 writeVideo(writerObj,frame); %��֡д����Ƶ
%             end
%         end
%     end
end
if videoFlag == 1
    close(writerObj); %�ر���Ƶ�ļ����
end

%% �����ӡ
for i = 1:robotNum
    disp('*************************');
    if complete(i) == 2       
        disp(['Robot ' num2str(i) ' δ�ҵ�·��']);
    elseif complete(i) == 1
        disp(['Robot ' num2str(i) ' �ɹ�����Ŀ�ĵ�, ���: [' num2str(LocTemp(i,1)) ',' num2str(LocTemp(i,2)) '], �յ�: [' num2str(LocTemp(i,3)) ',' num2str(LocTemp(i,4)) ']']);
        disp(['�滮����: ' num2str(planningDis(i)) ',����ʱ��:' num2str(planningTime(i))]);
    end
end