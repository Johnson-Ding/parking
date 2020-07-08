function PlotTraj(obstacleList, path, VEHICLE, CONFIG, imgPath, figNum, figTitle, timeInterval)
%PLOTTRAJ ��ʾ�����˶��켣
%   obstacleList���ϰ����λ���б���m*3�ľ��󣬵�һ�е�����������Ϊ�����ꡢ�����ꡢƫ����
%   path�ǳ����˶�·������n*4�ľ��󣬵�һ�е�����������Ϊ�����ꡢ�����ꡢƫ���ǡ�ǰ��ת��
%   VEHICLE��struct���������������ĸ��ֲ���
%   CONFIG��struct�����������������õĸ��ֲ���
    
    DIST = VEHICLE.LENGTH/2 - VEHICLE.LB;   %����ԭ�㣨���������ģ��������������ĵľ���
    figure(figNum);
    title(figTitle);
    set(gcf,'color','w')
    set(gca,'XTick',[0,5,10,15,20],'color','w');
    set(gca,'YTick',[0,3,6,9,12,15],'color','w');
%     grid on
    xlabel('$X/m$','Interpreter','latex');
    ylabel('$Y/m$','Interpreter','latex');    
    axis equal;
    PlotMap(CONFIG);
    hold on;
    PlotPath(path,'r');
    hold on;
    [obstacleListLength, ~] = size(obstacleList);
    for i = 1:1:obstacleListLength
        PlotObstacle(obstacleList(i,:), VEHICLE.LENGTH, VEHICLE.WIDTH);
        hold on;
    end
    axis equal;
    [pathLength, ~] = size(path);
    for i = 1:1:pathLength
        PlotVehicle(path(i,1:3), path(i,4), DIST, VEHICLE);
        PlotPath(path,'r');
        PlotPath(path(1:i,:),'g');
        hold on;
        pause(timeInterval);
        photo = getframe(gcf);
        imind = frame2im(photo);
        [imind,cm] = rgb2ind(imind,256);
        if i == 1
            imwrite(imind,cm,strcat(imgPath, figTitle,'.gif'),'GIF', 'Loopcount',inf,'DelayTime',timeInterval);
        else
            imwrite(imind,cm,strcat(imgPath, figTitle,'.gif'),'GIF','WriteMode','append','DelayTime',timeInterval);
        end 
        EraseVehicle(path(i,1:3), path(i,4), DIST, VEHICLE);
        PlotMap(CONFIG);
        for j = 1:1:obstacleListLength
            PlotObstacle(obstacleList(j,:), VEHICLE.LENGTH, VEHICLE.WIDTH);
            hold on;
        end
        hold on;
    end
    PlotVehicle(path(i,1:3), path(i,4), DIST, VEHICLE);
    PlotPath(path,'g');
    hold on;
    pause(1);
    photo = getframe(gcf);
    imind = frame2im(photo);
    [imind,cm] = rgb2ind(imind,256);
    if i == 1
        imwrite(imind,cm,strcat(imgPath, figTitle,'.gif'),'GIF', 'Loopcount',inf,'DelayTime',1);
    else
        imwrite(imind,cm,strcat(imgPath, figTitle,'.gif'),'GIF','WriteMode','append','DelayTime',1);
    end 
end

function PlotMap(CONFIG)
    x1 = CONFIG.MAP_XLIM(1); x2 = CONFIG.MAP_XLIM(1)+CONFIG.WALL_WIDTH; 
    x3 = CONFIG.MAP_XLIM(2)-CONFIG.WALL_WIDTH; x4 = CONFIG.MAP_XLIM(2);
    y1 = CONFIG.MAP_YLIM(1); y2 = CONFIG.MAP_YLIM(1)+CONFIG.WALL_WIDTH; 
    y3 = CONFIG.MAP_YLIM(2)-CONFIG.WALL_WIDTH; y4 = CONFIG.MAP_YLIM(2);
    wallVertex = [x1, y1;
                  x2, y2;
                  x3, y2;
                  x3, y3;
                  x2, y3;
                  x2, y2;
                  x1, y1;
                  x4, y1;
                  x4, y4;
                  x1, y4;
                  x1, y1];
    patch(wallVertex(:,1),wallVertex(:,2),[0,0,0]);
end

function PlotPath(path, c)
    plot(path(:,1),path(:,2),c);
end

function PlotObstacle(pose, length, width)
    recVertex = [-length/2.0, -width/2.0; 
                  length/2.0, -width/2.0; 
                  length/2.0, width/2.0; 
                  -length/2.0, width/2.0; 
                  -length/2.0, -width/2.0]; %��תƽ��֮ǰ�ľ��Σ���5*2�ľ���ֱ��ʾ���½Ƕ��㡢���½Ƕ��㡢���ϽǶ�������ϽǶ��㣨����ʱ�룩�Լ����½Ƕ��㣨�γɱպ����ߣ�
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    rec = recVertex*rotMatrix+[pose(1)*ones(5,1), pose(2)*ones(5,1)];
    patch(rec(:,1),rec(:,2),[0, 0, 0]);   
end

function PlotVehicle(pose, delta, DIST, VEHICLE)
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    wheelPosList = [0, -VEHICLE.TRACK/2.0;   %�Һ���
                    0, VEHICLE.TRACK/2.0;    %�����
                    VEHICLE.WB, -VEHICLE.TRACK/2.0; %��ǰ��
                    VEHICLE.WB, VEHICLE.TRACK/2.0];  %��ǰ��
    wheelPosList = wheelPosList*rotMatrix+[pose(1)*ones(4,1), pose(2)*ones(4,1)];
    body = [pose(1)+DIST*cos(pose(3)), pose(2)+DIST*sin(pose(3)), pose(3)];
    PlotRectangle(body, VEHICLE.LENGTH, VEHICLE.WIDTH); %���Ƴ���
    hold on;
    PlotRectangle([wheelPosList(1,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %�����Һ���
    hold on;
    PlotRectangle([wheelPosList(2,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %���������
    hold on;
    PlotRectangle([wheelPosList(3,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %������ǰ��
    hold on;
    PlotRectangle([wheelPosList(4,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %������ǰ��
end

function PlotRectangle(pose, length, width)
%PLOTRECTANGLE ���ƾ���
%   pose��1*3���󣬲��������Ǽ������ĵĺ��������ƫ����
%   length��width�Ǿ���ĳ��Ϳ�
    recVertex = [-length/2.0, -width/2.0; length/2.0, -width/2.0; length/2.0, width/2.0; -length/2.0, width/2.0; -length/2.0, -width/2.0]; %��תƽ��֮ǰ�ľ��Σ���5*2�ľ���ֱ��ʾ���½Ƕ��㡢���½Ƕ��㡢���ϽǶ�������ϽǶ��㣨����ʱ�룩�Լ����½Ƕ��㣨�γɱպ����ߣ�
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    rec = recVertex*rotMatrix+[pose(1)*ones(5,1), pose(2)*ones(5,1)];
    plot(rec(:,1),rec(:,2),'color',[0,0,1]);
end

function EraseVehicle(pose, delta, DIST, VEHICLE)
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    wheelPosList = [0, -VEHICLE.TRACK/2.0;   %�Һ���
                    0, VEHICLE.TRACK/2.0;    %�����
                    VEHICLE.WB, -VEHICLE.TRACK/2.0; %��ǰ��
                    VEHICLE.WB, VEHICLE.TRACK/2.0];  %��ǰ��
    wheelPosList = wheelPosList*rotMatrix+[pose(1)*ones(4,1), pose(2)*ones(4,1)];
    body = [pose(1)+DIST*cos(pose(3)), pose(2)+DIST*sin(pose(3)), pose(3)];
    hold on;
    EraseRectangle(body, VEHICLE.LENGTH, VEHICLE.WIDTH); %���Ƴ���
    hold on;
    EraseRectangle([wheelPosList(1,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %�����Һ���
    hold on;
    EraseRectangle([wheelPosList(2,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %���������
    hold on;
    EraseRectangle([wheelPosList(3,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %������ǰ��
    hold on;
    EraseRectangle([wheelPosList(4,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %������ǰ��
    hold on;
end

function EraseRectangle(pose, length, width)
%PLOTRECTANGLE ��������
%   pose��1*3���󣬲��������Ǽ������ĵĺ��������ƫ����
%   length��width�Ǿ���ĳ��Ϳ�
    recVertex = [-length/2.0, -width/2.0; length/2.0, -width/2.0; length/2.0, width/2.0; -length/2.0, width/2.0; -length/2.0, -width/2.0]; %��תƽ��֮ǰ�ľ��Σ���5*2�ľ���ֱ��ʾ���½Ƕ��㡢���½Ƕ��㡢���ϽǶ�������ϽǶ��㣨����ʱ�룩�Լ����½Ƕ��㣨�γɱպ����ߣ�
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    rec = recVertex*rotMatrix+[pose(1)*ones(5,1), pose(2)*ones(5,1)];
    plot(rec(:,1),rec(:,2),'w');
end

