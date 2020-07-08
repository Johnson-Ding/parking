function PlotTraj(obstacleList, path, VEHICLE, CONFIG, imgPath, figNum, figTitle, timeInterval)
%PLOTTRAJ 显示车辆运动轨迹
%   obstacleList是障碍物的位置列表，是m*3的矩阵，第一列到第三列依次为横坐标、纵坐标、偏航角
%   path是车辆运动路径，是n*4的矩阵，第一列到第四列依次为横坐标、纵坐标、偏航角、前轮转角
%   VEHICLE是struct变量，包含车辆的各种参数
%   CONFIG是struct变量，包含环境配置的各种参数
    
    DIST = VEHICLE.LENGTH/2 - VEHICLE.LB;   %坐标原点（后轮轴中心）到车辆几何中心的距离
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
                  -length/2.0, -width/2.0]; %旋转平移之前的矩形，用5*2的矩阵分别表示左下角顶点、右下角顶点、右上角顶点和左上角顶点（即逆时针）以及左下角顶点（形成闭合曲线）
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    rec = recVertex*rotMatrix+[pose(1)*ones(5,1), pose(2)*ones(5,1)];
    patch(rec(:,1),rec(:,2),[0, 0, 0]);   
end

function PlotVehicle(pose, delta, DIST, VEHICLE)
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    wheelPosList = [0, -VEHICLE.TRACK/2.0;   %右后轮
                    0, VEHICLE.TRACK/2.0;    %左后轮
                    VEHICLE.WB, -VEHICLE.TRACK/2.0; %右前轮
                    VEHICLE.WB, VEHICLE.TRACK/2.0];  %左前轮
    wheelPosList = wheelPosList*rotMatrix+[pose(1)*ones(4,1), pose(2)*ones(4,1)];
    body = [pose(1)+DIST*cos(pose(3)), pose(2)+DIST*sin(pose(3)), pose(3)];
    PlotRectangle(body, VEHICLE.LENGTH, VEHICLE.WIDTH); %绘制车身
    hold on;
    PlotRectangle([wheelPosList(1,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %绘制右后轮
    hold on;
    PlotRectangle([wheelPosList(2,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %绘制左后轮
    hold on;
    PlotRectangle([wheelPosList(3,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %绘制右前轮
    hold on;
    PlotRectangle([wheelPosList(4,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %绘制左前轮
end

function PlotRectangle(pose, length, width)
%PLOTRECTANGLE 绘制矩形
%   pose是1*3矩阵，参数依次是几何中心的横纵坐标和偏航角
%   length和width是矩阵的长和宽
    recVertex = [-length/2.0, -width/2.0; length/2.0, -width/2.0; length/2.0, width/2.0; -length/2.0, width/2.0; -length/2.0, -width/2.0]; %旋转平移之前的矩形，用5*2的矩阵分别表示左下角顶点、右下角顶点、右上角顶点和左上角顶点（即逆时针）以及左下角顶点（形成闭合曲线）
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    rec = recVertex*rotMatrix+[pose(1)*ones(5,1), pose(2)*ones(5,1)];
    plot(rec(:,1),rec(:,2),'color',[0,0,1]);
end

function EraseVehicle(pose, delta, DIST, VEHICLE)
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    wheelPosList = [0, -VEHICLE.TRACK/2.0;   %右后轮
                    0, VEHICLE.TRACK/2.0;    %左后轮
                    VEHICLE.WB, -VEHICLE.TRACK/2.0; %右前轮
                    VEHICLE.WB, VEHICLE.TRACK/2.0];  %左前轮
    wheelPosList = wheelPosList*rotMatrix+[pose(1)*ones(4,1), pose(2)*ones(4,1)];
    body = [pose(1)+DIST*cos(pose(3)), pose(2)+DIST*sin(pose(3)), pose(3)];
    hold on;
    EraseRectangle(body, VEHICLE.LENGTH, VEHICLE.WIDTH); %绘制车身
    hold on;
    EraseRectangle([wheelPosList(1,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %绘制右后轮
    hold on;
    EraseRectangle([wheelPosList(2,:),pose(3)], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);  %绘制左后轮
    hold on;
    EraseRectangle([wheelPosList(3,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %绘制右前轮
    hold on;
    EraseRectangle([wheelPosList(4,:),pose(3)+delta], VEHICLE.WHEEL_LENGTH, VEHICLE.WHEEL_WIDTH);    %绘制左前轮
    hold on;
end

function EraseRectangle(pose, length, width)
%PLOTRECTANGLE 擦除矩形
%   pose是1*3矩阵，参数依次是几何中心的横纵坐标和偏航角
%   length和width是矩阵的长和宽
    recVertex = [-length/2.0, -width/2.0; length/2.0, -width/2.0; length/2.0, width/2.0; -length/2.0, width/2.0; -length/2.0, -width/2.0]; %旋转平移之前的矩形，用5*2的矩阵分别表示左下角顶点、右下角顶点、右上角顶点和左上角顶点（即逆时针）以及左下角顶点（形成闭合曲线）
    rotMatrix = [cos(pose(3)), sin(pose(3)); -sin(pose(3)), cos(pose(3))];
    rec = recVertex*rotMatrix+[pose(1)*ones(5,1), pose(2)*ones(5,1)];
    plot(rec(:,1),rec(:,2),'w');
end

