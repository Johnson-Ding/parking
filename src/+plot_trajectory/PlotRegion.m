function PlotRegion(obstacleList, state, input, num, VEHICLE, CONFIG, imgPath, figNum, figTitle, T)
%PLOTMOVINGREGIONS 显示车辆运动过程中扫过的区域
%   obstacleList是障碍物的位置列表，是m*3的矩阵，第一列到第三列依次为横坐标、纵坐标、偏航角
%   path0是车辆运动路径，是n*5的矩阵，第一列到第五列依次为横坐标、纵坐标、偏航角、速度、前轮转角
%   input是n*2的矩阵，第一列是加速度a，第二列是转向角角速度omega
%   num是两个配置点之间插值点的个数
%   VEHICLE是struct变量，包含车辆的各种参数
%   CONFIG是struct变量，包含环境配置的各种参数
    DIST = VEHICLE.LENGTH/2 - VEHICLE.LB;   %坐标原点（后轮轴中心）到车辆几何中心的距离
    [n,~] = size(input);
    deltaT = T/num;
    path = [];
    f = @(z,u)[z(4)*cos(z(3)), z(4)*sin(z(3)), z(4)*tan(z(5))/VEHICLE.WB, u(1), u(2)];


    for i = 1:1:n-1
        path = [path;state(i,1:3)];
        for j = 1:1:num-1
            k1 = f(state(i,:),        input(i,:));
            k2 = f(state(i,:)+deltaT*j/2*k1, input(i,:));
            k3 = f(state(i,:)+deltaT*j/2*k2, input(i,:));
            k4 = f(state(i,:)+deltaT*j*k3,   input(i,:));
            z = state(i,:) + deltaT*j/6*(k1+2*k2+2*k3+k4); 
            path = [path; z(1:3)];
        end
    end
    path = [path; state(n,1:3)];
    
    figure(figNum);
    title(figTitle);
    set(gcf,'color','w')
    set(gca,'XTick',[0,5,10,15,20],'color','w');
    set(gca,'YTick',[0,3,6,9,12,15],'color','w');
    grid on
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
    timeInterval = deltaT;
    for i = 1:1:pathLength
        PlotVehicle(path(i,1:3), DIST, VEHICLE);
        PlotPath(path,'r');
        PlotPath(path(1:i,:),'g');
        hold on;
        pause(timeInterval);
        photo = getframe(gcf);
        imind = frame2im(photo);
        [imind,cm] = rgb2ind(imind,256);
        if i == 1
            imwrite(imind,cm,strcat(imgPath, figTitle,'(region).gif'),'GIF', 'Loopcount',inf,'DelayTime',timeInterval);
        else
            imwrite(imind,cm,strcat(imgPath, figTitle,'(region).gif'),'GIF','WriteMode','append','DelayTime',timeInterval);
        end 
%         EraseVehicle(path(i,1:3), path(i,4), DIST, VEHICLE);
        PlotMap(CONFIG);
        for j = 1:1:obstacleListLength
            PlotObstacle(obstacleList(j,:), VEHICLE.LENGTH, VEHICLE.WIDTH);
            hold on;
        end
    end
    PlotVehicle(path(i,1:3), DIST, VEHICLE);
    PlotPath(path,'r');
    PlotPath(path(1:i,:),'g');
    hold on;
    pause(1);
    photo = getframe(gcf);
    imind = frame2im(photo);
    [imind,cm] = rgb2ind(imind,256);
    if i == 1
        imwrite(imind,cm,strcat(imgPath, figTitle,'(region).gif'),'GIF', 'Loopcount',inf,'DelayTime',1);
    else
        imwrite(imind,cm,strcat(imgPath, figTitle,'(region).gif'),'GIF','WriteMode','append','DelayTime',1);
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

function PlotVehicle(pose, DIST, VEHICLE)
    body = [pose(1)+DIST*cos(pose(3)), pose(2)+DIST*sin(pose(3)), pose(3)];
    length = VEHICLE.LENGTH; width = VEHICLE.WIDTH;
    recVertex = [-length/2.0, -width/2.0; 
                  length/2.0, -width/2.0; 
                  length/2.0, width/2.0; 
                  -length/2.0, width/2.0; 
                  -length/2.0, -width/2.0]; %旋转平移之前的矩形，用5*2的矩阵分别表示左下角顶点、右下角顶点、右上角顶点和左上角顶点（即逆时针）以及左下角顶点（形成闭合曲线）
    rotMatrix = [cos(body(3)), sin(body(3)); -sin(body(3)), cos(body(3))];
    rec = recVertex*rotMatrix+[body(1)*ones(5,1), body(2)*ones(5,1)];
    patch(rec(:,1),rec(:,2),[0, 0, 1],'EdgeColor','b');   
end
