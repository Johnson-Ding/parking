function PlotRegion(obstacleList, state, input, num, VEHICLE, CONFIG, imgPath, figNum, figTitle, T)
%PLOTMOVINGREGIONS ��ʾ�����˶�������ɨ��������
%   obstacleList���ϰ����λ���б���m*3�ľ��󣬵�һ�е�����������Ϊ�����ꡢ�����ꡢƫ����
%   path0�ǳ����˶�·������n*5�ľ��󣬵�һ�е�����������Ϊ�����ꡢ�����ꡢƫ���ǡ��ٶȡ�ǰ��ת��
%   input��n*2�ľ��󣬵�һ���Ǽ��ٶ�a���ڶ�����ת��ǽ��ٶ�omega
%   num���������õ�֮���ֵ��ĸ���
%   VEHICLE��struct���������������ĸ��ֲ���
%   CONFIG��struct�����������������õĸ��ֲ���
    DIST = VEHICLE.LENGTH/2 - VEHICLE.LB;   %����ԭ�㣨���������ģ��������������ĵľ���
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
                  -length/2.0, -width/2.0]; %��תƽ��֮ǰ�ľ��Σ���5*2�ľ���ֱ��ʾ���½Ƕ��㡢���½Ƕ��㡢���ϽǶ�������ϽǶ��㣨����ʱ�룩�Լ����½Ƕ��㣨�γɱպ����ߣ�
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
                  -length/2.0, -width/2.0]; %��תƽ��֮ǰ�ľ��Σ���5*2�ľ���ֱ��ʾ���½Ƕ��㡢���½Ƕ��㡢���ϽǶ�������ϽǶ��㣨����ʱ�룩�Լ����½Ƕ��㣨�γɱպ����ߣ�
    rotMatrix = [cos(body(3)), sin(body(3)); -sin(body(3)), cos(body(3))];
    rec = recVertex*rotMatrix+[body(1)*ones(5,1), body(2)*ones(5,1)];
    patch(rec(:,1),rec(:,2),[0, 0, 1],'EdgeColor','b');   
end
