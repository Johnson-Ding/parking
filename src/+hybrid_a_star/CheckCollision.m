function isCollision = CheckCollision( pose, obstacleList, CONFIG, VEHICLE )
%CHECKCOLLISION 检测给定位姿pose下，车辆是否会与障碍物发生碰撞
%   pose：车辆位姿
%   obstacleList：障碍物位姿
%   CONFIG：环境参数
%   VEHICLE：车辆参数
    if strcmp(CONFIG.MODE,'SIM')
        inflate = 0.01;
    elseif strcmp(CONFIG.MODE,'REAL')
        inflate = 0.001;
    end
    OFFSET = VEHICLE.LENGTH/2 - VEHICLE.LB;   %坐标原点（后轮轴中心）到车辆几何中心的距离
    boundingBox.length = VEHICLE.LENGTH*abs(cos(pose(3)))+VEHICLE.WIDTH*abs(sin(pose(3)));
    boundingBox.width = VEHICLE.LENGTH*abs(sin(pose(3)))+VEHICLE.WIDTH*abs(cos(pose(3)));
    boundingBox.x = pose(1)+OFFSET*cos(pose(3));
    boundingBox.y = pose(2)+OFFSET*sin(pose(3));
    egoPose = [boundingBox.x, boundingBox.y, pose(3)]; %原本pose中的x和y是与后轮轴中心相对应的，这里变换到几何中心，与障碍物的pose表示方式相统一
    if ~CheckInMap(CONFIG, boundingBox)
        isCollision = true;
        return;
    end
    [n, ~] = size(obstacleList);
    isCollision = false;
    for i = 1:1:n
        isCollision = CheckRectangleIntersect(egoPose, obstacleList(i,:), VEHICLE.LENGTH, VEHICLE.WIDTH+inflate);
        if isCollision
            return;
        end
    end
end

function isInMap = CheckInMap(CONFIG, boundingBox)
    if boundingBox.x - boundingBox.length/2 > CONFIG.MAP_XLIM(1)+CONFIG.WALL_WIDTH && ...
            boundingBox.x + boundingBox.length/2 < CONFIG.MAP_XLIM(2)-CONFIG.WALL_WIDTH && ...
            boundingBox.y - boundingBox.width/2 > CONFIG.MAP_YLIM(1)+CONFIG.WALL_WIDTH && ...
            boundingBox.y + boundingBox.width/2 < CONFIG.MAP_YLIM(2)-CONFIG.WALL_WIDTH
        isInMap = true;
    else
        isInMap = false;
    end
end

function isCollision = CheckRectangleIntersect(rec1, rec2, length, width)
%CHECKRECTANGLEINTERSECT 判断两个矩形是否相交
%   rec1和rec2分别表示两个矩形的位姿，都是1*3的矩阵，元素依次是矩形几何中心的横坐标、纵坐标、偏航角
%   length和width分别是矩形的长和宽，两个矩形的大小完全相同
    S = length*width;
    recVertex = [0, 0; -length/2.0, -width/2.0; length/2.0, -width/2.0; length/2.0, width/2.0; -length/2.0, width/2.0]; %旋转平移之前的矩形，用5*2的矩阵分别表示几何中心，左下角顶点、右下角顶点、右上角顶点和左上角顶点（即逆时针）
    rotMatrix1 = [cos(rec1(3)), sin(rec1(3)); -sin(rec1(3)), cos(rec1(3))];
    rotMatrix2 = [cos(rec2(3)), sin(rec2(3)); -sin(rec2(3)), cos(rec2(3))];
    recVertex1 = recVertex*rotMatrix1+[rec1(1)*ones(5,1), rec1(2)*ones(5,1)];
    recVertex2 = recVertex*rotMatrix2+[rec2(1)*ones(5,1), rec2(2)*ones(5,1)];
    % 以下是检查矩形1距离矩形2中心最近的两个顶点是否在矩形2内
    [index1, index2] = FindNearestVertexs(recVertex1, recVertex2(1,:));
    vertex1 = recVertex1(index1,:); vertex2 = recVertex1(index2,:);
    if CheckInRec(vertex1, recVertex2, S) || CheckInRec(vertex2, recVertex2, S)
        isCollision = true;
        return;
    else
        %  以下是检查矩形2距离矩形1中心最近的两个顶点是否在矩形1内
        [index1, index2] = FindNearestVertexs(recVertex2, recVertex1(1,:));
        vertex1 = recVertex2(index1,:); vertex2 = recVertex2(index2,:);
        if CheckInRec(vertex1, recVertex1, S) || CheckInRec(vertex2, recVertex1, S)
            isCollision = true;
            return;
        else
            isCollision = false;
            return;
        end
    end
end

function [index1, index2] = FindNearestVertexs(recVertex, center)
%FINDNEARESTVERTEX 找到距离给定点最近的两个顶点的索引及坐标
%   recVertex表示矩形的中心和四个顶点的坐标， center是另一个矩形的中心
    dist1 = norm(recVertex(2,:)-center); 
    dist2 = norm(recVertex(3,:)-center);
    dist3 = norm(recVertex(4,:)-center);
    dist4 = norm(recVertex(5,:)-center);
    distList = [inf, dist1, dist2, dist3, dist4];
    [~, index1] = min(distList);
    distList(index1) = inf;
    [~, index2] = min(distList);   
end

function isInRec = CheckInRec(vertex, recVertex, s)
    epsilon = 0.00001;
    s1 = CalcTriangleArea(vertex, recVertex(2,:), recVertex(3,:));
    s2 = CalcTriangleArea(vertex, recVertex(3,:), recVertex(4,:));
    s3 = CalcTriangleArea(vertex, recVertex(4,:), recVertex(5,:));
    s4 = CalcTriangleArea(vertex, recVertex(5,:), recVertex(2,:));
    if s1+s2+s3+s4 > s + epsilon
        isInRec = false;
    else
        isInRec = true;
    end
end

function s = CalcTriangleArea(p1, p2, p3)
%CALCTRIANGLEAREA 已知三点坐标求三角形面积
%   p1,p2,p3分别是三个顶点的横纵坐标，用1*2的矩阵表示

    % 公式可根据叉乘计算公式推导
    m = [p1, 1; p2, 1; p3, 1];
    s = abs(det(m))/2;
end