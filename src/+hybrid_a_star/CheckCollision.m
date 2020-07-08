function isCollision = CheckCollision( pose, obstacleList, CONFIG, VEHICLE )
%CHECKCOLLISION ������λ��pose�£������Ƿ�����ϰ��﷢����ײ
%   pose������λ��
%   obstacleList���ϰ���λ��
%   CONFIG����������
%   VEHICLE����������
    if strcmp(CONFIG.MODE,'SIM')
        inflate = 0.01;
    elseif strcmp(CONFIG.MODE,'REAL')
        inflate = 0.001;
    end
    OFFSET = VEHICLE.LENGTH/2 - VEHICLE.LB;   %����ԭ�㣨���������ģ��������������ĵľ���
    boundingBox.length = VEHICLE.LENGTH*abs(cos(pose(3)))+VEHICLE.WIDTH*abs(sin(pose(3)));
    boundingBox.width = VEHICLE.LENGTH*abs(sin(pose(3)))+VEHICLE.WIDTH*abs(cos(pose(3)));
    boundingBox.x = pose(1)+OFFSET*cos(pose(3));
    boundingBox.y = pose(2)+OFFSET*sin(pose(3));
    egoPose = [boundingBox.x, boundingBox.y, pose(3)]; %ԭ��pose�е�x��y����������������Ӧ�ģ�����任���������ģ����ϰ����pose��ʾ��ʽ��ͳһ
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
%CHECKRECTANGLEINTERSECT �ж����������Ƿ��ཻ
%   rec1��rec2�ֱ��ʾ�������ε�λ�ˣ�����1*3�ľ���Ԫ�������Ǿ��μ������ĵĺ����ꡢ�����ꡢƫ����
%   length��width�ֱ��Ǿ��εĳ��Ϳ��������εĴ�С��ȫ��ͬ
    S = length*width;
    recVertex = [0, 0; -length/2.0, -width/2.0; length/2.0, -width/2.0; length/2.0, width/2.0; -length/2.0, width/2.0]; %��תƽ��֮ǰ�ľ��Σ���5*2�ľ���ֱ��ʾ�������ģ����½Ƕ��㡢���½Ƕ��㡢���ϽǶ�������ϽǶ��㣨����ʱ�룩
    rotMatrix1 = [cos(rec1(3)), sin(rec1(3)); -sin(rec1(3)), cos(rec1(3))];
    rotMatrix2 = [cos(rec2(3)), sin(rec2(3)); -sin(rec2(3)), cos(rec2(3))];
    recVertex1 = recVertex*rotMatrix1+[rec1(1)*ones(5,1), rec1(2)*ones(5,1)];
    recVertex2 = recVertex*rotMatrix2+[rec2(1)*ones(5,1), rec2(2)*ones(5,1)];
    % �����Ǽ�����1�������2������������������Ƿ��ھ���2��
    [index1, index2] = FindNearestVertexs(recVertex1, recVertex2(1,:));
    vertex1 = recVertex1(index1,:); vertex2 = recVertex1(index2,:);
    if CheckInRec(vertex1, recVertex2, S) || CheckInRec(vertex2, recVertex2, S)
        isCollision = true;
        return;
    else
        %  �����Ǽ�����2�������1������������������Ƿ��ھ���1��
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
%FINDNEARESTVERTEX �ҵ����������������������������������
%   recVertex��ʾ���ε����ĺ��ĸ���������꣬ center����һ�����ε�����
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
%CALCTRIANGLEAREA ��֪�������������������
%   p1,p2,p3�ֱ�����������ĺ������꣬��1*2�ľ����ʾ

    % ��ʽ�ɸ��ݲ�˼��㹫ʽ�Ƶ�
    m = [p1, 1; p2, 1; p3, 1];
    s = abs(det(m))/2;
end