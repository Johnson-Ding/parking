function [ path, EXITFLAG ] = HybridAStar(obstacleList, costMap, ego, VEHICLE, CONFIG)
%ASTAR ��A*�����㷨�ҵ���ʼ�㵽Ŀ��������·��
%   map��robotics.BinaryOccupancyGrid���͵ı������ǻ���������ͼ
%   �������ϰ�����Ϣ����ͨ��getOccupancy������֪ĳλ���Ƿ�ռ��
%   obstacleList���ϰ��Ҳ��������ͣ�ŵĳ�������λ����Ϣ�������Ǽ������ĵĺ����ꡢ������ͳ���ƫ���ǣ�ƫ�����û����Ʊ�ʾ
%   ego������������Ϣ��struct���������а���length, width, start, goal��4���ֶ�
%   ����length��width�ֱ��Ӧ�����ĳ��Ϳ�
%   start����ʼ�����꣬��1*4�ľ���Ԫ������Ϊ�����ꡢ�����ꡢƫ���ǡ��ٶ�
%   goal��Ŀ������꣬��1*4�ľ���Ԫ������Ϊ�����ꡢ�����ꡢƫ���ǡ��ٶ�
%   CONFIG����������
%   VEHICLE����������

    EXITFLAG = 0;
    openList = containers.Map('KeyType','uint64','ValueType','any');
    closeList = containers.Map('KeyType','uint64','ValueType','any');
    costIndexList = []; %����index��ӦopenList��closeList���index����uint64���͵ı�������cost��double���͵ı���
    costList = [];      %�޷��洢��ͬһ�������У����������������ֱ�洢
    motionList = GenerateMotionList(VEHICLE);
    startPose = ego.start(1:3); 
    startDelta = ego.start(4);
    startPosIndex = CalcPosIndex(CONFIG, startPose(1:2));
    startYawIndex = CalcYawIndex(startPose(3), CONFIG.MIN_YAW, CONFIG.YAW_RESOLUTION);
    startNode = Node([], [], [], startPose, startDelta, CalcIndex(startPosIndex, startYawIndex), -10000, startPosIndex, startYawIndex, 0, CalcHCost(CONFIG, costMap, startPose(1:2)));
    openList(startNode.index) = startNode;
    costIndexList = Enqueue(costIndexList, startNode.index);
    costList = Enqueue(costList, startNode.cost);
    while openList.Count ~= 0
        [costIndex, costList] = Dequeue(costList);
        currentIndex = costIndexList(costIndex);
        costIndexList(costIndex) = [];  %�൱��Dequeue
        currentNode = openList(currentIndex);
%         disp(currentNode.pose);
        % RSpath�Ѿ���ɢ����
%         tic
        [RSpath, RSEXITFLAG] = AnalysticExpantion(currentNode.pose, ego.goal(1:3), VEHICLE, CONFIG, obstacleList);
%         toc
        if RSEXITFLAG == 1
            EXITFLAG = 1;
            if currentNode.hCost ~= 0
                openList.remove(currentIndex);
                closeList(currentIndex) = currentNode;
            end
%             tic
            path = GetPath(startNode, currentNode, closeList, RSpath);
%             toc
            break;
        else
            openList.remove(currentIndex);
            closeList(currentIndex) = currentNode;
%             tic
            [openList, closeList, costList, costIndexList] = ExpandOpenList(obstacleList, costMap, costList, costIndexList, openList, closeList, motionList, currentNode, ego.goal, VEHICLE, CONFIG);
%             toc
        end
    end    
    if EXITFLAG == 0
        path = [];
    end
end

function motionList = GenerateMotionList(VEHICLE)
    maxDelta = atan2(VEHICLE.WB, VEHICLE.MIN_TURNING_RADIUS);
    deltaResolution = maxDelta/2;
    deltaList = -maxDelta:deltaResolution:maxDelta;
    direction = ones(1, length(deltaList));
    direction = [direction, -ones(1, length(deltaList))];
    deltaList = [deltaList, deltaList];    
    motionList = [direction', deltaList'];
end

function node = Node(xList, yList, psiList, pose, delta, index, parent, posIndex, yawIndex, gCost, hCost)
    node.xList = xList;
    node.yList = yList;
    node.psiList = psiList;
    node.pose = pose;
    node.delta = delta;
    node.index = index;
    node.parent = parent;
    node.posIndex = posIndex;
    node.yawIndex = yawIndex;
    node.gCost = gCost;
    node.hCost = hCost;
    node.cost = gCost+hCost;
end

function posIndex = CalcPosIndex(CONFIG, pos)
    posIndex = round(((round(pos(1)*100)/100.0-CONFIG.MAP_XLIM(1))+(CONFIG.MAP_XLIM(2)-CONFIG.MAP_XLIM(1))*CONFIG.MAP_RESOLUTION*(round(pos(2)*100)/100.0-CONFIG.MAP_YLIM(1)))*CONFIG.MAP_RESOLUTION);
end

function yawIndex = CalcYawIndex(yaw, minYaw, yawResolution)
    yawIndex = ceil((yaw-minYaw)/yawResolution);
end

function index = CalcIndex(posIndex, yawIndex)
    index = uint64(posIndex+yawIndex*1000*1000*10);
end

function hCost = CalcHCost(CONFIG, costMap, pos)
    epsilon = 0.00001;
    x = round(pos(1)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
    y = round(pos(2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
    if abs(x-pos(1)) > epsilon
        x = floor(pos(1)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
    end
    if abs(y-pos(2)) > epsilon
        y = floor(pos(2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
    end
    posIndex = round(((x-CONFIG.MAP_XLIM(1))+(CONFIG.MAP_XLIM(2)-CONFIG.MAP_XLIM(1))*CONFIG.GRID_RESOLUTION*(y-CONFIG.MAP_YLIM(1)))*CONFIG.GRID_RESOLUTION);
    hCost = costMap(posIndex).cost;
end

function headingCost = CalcHeadingCost(currentPos, nextPos, goalPos)
%CALCHEADINGCOST �ô�����Ϊ�˱�ʾ����ǰ���������յ����ڷ����ƫ��
%   currentPos, nextPos, goalPos����1*2����
    vector1 = nextPos-currentPos;
    vector2 = goalPos-currentPos;
    theta = acos((vector1*vector2')/(norm(vector1)*norm(vector2)));
    headingCost = theta/pi;
end

function q = Enqueue(q, ele)
    q = [q; ele];
end

function [index, q] = Dequeue(q)
%DEQUEUE ��q���ҵ���Сcost��Ӧ��index�������index�����е�index
    minCostIndexList = find(q==min(q)); %����q����Сcost�����ж��
    index = minCostIndexList(1);
    q(index) = [];
end

function path = GetPath(startNode, endNode, closeList, RSpath)
    n = closeList.Count;
    path = inf(n*5, 4);
    i = 1;
    startIndex = startNode.index;
    current = endNode.index;
    currentNode = closeList(current);
    while current ~= startIndex
        if ~isempty(currentNode.xList)
            listLength = length(currentNode.xList);
            for j = 1:1:listLength
                path(i+j-1,:) = [currentNode.xList(j), currentNode.yList(j), currentNode.psiList(j), currentNode.delta];
            end
            i = i+listLength;
        else
            path(i,:) = [currentNode.pose, currentNode.delta];
            i = i+1;
        end
        current = currentNode.parent;
        currentNode = closeList(current);
    end
    path(i,:) = [currentNode.pose, currentNode.delta];
    while path(end,1) == inf
        path(end,:) = [];
    end
    path = flipud(path);
    path = [path; RSpath];
end

function [path, EXITFLAG] = AnalysticExpantion(start, goal, VEHICLE, CONFIG, obstacleList)
%ANALYSTICEXPANTION ������չ���ڸ���start��goal֮�����Reed-Shepp���ߣ�
%�����ҵ�������Reeds-Shepp��������·������
%   start��goal����1*3�ľ���Ԫ�������Ǻ����ꡢ�������ƫ���ǣ�ƫ�����û����Ʊ�ʾ
%   map��robotics.BinaryOccupancyGrid��������ȫ�ֵ�ͼ
    minR = VEHICLE.MIN_TURNING_RADIUS;
% �任����ϵ��ԭ����start��goal������ȫ������ϵ�±�ʾ�ģ���
% ���½�goal�任����startΪԭ�������ϵ��
    deltaPose = goal-start;
    psi = mod2pi(start(3));
    cpsi = cos(psi);
    spsi = sin(psi);
    pose = deltaPose;
    pose(1) = (deltaPose(1)*cpsi+deltaPose(2)*spsi)/minR;
    pose(2) = (deltaPose(1)*(-spsi)+deltaPose(2)*cpsi)/minR;
% Ѱ�Ҵӵ�ǰ�㵽Ŀ����������ײ��Reeds-Shepp�켣
%     tic
    [RSpath, RSEXITFLAG] = hybrid_a_star.FindRSPath(pose);
%     toc
% �����Ǹ���·����ͳ����˶�ѧģ�ͼ���λ�ã�����Ƿ�������ײ��
    if ~RSEXITFLAG
        path = [];
        EXITFLAG = false;
        return;
    else
        EXITFLAG = true;
    end
    maxDelta = atan2(VEHICLE.WB, VEHICLE.MIN_TURNING_RADIUS);
    motionResolution = CONFIG.MOTION_RESOLUTION;
    types = RSpath.type;
    t = minR*RSpath.t;
    u = minR*RSpath.u;
    v = minR*RSpath.v;
    w = minR*RSpath.w;
    x = minR*RSpath.x;
    segments = [t, u, v, w, x];
    px =start(1);
    py = start(2);
    psi = start(3);
    interPointIndex = 1;
    n = ceil(RSpath.totalLength*minR/motionResolution)+5;
    path = inf(n,4);
    
    for i = 1:5
        if segments(i) == 0
            continue;
        end
        % ���ݳ�����2*3���˶�����(ǰ��2�֣�ת��3��)������dist��delta
        dist = sign(segments(i))*motionResolution; % �ֱ��ʵ�����
        if types(i) == 'S'
            delta = 0;
            odo = 0;
            while abs(odo) < abs(segments(i))
                path(interPointIndex, 1) = px+odo*cos(psi);
                path(interPointIndex, 2) = py+odo*sin(psi);
                path(interPointIndex, 3) = psi;
                path(interPointIndex, 4) = delta;
                if mod(interPointIndex,5) == 0
                    isCollision = hybrid_a_star.CheckCollision([path(interPointIndex, 1),path(interPointIndex, 2),path(interPointIndex, 3)], obstacleList, CONFIG, VEHICLE);
                    if isCollision
                        EXITFLAG = false;
                        path = [];
                        return; % ���·��������ײ����������Reeds-Shepp·��
                    end
                end
                interPointIndex = interPointIndex+1;
                odo = odo+dist;
            end
            path(interPointIndex, 1) = px+segments(i)*cos(psi);
            path(interPointIndex, 2) = py+segments(i)*sin(psi);
            path(interPointIndex, 3) = psi;
            path(interPointIndex, 4) = delta;
            isCollision = hybrid_a_star.CheckCollision([path(interPointIndex, 1),path(interPointIndex, 2),path(interPointIndex, 3)], obstacleList, CONFIG, VEHICLE);
        elseif types(i) == 'L'
            delta = maxDelta;
            odo = 0;
            while abs(odo) < abs(segments(i))
                theta = odo/minR;
                chordLength = 2*minR*sin(theta/2);
                path(interPointIndex, 1) = px+chordLength*cos(psi+theta/2);
                path(interPointIndex, 2) = py+chordLength*sin(psi+theta/2);
                path(interPointIndex, 3) = psi+theta;
                path(interPointIndex, 4) = delta;
                if mod(interPointIndex,5) == 0
                    isCollision = hybrid_a_star.CheckCollision([path(interPointIndex, 1),path(interPointIndex, 2),path(interPointIndex, 3)], obstacleList, CONFIG, VEHICLE);
                    if isCollision
                        EXITFLAG = false;
                        path = [];
                        return; % ���·��������ײ����������Reeds-Shepp·��
                    end
                end
                interPointIndex = interPointIndex+1;
                odo = odo+dist;
            end
            theta = segments(i)/minR;
            chordLength = 2*minR*sin(theta/2);
            path(interPointIndex, 1) = px+chordLength*cos(psi+theta/2);
            path(interPointIndex, 2) = py+chordLength*sin(psi+theta/2);
            path(interPointIndex, 3) = psi+theta;
            path(interPointIndex, 4) = delta;
            isCollision = hybrid_a_star.CheckCollision([path(interPointIndex, 1),path(interPointIndex, 2),path(interPointIndex, 3)], obstacleList, CONFIG, VEHICLE);
        elseif types(i) == 'R'
            delta = -maxDelta;
            odo = 0;
            while abs(odo) < abs(segments(i))
                theta = odo/minR;
                chordLength = 2*minR*sin(theta/2);
                path(interPointIndex, 1) = px+chordLength*cos(psi-theta/2);
                path(interPointIndex, 2) = py+chordLength*sin(psi-theta/2);
                path(interPointIndex, 3) = psi-theta;
                path(interPointIndex, 4) = delta;
                if mod(interPointIndex,5) == 0
                    isCollision = hybrid_a_star.CheckCollision([path(interPointIndex, 1),path(interPointIndex, 2),path(interPointIndex, 3)], obstacleList, CONFIG, VEHICLE);
                    if isCollision
                        EXITFLAG = false;
                        path = [];
                        return; % ���·��������ײ����������Reeds-Shepp·��
                    end
                end
                interPointIndex = interPointIndex+1;
                odo = odo+dist;
            end
            theta = segments(i)/minR;
            chordLength = 2*minR*sin(theta/2);
            path(interPointIndex, 1) = px+chordLength*cos(psi-theta/2);
            path(interPointIndex, 2) = py+chordLength*sin(psi-theta/2);
            path(interPointIndex, 3) = psi-theta;
            path(interPointIndex, 4) = delta;
            isCollision = hybrid_a_star.CheckCollision([path(interPointIndex, 1),path(interPointIndex, 2),path(interPointIndex, 3)], obstacleList, CONFIG, VEHICLE);
        else
            continue;
        end
        if isCollision
            EXITFLAG = false;
            path = [];
            return; % ���·��������ײ����������Reeds-Shepp·��
        end
        px = path(interPointIndex, 1);
        py = path(interPointIndex, 2);
        psi = path(interPointIndex, 3);
        interPointIndex = interPointIndex+1;
    end
    while path(end, 1) == inf
        path(end,:) = [];
    end    
end

function [openList, closeList, costList, costIndexList] = ExpandOpenList(obstacleList, costMap, costList, costIndexList, openList, closeList, motionList, currentNode, goal, VEHICLE, CONFIG)
    dist = 1/CONFIG.GRID_RESOLUTION;
    interPointN = 5;
    currentPose = currentNode.pose;
    currentDelta = currentNode.delta;
    [n, ~] = size(motionList);
    for i = 1:1:n
        direction = motionList(i,1);
        delta = motionList(i,2);
        [xList,yList,psiList] = VehicleDynamic(interPointN, currentPose(1),currentPose(2),currentPose(3),direction*dist,delta,VEHICLE.WB);     
        x = xList(1);
        y = yList(1);
        psi = psiList(1);
        if hybrid_a_star.CheckCollision([x,y,psi], obstacleList, CONFIG, VEHICLE)
            continue;
        else
            if direction < 0
                motionCost = CONFIG.BACKWARD_COST*dist;
            else
                motionCost = dist;
            end
            steerCost = CONFIG.STEER_COST*abs(delta)+CONFIG.STEER_CHANGE_COST*abs(delta-currentDelta);
            headingCost = CalcHeadingCost(currentPose(1:2), [x, y], goal(1:2));
            tg = currentNode.gCost+motionCost+steerCost;
            neighborPosIndex = CalcPosIndex(CONFIG, [x,y]);
            neighborYawIndex = CalcYawIndex(psi, CONFIG.MIN_YAW, CONFIG.YAW_RESOLUTION);
            neighborIndex = CalcIndex(neighborPosIndex, neighborYawIndex);
            neighborHCost = CalcHCost(CONFIG, costMap, [x,y])+headingCost*2;
            if closeList.isKey(neighborIndex) && tg >= closeList(neighborIndex).gCost
                    continue;
            else
                if openList.isKey(neighborIndex)
                    if tg < openList(neighborIndex).gCost
                        neighborNode = openList(neighborIndex);
                        neighborNode.hCost = neighborHCost;
                        neighborNode.gCost = tg;
                        neighborNode.parent = currentNode.index;
                        neighborNode.cost = tg + neighborNode.hCost;
                        openList(neighborIndex) = neighborNode;
                        neighborInCostIndexList = costIndexList == neighborIndex;    %���ﲻ������������ͬ��index��costIndexList��
                        costList(neighborInCostIndexList) = neighborNode.cost;
                        if closeList.isKey(neighborIndex)
                            closeList(neighborIndex) = neighborNode;
                        end                     
                    end
                else
                    neighborNode = Node(xList, yList, psiList, [x, y, psi], delta, neighborIndex, currentNode.index, neighborPosIndex, neighborYawIndex, tg, neighborHCost);
                    openList(neighborNode.index) = neighborNode;
                    costList = Enqueue(costList, neighborNode.cost);
                    costIndexList = Enqueue(costIndexList, neighborIndex);
                end
            end
        end
    end
end

function a = mod2pi(a)
    a = rem(a,2*pi); % ������x/2pi������
    if a < -pi
        a = a+2*pi;
    elseif a > pi
        a = a-2*pi;
    end
end

function [xList,yList,psiList] = VehicleDynamic(n,x,y,psi,dist,delta,L)
    xList = zeros(n, 1);
    yList = zeros(n, 1);
    psiList = zeros(n, 1);
    for i = 1:1:n
        xList(i) = round((x+dist*(n-i+1)*cos(delta))*100)/100; % �˶�ѧ��ʽ�� x_dot = v_x * cos(theta); x_dot * t = v_x * t * cos(theta),�ڲ���ʱ��t��,����x = x + v_x * t * cos(theta)������v_x * t=D
        yList(i) = round((y+dist*(n-i+1)*sin(delta))*100)/100; % �˶�ѧ��ʽ
        psiList(i) = psi+dist*(n-i+1)/L*tan(delta); % L�����,����仯,theta_dot=v/R,R=L/tan(delta)
        psiList(i) = mod2pi(psiList(i));
    end
end
