function [ costMap ] = GetAStarCostMap(map, obstacleList, ego, CONFIG,VEHICLE)
%GETASTARCOSTMAP ��A*�����㷨�ֱ�����ͼ�ϸ��㵽Ŀ���Ĵ���
%   map��robotics.BinaryOccupancyGrid���͵ı������ǻ���������ͼ
%   �������ϰ�����Ϣ����ͨ��getOccupancy������֪ĳλ���Ƿ�ռ��
%   obstacleList���ϰ��Ҳ��������ͣ�ŵĳ�������λ����Ϣ�������Ǽ������ĵĺ����ꡢ������ͳ���ƫ���ǣ�ƫ�����û����Ʊ�ʾ
%   ego������������Ϣ��struct���������а���length, width, start, goal��4���ֶ�
%   ����length��width�ֱ��Ӧ�����ĳ��Ϳ�
%   start����ʼ�����꣬��1*4�ľ���Ԫ������Ϊ�����ꡢ�����ꡢƫ���ǡ��ٶ�
%   goal��Ŀ������꣬��1*4�ľ���Ԫ������Ϊ�����ꡢ�����ꡢƫ���ǡ��ٶ�
%   CONFIG����������
%   VEHICLE����������
    cMap = copy(map);
    cMap = MyInflate(cMap, obstacleList, ego, ego.width/2, CONFIG,VEHICLE); %cMap��ʾ���ͺ��binary occupancy map����ӦC-Space
    costMap = containers.Map('KeyType','int32','ValueType','any');
    steps = 1.0/CONFIG.GRID_RESOLUTION;
    motionList = GenerateMotionList(steps);
    goalPos = ego.goal(1,1:2);
    maxm = round(max(goalPos(1)-CONFIG.MAP_XLIM(1), CONFIG.MAP_XLIM(2)-goalPos(1))*CONFIG.GRID_RESOLUTION);
    maxn = round(max(goalPos(2)-CONFIG.MAP_YLIM(1), CONFIG.MAP_YLIM(2)-goalPos(2))*CONFIG.GRID_RESOLUTION);
    goalIndex = CalcIndex(CONFIG, goalPos);
    costNode.index = goalIndex;
    costNode.pos = goalPos;
    costNode.cost = 0;
    costMap(goalIndex) = costNode;
    
    x = zeros(1,4); y = zeros(1,4);
    
    for i = 0:1:maxm
        for j = 0:1:maxn
            if i == 0 && j == 0
                continue;
            elseif i == 0
                x(1) = goalPos(1)+i*steps;
                y(1) = goalPos(2)+j*steps;
                x(2) = goalPos(1)+i*steps;
                y(2) = goalPos(2)-j*steps;  
                for k = 1:1:2
                    startPos = [round(x(k)*1000)/1000, round(y(k)*1000)/1000];
                    if CheckInMap(CONFIG, startPos)
                        startIndex = CalcIndex(CONFIG, startPos);
%                         disp(startPos);
%                         tic
                        if getOccupancy(cMap, startPos)
                            costNode.index = startIndex;
                            costNode.pos = startPos;
                            costNode.cost = inf;
                            costMap(startIndex) = costNode;
                        else
                            costMap = CalcAStarCost(cMap, costMap, startIndex, startPos, goalPos, motionList, CONFIG);
                        end
%                         toc
                    end
                end
                continue;
            end
            x(1) = goalPos(1)+i*steps;
            y(1) = goalPos(2)+j*steps;
            x(2) = goalPos(1)+i*steps;
            y(2) = goalPos(2)-j*steps;
            x(3) = goalPos(1)-i*steps;
            y(3) = goalPos(2)+j*steps;
            x(4) = goalPos(1)-i*steps;
            y(4) = goalPos(2)-j*steps;
            for k = 1:1:4
                startPos = [round(x(k)*1000)/1000, round(y(k)*1000)/1000];
                if CheckInMap(CONFIG, startPos)
                    startIndex = CalcIndex(CONFIG, startPos);
%                     disp(startPos);
%                     tic
                    if getOccupancy(cMap, startPos)
                        costNode.index = startIndex;
                        costNode.pos = startPos;
                        costNode.cost = inf;
                        costMap(startIndex) = costNode;
                    else
                        costMap = CalcAStarCost(cMap, costMap, startIndex, startPos, goalPos, motionList, CONFIG);
                    end
%                     toc
                end
            end
        end
    end
end

function [ costMap ] = CalcAStarCost(cMap, costMap, startIndex, startPos, goalPos, motionList, CONFIG)
    openList = containers.Map('KeyType','int32','ValueType','any');
    closeList = containers.Map('KeyType','int32','ValueType','any');
    startNode = Node(startPos, 0, CalcHCost(startPos, goalPos), startIndex, -100000);
    openList(startNode.index) = startNode;
    step = 1.0/CONFIG.GRID_RESOLUTION; epsilon = step;
    costList = [];
    costList = Enqueue(costList, startNode.index, startNode.fCost);
    EXITFLAG = 0;   costNode.index = startIndex;   costNode.pos = startPos;
    while openList.Count ~= 0
        [currentIndex, costList] = Dequeue(costList);
        currentNode = openList(currentIndex);
        if costMap.isKey(currentIndex) && costMap(currentIndex).cost < 100000
            costNode.cost = costMap(currentIndex).cost+currentNode.gCost;
            costMap(startIndex) = costNode;
            EXITFLAG = 1;
            break;
        end
        if currentNode.hCost < epsilon
            EXITFLAG = 1;
            costNode.cost = currentNode.gCost+currentNode.hCost;
            costMap(startIndex) = costNode;
            break;
        else
            openList.remove(currentIndex);
            closeList(currentIndex) = currentNode;
            [openList, closeList, costList] = ExpandOpenList(cMap, costList, openList, closeList, currentNode, motionList, goalPos, CONFIG);
        end     
    end
    if EXITFLAG == 0
        costNode.cost = inf;
        costMap(startIndex) = costNode;
    end
end

function cMap = MyInflate(map, obstacleList, ego, radius, CONFIG,VEHICLE)
%MYINFLATE �Զ�������ͺ���������ԭ���ĵ�ͼ�Լ����Ͱ뾶���õ����Ϳռ�C-Space�ĵ�ͼ
%   map��robotics.BinaryOccupancyGrid���͵ı������ǻ���������ͼ
%   obstacleList���ϰ��Ҳ��������ͣ�ŵĳ�������λ����Ϣ�������Ǽ������ĵĺ����ꡢ������ͳ���ƫ���ǣ�ƫ�����û����Ʊ�ʾ
%   ego������������Ϣ��struct���������а���length, width, start, goal��4���ֶ�
%   ����length��width�ֱ��Ӧ�����ĳ��Ϳ�
%   start����ʼ�����꣬��1*2�ľ���Ԫ������Ϊ�����ꡢ������
%   goal��Ŀ������꣬��1*2�ľ���Ԫ������Ϊ�����ꡢ������
%   radius�����Ͱ뾶
%   cMap���������ʾC-Space�ĵ�ͼ
    cMap = map;
    mapLength = CONFIG.MAP_XLIM(2)-CONFIG.MAP_XLIM(1);    
    mapWidth = CONFIG.MAP_YLIM(2)-CONFIG.MAP_YLIM(1);    
    mapResolution = CONFIG.MAP_RESOLUTION; 
    mapDelta = 1/(mapResolution*2);
    wallWidth = CONFIG.WALL_WIDTH;
    [wallX, wallY] = meshgrid(0:mapDelta:wallWidth, 0:mapDelta:mapWidth);
    setOccupancy(cMap, [wallX(:) wallY(:)], 1);
    [wallX, wallY] = meshgrid((mapLength-wallWidth):mapDelta:mapLength, 0:mapDelta:mapWidth);
    setOccupancy(cMap, [wallX(:) wallY(:)], 1);
    [wallX, wallY] = meshgrid(0:mapDelta:mapLength, 0:mapDelta:wallWidth);
    setOccupancy(cMap, [wallX(:) wallY(:)], 1);
    [wallX, wallY] = meshgrid(0:mapDelta:mapLength, (mapWidth-wallWidth):mapDelta:mapWidth);
    setOccupancy(cMap, [wallX(:) wallY(:)], 1);
    [row, ~] = size(obstacleList);
    for i = 1:1:row
        obstaclePose = obstacleList(i,:);
        if obstaclePose(3) == 0 || obstaclePose(3) == pi
            [obstacleX, obstacleY] = meshgrid((obstaclePose(1)-ego.length/2-VEHICLE.LB):mapDelta:(obstaclePose(1)+ego.length/2+VEHICLE.LB), (obstaclePose(2)-ego.width/2-radius):mapDelta:(obstaclePose(2)+ego.width/2)+radius);
        elseif obstaclePose(3) == pi/2 || obstaclePose(3) == -pi/2
            [obstacleX, obstacleY] = meshgrid((obstaclePose(1)-ego.width/2-radius):mapDelta:(obstaclePose(1)+ego.width/2+radius), (obstaclePose(2)-ego.length/2-radius):mapDelta:(obstaclePose(2)+ego.length/2)+radius);
        else
            [obstacleX, obstacleY] = meshgrid((-ego.length/2-radius):mapDelta:(ego.length/2+radius), (-ego.width/2-radius):mapDelta:(ego.width/2+radius));
            tempObstacleX = obstacleX;
            obstacleX = obstacleX.*cos(obstaclePose(3))-obstacleY.*sin(obstaclePose(3))+obstaclePose(1);
            obstacleY = tempObstacleX.*(sin(obstaclePose(3)))+obstacleY.*cos(obstaclePose(3))+obstaclePose(2);
        end
        obstacleX = obstacleX(:);   obstacleY = obstacleY(:);
        n = length(obstacleX);
        xlim = map.XWorldLimits;
        ylim = map.YWorldLimits;
        for j = 1:1:n
            if obstacleX(j) < xlim(1) || obstacleX(j) > xlim(2) || obstacleY(j) < ylim(1) || obstacleY(j) > ylim(2)
                obstacleX(j) = 0; obstacleY(j) = 0;
            end
        end
        setOccupancy(cMap, [obstacleX obstacleY], 1);
    end
end

function node = Node(pos, gCost, hCost, index, parent)
%NODE �������ڵ�Ĺ��캯��
%   pos�ǵ�ǰ��ĺ������꣬��1*2�ľ����ʾ
%   gCost�ǵ�ǰ���ʵ�ʴ��ۣ�����ʼ�㵽��ǰ���ʵ�ʴ���
%   hCost�ǵ�ǰ��Ĺ��ƴ��ۣ�����ǰ�㵽Ŀ���Ĺ��ƴ��ۣ���ŷʽ�������
%   fCost�ǵ�ǰ����ܴ��ۣ���ʵ�ʴ���+���ƴ���
%   index�ǵ�ǰ������������������п���������ֱ�ӷ��ʸýڵ�
%   parent�ǵ�ǰ��ĸ��ڵ㣬����ǰ�ڵ����������ĵ�һ���ڵ㣬�򸸽ڵ���-1
%   �������ص�node��struct����
    node.pos = pos;
    node.gCost = gCost;
    node.hCost = hCost;
    node.fCost = gCost+hCost;
    node.index = index;
    node.parent = parent;
end

function index = CalcIndex(CONFIG, pos)
%CALCINDEX �ú�������ά�ĵ�ͼ����ӳ���һά����������һ����Ϊ֮���ж�һ���ڵ��Ƿ���openList��closeList��
%   map��robotics.BinaryOccupancyGrid����Ӧ����C-Space�ĵ�ͼ
%   pos�ǵ�ǰ������꣬��1*2�ľ����ʾ

%   index��0��ʼ
    index = round(((round(pos(1)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION-CONFIG.MAP_XLIM(1))+(CONFIG.MAP_XLIM(2)-CONFIG.MAP_XLIM(1))*CONFIG.GRID_RESOLUTION*(round(pos(2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION-CONFIG.MAP_YLIM(1)))*CONFIG.GRID_RESOLUTION);
end

function dist = CalcHCost(current, goal)
%CALCHCOST ����ŷ����þ���Ե�ǰ�㵽Ŀ���Ĵ��۽��й���
%   current�ǵ�ǰ������꣬��1*2�ľ����ʾ
%   goal��Ŀ�������꣬��1*2�ľ����ʾ
%   dist��������ŷ�Ͼ���
    dist = norm(current-goal);
end

function motionList = GenerateMotionList(step)
%                  dx,  dy,  cost
    motionList = [step   0   step;
                  step  step step*sqrt(2);
                    0   step step;
                 -step  step step*sqrt(2);
                 -step   0   step;
                 -step -step step*sqrt(2);
                    0  -step step;
                  step -step step*sqrt(2)];
end

function q = Enqueue(q, index, cost)
    q = [q; index, cost];
end

function [index, q] = Dequeue(q)
%DEQUEUE ��q���ҵ���Сcost��Ӧ��index
    minCostIndexList = find(q(:,2)==min(q(:,2))); %����q����Сcost�����ж��
    index = q(minCostIndexList(1),1);
    q(minCostIndexList(1),:) = [];
end

function [openList, closeList, costList] = ExpandOpenList(map, costList, openList, closeList, currentNode, motionList, goalPos, CONFIG)
    [n, ~] = size(motionList);
    for i = 1:1:n
        x = currentNode.pos(1)+motionList(i,1); y = currentNode.pos(2)+motionList(i,2);
        if ~CheckInMap(CONFIG, [x, y])
            continue;
        end
        if getOccupancy(map, [x, y])
            continue;
        else
            tg = currentNode.gCost+motionList(i,3);
            neighborIndex = CalcIndex(CONFIG, [x, y]);
            if closeList.isKey(neighborIndex) && tg >= closeList(neighborIndex).gCost
                    continue;
            else
                if ~openList.isKey(neighborIndex)
                    neighborNode = Node([x, y], tg, CalcHCost([x, y], goalPos), neighborIndex, currentNode.index);
                    openList(neighborNode.index) = neighborNode;
                    costList = Enqueue(costList, neighborIndex, neighborNode.fCost);
                else
                    if tg < openList(neighborIndex).gCost
                        neighborNode = openList(neighborIndex);
%                         neighborNode.index = neighbor;
                        neighborNode.gCost = tg;
%                         neighborNode.hCost = CalcHCost([x, y], goalPos);
                        neighborNode.parent = currentNode.index;
                        neighborNode.fCost = tg + neighborNode.hCost;
                        openList(neighborIndex) = neighborNode;
                        neighborInCostList = costList(:,1) == neighborIndex;
                        costList(neighborInCostList, 2) = neighborNode.fCost;
                        if closeList.isKey(neighborIndex)
                            closeList(neighborIndex) = neighborNode;
                        end                     
                    end
                end
            end
        end
    end
end

function flag = CheckInMap(CONFIG, pos)
%CHECKINMAP �жϸ��������Ƿ����ڵ�ͼ��
    epsilon = 0.000000001;
    if pos(1) < CONFIG.MAP_XLIM(1)-epsilon || pos(1) > CONFIG.MAP_XLIM(2)-1/CONFIG.GRID_RESOLUTION+epsilon
        flag = false;
    elseif pos(2) < CONFIG.MAP_YLIM(1)-epsilon || pos(2) > CONFIG.MAP_YLIM(2)-1/CONFIG.GRID_RESOLUTION+epsilon
        flag = false;
    else
        flag = true;
    end
end
