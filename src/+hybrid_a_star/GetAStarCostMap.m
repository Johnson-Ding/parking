function [ costMap ] = GetAStarCostMap(map, obstacleList, ego, CONFIG,VEHICLE)
%GETASTARCOSTMAP 用A*搜索算法分别计算地图上各点到目标点的代价
%   map是robotics.BinaryOccupancyGrid类型的变量，是环境完整地图
%   其中有障碍物信息，可通过getOccupancy函数得知某位置是否被占据
%   obstacleList是障碍物（也即其它已停放的车辆）的位姿信息，依次是几何中心的横坐标、纵坐标和车辆偏航角，偏航角用弧度制表示
%   ego是描述本车信息的struct变量，其中包含length, width, start, goal共4个字段
%   其中length和width分别对应车辆的长和宽
%   start是起始点坐标，是1*4的矩阵，元素依次为横坐标、纵坐标、偏航角、速度
%   goal是目标点坐标，是1*4的矩阵，元素依次为横坐标、纵坐标、偏航角、速度
%   CONFIG：环境参数
%   VEHICLE：车辆参数
    cMap = copy(map);
    cMap = MyInflate(cMap, obstacleList, ego, ego.width/2, CONFIG,VEHICLE); %cMap表示膨胀后的binary occupancy map，对应C-Space
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
%MYINFLATE 自定义的膨胀函数，根据原来的地图以及膨胀半径，得到构型空间C-Space的地图
%   map是robotics.BinaryOccupancyGrid类型的变量，是环境完整地图
%   obstacleList是障碍物（也即其它已停放的车辆）的位姿信息，依次是几何中心的横坐标、纵坐标和车辆偏航角，偏航角用弧度制表示
%   ego是描述本车信息的struct变量，其中包含length, width, start, goal共4个字段
%   其中length和width分别对应车辆的长和宽
%   start是起始点坐标，是1*2的矩阵，元素依次为横坐标、纵坐标
%   goal是目标点坐标，是1*2的矩阵，元素依次为横坐标、纵坐标
%   radius是膨胀半径
%   cMap是输出，表示C-Space的地图
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
%NODE 搜索树节点的构造函数
%   pos是当前点的横纵坐标，用1*2的矩阵表示
%   gCost是当前点的实际代价，即起始点到当前点的实际代价
%   hCost是当前点的估计代价，即当前点到目标点的估计代价，用欧式距离估计
%   fCost是当前点的总代价，即实际代价+估计代价
%   index是当前点的索引，在搜索树中可以用索引直接访问该节点
%   parent是当前点的父节点，若当前节点是搜索树的第一个节点，则父节点是-1
%   函数返回的node是struct变量
    node.pos = pos;
    node.gCost = gCost;
    node.hCost = hCost;
    node.fCost = gCost+hCost;
    node.index = index;
    node.parent = parent;
end

function index = CalcIndex(CONFIG, pos)
%CALCINDEX 该函数将二维的地图坐标映射成一维的索引，这一步是为之后判断一个节点是否在openList和closeList中
%   map是robotics.BinaryOccupancyGrid，对应的是C-Space的地图
%   pos是当前点的坐标，用1*2的矩阵表示

%   index从0开始
    index = round(((round(pos(1)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION-CONFIG.MAP_XLIM(1))+(CONFIG.MAP_XLIM(2)-CONFIG.MAP_XLIM(1))*CONFIG.GRID_RESOLUTION*(round(pos(2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION-CONFIG.MAP_YLIM(1)))*CONFIG.GRID_RESOLUTION);
end

function dist = CalcHCost(current, goal)
%CALCHCOST 采用欧几里得距离对当前点到目标点的代价进行估计
%   current是当前点的坐标，用1*2的矩阵表示
%   goal是目标点的坐标，用1*2的矩阵表示
%   dist是两点间的欧氏距离
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
%DEQUEUE 在q中找到最小cost对应的index
    minCostIndexList = find(q(:,2)==min(q(:,2))); %队列q中最小cost可能有多个
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
%CHECKINMAP 判断给定坐标是否落在地图内
    epsilon = 0.000000001;
    if pos(1) < CONFIG.MAP_XLIM(1)-epsilon || pos(1) > CONFIG.MAP_XLIM(2)-1/CONFIG.GRID_RESOLUTION+epsilon
        flag = false;
    elseif pos(2) < CONFIG.MAP_YLIM(1)-epsilon || pos(2) > CONFIG.MAP_YLIM(2)-1/CONFIG.GRID_RESOLUTION+epsilon
        flag = false;
    else
        flag = true;
    end
end
