function [ path, EXITFLAG ] = AStar(map, obstacleList, ego)
%ASTAR ��A*�����㷨�ҵ���ʼ�㵽Ŀ��������·��
%   map��robotics.BinaryOccupancyGrid���͵ı������ǻ���������ͼ
%   �������ϰ�����Ϣ����ͨ��getOccupancy������֪ĳλ���Ƿ�ռ��
%   obstacleList���ϰ��Ҳ��������ͣ�ŵĳ�������λ����Ϣ�������Ǽ������ĵĺ����ꡢ������ͳ���ƫ���ǣ�ƫ�����û����Ʊ�ʾ
%   ego������������Ϣ��struct���������а���length, width, start, goal��4���ֶ�
%   ����length��width�ֱ��Ӧ�����ĳ��Ϳ�
%   start����ʼ�����꣬��1*2�ľ���Ԫ������Ϊ�����ꡢ������
%   goal��Ŀ������꣬��1*2�ľ���Ԫ������Ϊ�����ꡢ������
    path = []; EXITFLAG = 0;
    cMap = copy(map);
    cMap = MyInflate(cMap, obstacleList, ego, ego.width/2); %cMap��ʾ���ͺ��binary occupancy map����ӦC-Space
%     show(cMap);
    step = 0.2; epsilon = 0.2;
    openList = containers.Map('KeyType','int32','ValueType','any');
    closeList = containers.Map('KeyType','int32','ValueType','any');
    motionList = GenerateMotionList(step);
    costList = [];
    startPos = ego.start(1,1:2); goalPos = ego.goal(1,1:2);
    startNode = Node(startPos, 0, CalcHCost(startPos, goalPos), CalcIndex(cMap, startPos), -100000);
    openList(startNode.index) = startNode;
    costList = Enqueue(costList, startNode.index, startNode.fCost);
    
    while openList.Count ~= 0
        [currentIndex, costList] = Dequeue(costList);
        currentNode = openList(currentIndex);
        if currentNode.hCost < epsilon
            disp('A Star succeeds');
            EXITFLAG = 1;
            if currentNode.hCost ~= 0
                openList.remove(currentIndex);
                closeList(currentIndex) = currentNode;
            end
            path = GetPath(closeList, startNode, currentNode);
            break;
        else
            openList.remove(currentIndex);
            closeList(currentIndex) = currentNode;
            [openList, closeList, costList] = ExpandOpenList(cMap, costList, openList, closeList, currentNode, motionList, goalPos);
        end     
    end
end

function cMap = MyInflate(map, obstacleList, ego, radius)
%MYINFLATE �Զ�������ͺ���������ԭ���ĵ�ͼ�Լ����Ͱ뾶���õ����Ϳռ�C-Space�ĵ�ͼ
%   map��robotics.BinaryOccupancyGrid���͵ı������ǻ���������ͼ
%   obstacleList���ϰ��Ҳ��������ͣ�ŵĳ�������λ����Ϣ�������Ǽ������ĵĺ����ꡢ������ͳ���ƫ���ǣ�ƫ�����û����Ʊ�ʾ
%   ego������������Ϣ��struct���������а���length, width, start, goal��4���ֶ�
%   ����length��width�ֱ��Ӧ�����ĳ��Ϳ�
%   start����ʼ�����꣬��1*2�ľ���Ԫ������Ϊ�����ꡢ������
%   goal��Ŀ������꣬��1*2�ľ���Ԫ������Ϊ�����ꡢ������
%   radius�����Ͱ뾶
%   cMap���������ʾC-Space�ĵ�ͼ
    mapDelta = 0.005;
    cMap = map;
    [row, ~] = size(obstacleList);
    for i = 1:1:row
        obstaclePose = obstacleList(i,:);
        if obstaclePose(3) == 0 || obstaclePose(3) == pi
            [obstacleX, obstacleY] = meshgrid((obstaclePose(1)-ego.length/2-radius):mapDelta:(obstaclePose(1)+ego.length/2+radius), (obstaclePose(2)-ego.width/2-radius):mapDelta:(obstaclePose(2)+ego.width/2)+radius);
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
        xlim = cMap.XWorldLimits;
        ylim = cMap.YWorldLimits;
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

function index = CalcIndex(map, pos)
%CALCINDEX �ú�������ά�ĵ�ͼ����ӳ���һά����������һ����Ϊ֮���ж�һ���ڵ��Ƿ���openList��closeList��
%   map��robotics.BinaryOccupancyGrid����Ӧ����C-Space�ĵ�ͼ
%   pos�ǵ�ǰ������꣬��1*2�ľ����ʾ
    xlim = map.XWorldLimits;
    ylim = map.YWorldLimits;
    index = ceil(((pos(1)-xlim(1))+(xlim(2)-xlim(1))*map.Resolution*(pos(2)-ylim(1)))*map.Resolution);
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

function [openList, closeList, costList] = ExpandOpenList(map, costList, openList, closeList, currentNode, motionList, goalPos)
    [n, ~] = size(motionList);
    for i = 1:1:n
        x = currentNode.pos(1)+motionList(i,1); y = currentNode.pos(2)+motionList(i,2);
        if ~CheckInMap(map, [x, y])
            continue;
        end
        if getOccupancy(map, [x, y])
            continue;
        else
            tg = currentNode.gCost+motionList(i,3);
            neighbor = CalcIndex(map, [x, y]);
            if closeList.isKey(neighbor) && tg >= closeList(neighbor).gCost
                    continue;
            else
                if ~openList.isKey(neighbor)
                    neighborNode = Node([x, y], tg, CalcHCost([x, y], goalPos), neighbor, currentNode.index);
                    openList(neighborNode.index) = neighborNode;
                    costList = Enqueue(costList, neighbor, neighborNode.fCost);
                else
                    if tg < openList(neighbor).gCost
                        neighborNode = openList(neighbor);
%                         neighborNode.index = neighbor;
                        neighborNode.gCost = tg;
%                         neighborNode.hCost = CalcHCost([x, y], goalPos);
                        neighborNode.parent = currentNode.index;
                        neighborNode.fCost = tg + neighborNode.hCost;
                        openList(neighbor) = neighborNode;
                        neighborInCostList = costList(:,1) == neighbor;
                        costList(neighborInCostList, 2) = neighborNode.fCost;
                        if closeList.isKey(neighbor)
                            closeList(neighbor) = neighborNode;
                        end                     
                    end
                end
            end
        end
    end
end

function flag = CheckInMap(map, pos)
%CHECKINMAP �жϸ��������Ƿ����ڵ�ͼ��
    xlim = map.XWorldLimits; ylim = map.YWorldLimits;
    if pos(1) < xlim(1) || pos(1) > xlim(2)
        flag = false;
    elseif pos(2) < ylim(1) || pos(2) > ylim(2)
        flag = false;
    else
        flag = true;
    end
end

function path = GetPath(closeList, startNode, endNode)
    path = [];
    startIndex = startNode.index;
    current = endNode.index;
    while current ~= startIndex
        path = [path; closeList(current).pos];
        current = closeList(current).parent;
    end
    path = [path; closeList(current).pos];
    path = flipud(path);
end
