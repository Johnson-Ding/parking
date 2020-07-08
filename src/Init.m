function [VEHICLE, CONFIG, map, obstacleList, ego, EXITFLAG] = Init(scenario,MODE)
%INIT 初始化函数，输入泊车场景，生成泊车环境、障碍物信息、本车起始构型和本车终点构型
%   scenario为字符串变量，其合法值只有'parallel'、'backwards'和'diagonal'三种，分别对应平行、垂直、斜向泊车场景
%   map是地图
%   obstacleList是障碍物（也即其它已停放的车辆）的位姿信息，依次是几何中心的横坐标、纵坐标和车辆偏航角，偏航角用弧度制表示
%   egoStartConfig是本车起始位姿
%   egoEndConfig是本车目标位姿，也即空闲车位的位姿
%   EXITFLAG是初始化成功与否的标志，若EXITFLAG为1，说明成功初始化地图信息、障碍物信息等，若EXITFLAG为0，说明没有成功初始化，其原因很可能是scenario的值不合法
    [VEHICLE, CONFIG] = InitConfig(scenario,MODE);
    global CAR_WIDTH CAR_LENGTH MARGIN PARKING_SPACE_LENGTH PARKING_SPACE_WIDTH
    InitGlobalVal(VEHICLE, CONFIG);
    mapLength = CONFIG.MAP_XLIM(2)-CONFIG.MAP_XLIM(1);    
    mapWidth = CONFIG.MAP_YLIM(2)-CONFIG.MAP_YLIM(1);    
    mapResolution = CONFIG.MAP_RESOLUTION; 
    mapDelta = 1/(mapResolution*5);
    wallWidth = CONFIG.WALL_WIDTH;
    map = robotics.BinaryOccupancyGrid(mapLength, mapWidth, mapResolution); %mapLength对应地图的长，mapWidth对应地图的宽，单位都是m，mapResolution对应每米有多少个栅格
    [wallX, wallY] = meshgrid(0:mapDelta:wallWidth, 0:mapDelta:mapWidth);
    setOccupancy(map, [wallX(:) wallY(:)], 1);
    [wallX, wallY] = meshgrid((mapLength-wallWidth+1/mapResolution):mapDelta:mapLength, 0:mapDelta:mapWidth);
    setOccupancy(map, [wallX(:) wallY(:)], 1);
    [wallX, wallY] = meshgrid(0:mapDelta:mapLength, 0:mapDelta:wallWidth);
    setOccupancy(map, [wallX(:) wallY(:)], 1);
    [wallX, wallY] = meshgrid(0:mapDelta:mapLength, (mapWidth-wallWidth+1/mapResolution):mapDelta:mapWidth);
    setOccupancy(map, [wallX(:) wallY(:)], 1);
    
    EXITFLAG = 1;
    ego.length = CAR_LENGTH;
    ego.width = CAR_WIDTH;
    OFFSET = VEHICLE.LENGTH/2 - VEHICLE.LB;   %坐标原点（后轮轴中心）到车辆几何中心的距离
    obstacleList = [];
    if strcmp(scenario,'backwards')
        disp('Start Reverse Parking');
        if strcmp(MODE,'SIM')
            egoStartPose = [5, 8, 0, 0];
%             egoStartPose = [15, 8, 0, 0];
        elseif strcmp(MODE,'REAL')
%             egoStartPose = [0.15, 0.6, 0 , 0];
            egoStartPose = [0.10, 0.45, 0, 0];
        end
        obstaclePose = [(mapLength-PARKING_SPACE_WIDTH-CAR_WIDTH)/2, round((CAR_LENGTH/2+wallWidth+MARGIN)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, pi/2];
        obstacleList = [obstacleList; obstaclePose];
        map = AddObstacle2Map(map, obstaclePose);
        obstaclePose = [(mapLength+PARKING_SPACE_WIDTH+CAR_WIDTH)/2, round((CAR_LENGTH/2+wallWidth+MARGIN)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, pi/2];
        obstacleList = [obstacleList; obstaclePose];
        map = AddObstacle2Map(map, obstaclePose);   
        egoEndX = mapLength/2;
        egoEndY = round((CAR_LENGTH/2+wallWidth+MARGIN)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
        egoEndPose = [egoEndX, egoEndY-OFFSET, pi/2, 0];
    elseif strcmp(scenario,'parallel')
        disp('Start Parallel Parking'); 
        if strcmp(MODE,'SIM')
            egoStartPose = [3, 4.4, 0, 0];
        elseif strcmp(MODE,'REAL')
            egoStartPose = [0.15,0.4,0,0];
        end
        obstaclePose = [(mapLength-PARKING_SPACE_LENGTH-CAR_LENGTH)/2, round((CAR_WIDTH/2+wallWidth+MARGIN/2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, 0];
        obstacleList = [obstacleList; obstaclePose];
        map = AddObstacle2Map(map, obstaclePose);
        obstaclePose = [(mapLength+PARKING_SPACE_LENGTH+CAR_LENGTH)/2, round((CAR_WIDTH/2+wallWidth+MARGIN/2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, 0];
        obstacleList = [obstacleList; obstaclePose];
        map = AddObstacle2Map(map, obstaclePose);
        if strcmp(MODE,'SIM')
            egoEndX = mapLength/2-MARGIN+0.35;
            egoEndY = round((CAR_WIDTH/2+wallWidth+MARGIN/2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
        elseif strcmp(MODE,'REAL')
            egoEndX = mapLength/2-MARGIN+0.02;
            egoEndY = round((CAR_WIDTH/2+wallWidth+MARGIN/2)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
        end
        egoEndPose = [egoEndX-OFFSET, egoEndY, 0, 0];
    elseif strcmp(scenario,'diagonal')
        disp('Start Diagonal Parking');
        if strcmp(MODE,'SIM')
            egoStartPose = [3, 8, 0, 0];
        elseif strcmp(MODE,'REAL')
            egoStartPose = [0.15, 0.5, 0, 0];
        end
        obstacleYawAngle = pi/3;
        obstaclePose = [(mapLength-(PARKING_SPACE_WIDTH+CAR_WIDTH)/sin(obstacleYawAngle))/2, round((CAR_LENGTH/2+wallWidth+MARGIN)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, obstacleYawAngle];
        obstacleList = [obstacleList; obstaclePose];
        map = AddObstacle2Map(map, obstaclePose);
        obstaclePose = [(mapLength+(PARKING_SPACE_WIDTH+CAR_WIDTH)/sin(obstacleYawAngle))/2, round((CAR_LENGTH/2+wallWidth+MARGIN)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, obstacleYawAngle];
        obstacleList = [obstacleList; obstaclePose];
        map = AddObstacle2Map(map, obstaclePose);   
        egoEndX = mapLength/2;
        egoEndY = round((CAR_LENGTH/2+wallWidth+MARGIN)*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION;
        egoEndPose = [round((egoEndX-OFFSET*cos(obstacleYawAngle))*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, round((egoEndY-OFFSET*sin(obstacleYawAngle))*CONFIG.GRID_RESOLUTION)/CONFIG.GRID_RESOLUTION, obstacleYawAngle, 0];
    else
        EXITFLAG = 0;
        fprintf('ERROR: please specify parking scenario\n');
        return;
    end
    ego.start = egoStartPose;
    ego.goal = egoEndPose;
end

function [VEHICLE, CONFIG] = InitConfig(scenario,MODE)
    if strcmp(MODE,'SIM')
        VEHICLE.WB = 2.7;    % [m] 轴距
        VEHICLE.TRACK = 1.5; % [m] 轮距
        VEHICLE.WIDTH = 1.8; % [m] 车宽
        VEHICLE.LENGTH = 4.6;% [m] 车长
        VEHICLE.LF = 1.0;    % [m] 前悬长度
        VEHICLE.LB = 0.9;    % [m] 后悬长度
        VEHICLE.WHEEL_WIDTH = 0.195;   % [m] 车轮宽度
        VEHICLE.WHEEL_LENGTH = 0.6345; % [m] 车轮长度（即车轮直径）
        if strcmp(scenario,'parallel')
            VEHICLE.MIN_TURNING_RADIUS = 5.2; % [m] 最小转弯半径
        else
            VEHICLE.MIN_TURNING_RADIUS = 8;   % [m] 垂直泊车和斜向泊车时，最小转弯半径过小很可能导致Hybrid A*规划不出路径
        end
        VEHICLE.MAX_DELTA = atan2(VEHICLE.WB, 5.2); % [rad] 最大转向角
        VEHICLE.MIN_DELTA = -VEHICLE.MAX_DELTA;
        VEHICLE.MAX_KAPPA = tan(VEHICLE.MAX_DELTA)/VEHICLE.WB; % [m^(-1)] 最大转向曲率
        VEHICLE.MIN_KAPPA = -VEHICLE.MAX_KAPPA;
        VEHICLE.DELTA_RESOLUTION = VEHICLE.MAX_DELTA/2;
        VEHICLE.MAX_V = 2;   % [m/s] 泊车过程的最大速度
        VEHICLE.MIN_V = -2;
        VEHICLE.MAX_A = 5;   % [m/s^(-2)] 泊车过程的最大加速度
        VEHICLE.MIN_A = -5;
        VEHICLE.MAX_SIGMA = 1.5;
        VEHICLE.MIN_SIGMA = -1.5;
        VEHICLE.MIN_OMEGA = -5;
        VEHICLE.MAX_OMEGA = 5;
        CONFIG.MAP_XLIM = [0,20];
        CONFIG.MAP_YLIM = [0,15];
        CONFIG.MAP_RESOLUTION = 100;
        CONFIG.WALL_WIDTH = 0.1;
        CONFIG.MOTION_RESOLUTION = 0.1; 
        CONFIG.GRID_RESOLUTION = 10; % 10代表每米有10个栅格，即栅格长和宽都是0.1m
        CONFIG.YAW_RESOLUTION = deg2rad(0.2);
        CONFIG.MIN_YAW = -pi;
        CONFIG.MAX_YAW = pi;
        CONFIG.SWITCH_BACK_COST = 0; 
        CONFIG.BACKWARD_COST = 10; 
        CONFIG.STEER_COST = 1.5;
        CONFIG.STEER_CHANGE_COST = 5; 
        CONFIG.MARGIN = 0.5;
        CONFIG.SAMPLE = 5;
        CONFIG.T = 0.3;
    elseif strcmp(MODE,'REAL')
        VEHICLE.WB = 0.14;  
        VEHICLE.TRACK = 0.16; 
        VEHICLE.WIDTH = 0.19; 
        VEHICLE.LENGTH = 0.22;
        VEHICLE.LF = 0.05; 
        VEHICLE.LB = 0.03; 
        VEHICLE.WHEEL_WIDTH = 0.018; 
        VEHICLE.WHEEL_LENGTH = 0.065; 
        VEHICLE.MAX_DELTA = 0.5;
        VEHICLE.MIN_DELTA = -VEHICLE.MAX_DELTA;
        VEHICLE.MAX_KAPPA = tan(VEHICLE.MAX_DELTA)/VEHICLE.WB;
        VEHICLE.MIN_KAPPA = -VEHICLE.MAX_KAPPA;
        VEHICLE.MIN_OMEGA = -1;
        VEHICLE.MAX_OMEGA = 1;
        if strcmp(scenario,'parallel')
            VEHICLE.MIN_TURNING_RADIUS = 1/VEHICLE.MAX_KAPPA;
        else
            VEHICLE.MIN_TURNING_RADIUS = 1.6/VEHICLE.MAX_KAPPA; 
        end
        VEHICLE.DELTA_RESOLUTION = VEHICLE.MAX_DELTA/2;
        VEHICLE.MIN_A = -2;
        VEHICLE.MAX_A = 2;
        VEHICLE.MIN_V = -1;
        VEHICLE.MAX_V = 1;
        VEHICLE.MIN_SIGMA = -5;
        VEHICLE.MAX_SIGMA = 5;
        CONFIG.MAP_XLIM = [0,1];
        CONFIG.MAP_YLIM = [0,1];
        CONFIG.MAP_RESOLUTION = 100;
        CONFIG.WALL_WIDTH = 0.01;
        CONFIG.MOTION_RESOLUTION = 0.01; 
        CONFIG.GRID_RESOLUTION = 100; 
        CONFIG.YAW_RESOLUTION = deg2rad(0.2);
        CONFIG.MIN_YAW = -pi;
        CONFIG.MAX_YAW = pi;
        CONFIG.SWITCH_BACK_COST = 0; 
        CONFIG.BACKWARD_COST = 10; 
        CONFIG.STEER_COST = 1.5;
        CONFIG.STEER_CHANGE_COST = 5; 
        if strcmp(scenario,'parallel')
            CONFIG.MARGIN = 0.075;
        else
            CONFIG.MARGIN = 0.04;
        end
        CONFIG.SAMPLE = 5;
        CONFIG.T = 0.3;        
    end
    CONFIG.SCENARIO = scenario;
    CONFIG.MODE = MODE;
end

function map = AddObstacle2Map(map, obstaclePose)
%ADDOBSTACLE2MAP 将障碍物添加到地图上
%   map是地图，类型是robotics.BinaryOccupancyGrid
%   obstaclePose是障碍物的位姿，用1*3的矩阵表示，其元素从左到右依次为相对于地图坐标系而言障碍物几何中心的横坐标、纵坐标以及障碍物的偏航角
%   偏航角用弧度制表示
    global CAR_WIDTH CAR_LENGTH
    mapDelta = 0.005;
    if obstaclePose(3) == 0 || obstaclePose(3) == pi
        [obstacleX, obstacleY] = meshgrid((obstaclePose(1)-CAR_LENGTH/2):mapDelta:(obstaclePose(1)+CAR_LENGTH/2), (obstaclePose(2)-CAR_WIDTH/2):mapDelta:(obstaclePose(2)+CAR_WIDTH/2));
    elseif obstaclePose(3) == pi/2 || obstaclePose(3) == -pi/2
        [obstacleX, obstacleY] = meshgrid((obstaclePose(1)-CAR_WIDTH/2):mapDelta:(obstaclePose(1)+CAR_WIDTH/2), (obstaclePose(2)-CAR_LENGTH/2):mapDelta:(obstaclePose(2)+CAR_LENGTH/2));
    else
        [obstacleX, obstacleY] = meshgrid((-CAR_LENGTH/2):mapDelta:(CAR_LENGTH/2), (-CAR_WIDTH/2):mapDelta:(CAR_WIDTH/2));
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
    setOccupancy(map, [obstacleX(:) obstacleY(:)], 1);
end

function InitGlobalVal(VEHICLE, CONFIG)
    global CAR_WIDTH CAR_LENGTH MARGIN PARKING_SPACE_LENGTH PARKING_SPACE_WIDTH
    CAR_WIDTH = VEHICLE.WIDTH;
    CAR_LENGTH = VEHICLE.LENGTH;
    MARGIN = CONFIG.MARGIN;
    PARKING_SPACE_WIDTH = CAR_WIDTH+MARGIN*2;
    PARKING_SPACE_LENGTH = CAR_LENGTH+MARGIN*4;
end
