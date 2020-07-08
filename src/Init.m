function [VEHICLE, CONFIG, map, obstacleList, ego, EXITFLAG] = Init(scenario,MODE)
%INIT ��ʼ�����������벴�����������ɲ����������ϰ�����Ϣ��������ʼ���ͺͱ����յ㹹��
%   scenarioΪ�ַ�����������Ϸ�ֵֻ��'parallel'��'backwards'��'diagonal'���֣��ֱ��Ӧƽ�С���ֱ��б�򲴳�����
%   map�ǵ�ͼ
%   obstacleList���ϰ��Ҳ��������ͣ�ŵĳ�������λ����Ϣ�������Ǽ������ĵĺ����ꡢ������ͳ���ƫ���ǣ�ƫ�����û����Ʊ�ʾ
%   egoStartConfig�Ǳ�����ʼλ��
%   egoEndConfig�Ǳ���Ŀ��λ�ˣ�Ҳ�����г�λ��λ��
%   EXITFLAG�ǳ�ʼ���ɹ����ı�־����EXITFLAGΪ1��˵���ɹ���ʼ����ͼ��Ϣ���ϰ�����Ϣ�ȣ���EXITFLAGΪ0��˵��û�гɹ���ʼ������ԭ��ܿ�����scenario��ֵ���Ϸ�
    [VEHICLE, CONFIG] = InitConfig(scenario,MODE);
    global CAR_WIDTH CAR_LENGTH MARGIN PARKING_SPACE_LENGTH PARKING_SPACE_WIDTH
    InitGlobalVal(VEHICLE, CONFIG);
    mapLength = CONFIG.MAP_XLIM(2)-CONFIG.MAP_XLIM(1);    
    mapWidth = CONFIG.MAP_YLIM(2)-CONFIG.MAP_YLIM(1);    
    mapResolution = CONFIG.MAP_RESOLUTION; 
    mapDelta = 1/(mapResolution*5);
    wallWidth = CONFIG.WALL_WIDTH;
    map = robotics.BinaryOccupancyGrid(mapLength, mapWidth, mapResolution); %mapLength��Ӧ��ͼ�ĳ���mapWidth��Ӧ��ͼ�Ŀ���λ����m��mapResolution��Ӧÿ���ж��ٸ�դ��
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
    OFFSET = VEHICLE.LENGTH/2 - VEHICLE.LB;   %����ԭ�㣨���������ģ��������������ĵľ���
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
        VEHICLE.WB = 2.7;    % [m] ���
        VEHICLE.TRACK = 1.5; % [m] �־�
        VEHICLE.WIDTH = 1.8; % [m] ����
        VEHICLE.LENGTH = 4.6;% [m] ����
        VEHICLE.LF = 1.0;    % [m] ǰ������
        VEHICLE.LB = 0.9;    % [m] ��������
        VEHICLE.WHEEL_WIDTH = 0.195;   % [m] ���ֿ��
        VEHICLE.WHEEL_LENGTH = 0.6345; % [m] ���ֳ��ȣ�������ֱ����
        if strcmp(scenario,'parallel')
            VEHICLE.MIN_TURNING_RADIUS = 5.2; % [m] ��Сת��뾶
        else
            VEHICLE.MIN_TURNING_RADIUS = 8;   % [m] ��ֱ������б�򲴳�ʱ����Сת��뾶��С�ܿ��ܵ���Hybrid A*�滮����·��
        end
        VEHICLE.MAX_DELTA = atan2(VEHICLE.WB, 5.2); % [rad] ���ת���
        VEHICLE.MIN_DELTA = -VEHICLE.MAX_DELTA;
        VEHICLE.MAX_KAPPA = tan(VEHICLE.MAX_DELTA)/VEHICLE.WB; % [m^(-1)] ���ת������
        VEHICLE.MIN_KAPPA = -VEHICLE.MAX_KAPPA;
        VEHICLE.DELTA_RESOLUTION = VEHICLE.MAX_DELTA/2;
        VEHICLE.MAX_V = 2;   % [m/s] �������̵�����ٶ�
        VEHICLE.MIN_V = -2;
        VEHICLE.MAX_A = 5;   % [m/s^(-2)] �������̵������ٶ�
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
        CONFIG.GRID_RESOLUTION = 10; % 10����ÿ����10��դ�񣬼�դ�񳤺Ϳ���0.1m
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
%ADDOBSTACLE2MAP ���ϰ�����ӵ���ͼ��
%   map�ǵ�ͼ��������robotics.BinaryOccupancyGrid
%   obstaclePose���ϰ����λ�ˣ���1*3�ľ����ʾ����Ԫ�ش���������Ϊ����ڵ�ͼ����ϵ�����ϰ��Ｘ�����ĵĺ����ꡢ�������Լ��ϰ����ƫ����
%   ƫ�����û����Ʊ�ʾ
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
