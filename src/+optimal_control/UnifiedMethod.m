function [optimalState, optimalInput, optimalT, EXITFLAG] = UnifiedMethod(CONFIG, VEHICLE, ego, obstacleList, N, initialState, initialInput, initialT)
%DESIGNANDSOLVE 函数建立了自主泊车轨迹规划的NLP问题，并调用CasADi中的IPOPT求解器对NLP问题进行求解
%   函数的输入参数列表共有8项，返回参数有4项。
%   CONFIG是包含环境各种参数的struct变量，包括停车场的边界、地图分辨率等
%   VEHICLE是包含了辆各种参数struct变量，包括轴距、轮距、车身长宽、最大转向角、最小转向半径等
%   ego是描述本车的struct变量，包括本车的长宽、起始位姿和目标位姿（即停车位的位置）
%   obstacleList描述了障碍物（即其它已停放的车辆）的位姿信息，用n*3的矩阵表示。假设所有车辆的长宽都与VEHICLE里的一致
%   N对应的泊车时域内有限元的个数，即将整个泊车时域划分为N个有限元
%   initialState是(N+1)*5的矩阵，表示状态变量一系列取值，N个有限元（时间区间）有N+1个时刻的状态需要描述，因此矩阵共N+1行
%   状态变量共5个，依次是表示位置的x,y，表示姿态psi，表示速度的v，表示转向角的delta
%   initialInput是(N+1)*2的矩阵
%   控制变量共2个，依次是表示加速度的a和表示转向角变化率的omega
%   initialT是泊车时长的初值
    opti = casadi.Opti();
    % 五个状态
    state = opti.variable(N+1,5)';
    x = state(1,:);
    y = state(2,:);
    pos = [x; y];
    psi = state(3,:);
    v = state(4,:);
    delta = state(5,:);
    % 两个输入
    input = opti.variable(N+1,2)';
    a = input(1,:);
    omega = input(2,:);
    % 泊车时间
    T = opti.variable();
    % 等分成N个时间区间
    t_f = T*N;
    s = sum(abs(v*T+a*T^2/2));
%     weight = 1;
%     opti.minimize(sum((state(1,:)-initialState(1,:)).^2 + (state(2,:)-initialState(2,:)).^2 + (state(3,:)-initialState(3,:)).^2)+ weight*t_f);
%     opti.minimize(sum(0.01*input(:,1).^2+0.5*input(:,2).^2) + ...
%                   sum(0.1*(diff(input(:,1))/T).^2+0.1*(diff(input(:,2))/T).^2))
%     opti.minimize(s);
    opti.minimize(t_f);
    XLIM = CONFIG.MAP_XLIM;
    YLIM = CONFIG.MAP_YLIM;
    MIN_DELTA = VEHICLE.MIN_DELTA;
    MAX_DELTA = VEHICLE.MAX_DELTA;
    WB = VEHICLE.WB;
    LENGTH = VEHICLE.LENGTH;
    WIDTH = VEHICLE.WIDTH;
    LF = VEHICLE.LF;
    LB = VEHICLE.LB;
    OFFSET = VEHICLE.LENGTH/2 - VEHICLE.LB;   %坐标原点（后轮轴中心）到车辆几何中心的距离
    MIN_A = VEHICLE.MIN_A;
    MAX_A = VEHICLE.MAX_A;
    MIN_V = VEHICLE.MIN_V;
    MAX_V = VEHICLE.MAX_V;
    MIN_DELTA = VEHICLE.MIN_DELTA;
    MAX_DELTA = VEHICLE.MAX_DELTA;
    MIN_OMEGA = VEHICLE.MIN_OMEGA;
    MAX_OMEGA = VEHICLE.MAX_OMEGA;

    originVertex = [VEHICLE.LF+VEHICLE.WB, VEHICLE.WIDTH/2.0; 
                    -VEHICLE.LB,           VEHICLE.WIDTH/2.0; 
                    -VEHICLE.LB,           -VEHICLE.WIDTH/2.0; 
                    VEHICLE.LF+VEHICLE.WB, -VEHICLE.WIDTH/2.0; 
                    VEHICLE.LF+VEHICLE.WB, VEHICLE.WIDTH/2.0]'; %旋转平移之前的矩形顶点，用5*2的矩阵分别表示右上角顶点、左上角顶点、左下角顶点、右下角顶点（即逆时针）、左后轮中心、右后轮中心
                
    [obstacleListLength, ~] = size(obstacleList);
    obstacleVertexList = zeros(2,obstacleListLength*5);
    j = 1;
    for i = 1:1:obstacleListLength
        obstacleVertex = GetObstacleVertexMatrix(obstacleList(i,:), LENGTH, WIDTH);
        obstacleVertexList(:,j:j+4) = obstacleVertex;      
        j = j+5;
    end
    obstacleVertexListLength = obstacleListLength*5;
                
    % 状态方程
    f = @(z,u)[z(4)*cos(z(3)); z(4)*sin(z(3)); z(4)*tan(z(5))/VEHICLE.WB; u(1); u(2)];
    % Runge-Kutta 4 integration
    for i = 1:N
        k1 = f(state(:,i),        input(:,i));
        k2 = f(state(:,i)+T/2*k1, input(:,i));
        k3 = f(state(:,i)+T/2*k2, input(:,i));
        k4 = f(state(:,i)+T*k3,   input(:,i));
        z_next = state(:,i) + T/6*(k1+2*k2+2*k3+k4); 
        opti.subject_to(state(:,i+1) == z_next);
    end
    % 边界约束
    opti.subject_to(XLIM(1) < x < XLIM(2)-sqrt((WB+LF)^2+(WIDTH/2)^2)-CONFIG.WALL_WIDTH);
    opti.subject_to(YLIM(1) < y < YLIM(2)-sqrt((WB+LF)^2+(WIDTH/2)^2)-CONFIG.WALL_WIDTH);
    opti.subject_to(-pi <= state(3,:) <= pi);
    opti.subject_to(MIN_V <= state(4,:) <= MAX_V);
    opti.subject_to(MIN_DELTA <= state(5,:) <= MAX_DELTA);
    % 输入量及其变化率的约束
    opti.subject_to(MIN_A <= input(1,:) <= MAX_A);
    opti.subject_to(MIN_OMEGA <= input(2,:) <= MAX_OMEGA);
    % 初值约束
    opti.subject_to(state(1:4,1) == [ego.start(1:3), 0]');
    % 终值约束
    opti.subject_to(state(1:4,N+1) == [ego.goal(1:3), 0]');
    input(1,N+1) = 0;
    input(2,N+1) = 0;
    opti.subject_to(t_f <= 80);
%     碰撞约束
%     保证配置点时刻无碰
    for i = 1:1:N
        for vertexIndex = 1:1:4
            p1 = pos(:,i) + [cos(psi(i)) -sin(psi(i));sin(psi(i)) cos(psi(i))]*originVertex(:,vertexIndex);
            p2 = pos(:,i) + [cos(psi(i)) -sin(psi(i));sin(psi(i)) cos(psi(i))]*originVertex(:,vertexIndex+1);
            for index = 1:1:obstacleVertexListLength
                if mod(index,5) == 0
                    continue
                end
                q1 = obstacleVertexList(:,index);
                q2 = obstacleVertexList(:,index+1);
                opti.subject_to(max(CrossProduct(p2-p1,q1-p1)*CrossProduct(p2-p1,q2-p1),CrossProduct(q2-q1,p1-q1)*CrossProduct(q2-q1,p2-q1))>0);
            end
        end
    end     

    opti.set_initial(T,initialT);
    opti.set_initial(state, initialState);
    opti.set_initial(input, initialInput);

    opti.solver('ipopt');
    sol = opti.solve();

    optimalT = sol.value(T);
    optimalState = sol.value(state);
    optimalInput = sol.value(input);
    EXITFLAG = true;
end

function vertexMatrix = GetObstacleVertexMatrix(pose, length, width)
    recVertex = [length/2.0, width/2.0;
                -length/2.0, width/2.0; 
                -length/2.0, -width/2.0; 
                 length/2.0, -width/2.0;
                 length/2.0, width/2.0]'; %旋转平移之前的矩形，用5*2的矩阵分别表示右上角顶点、左上角顶点、左下角顶点、右下角顶点（即逆时针）
    rotMatrix = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
    vertexMatrix = rotMatrix*recVertex+[pose(1)*ones(1,5); pose(2)*ones(1,5)];    
end

function result = CrossProduct(a,b)
    result = a(1)*b(2)-a(2)*b(1);
end
