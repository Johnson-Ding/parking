function [optimalState, optimalInput, optimalT, EXITFLAG] = UnifiedMethod(CONFIG, VEHICLE, ego, obstacleList, N, initialState, initialInput, initialT)
%DESIGNANDSOLVE �������������������켣�滮��NLP���⣬������CasADi�е�IPOPT�������NLP����������
%   ��������������б���8����ز�����4�
%   CONFIG�ǰ����������ֲ�����struct����������ͣ�����ı߽硢��ͼ�ֱ��ʵ�
%   VEHICLE�ǰ����������ֲ���struct������������ࡢ�־ࡢ���������ת��ǡ���Сת��뾶��
%   ego������������struct���������������ĳ�����ʼλ�˺�Ŀ��λ�ˣ���ͣ��λ��λ�ã�
%   obstacleList�������ϰ����������ͣ�ŵĳ�������λ����Ϣ����n*3�ľ����ʾ���������г����ĳ�����VEHICLE���һ��
%   N��Ӧ�Ĳ���ʱ��������Ԫ�ĸ�����������������ʱ�򻮷�ΪN������Ԫ
%   initialState��(N+1)*5�ľ��󣬱�ʾ״̬����һϵ��ȡֵ��N������Ԫ��ʱ�����䣩��N+1��ʱ�̵�״̬��Ҫ��������˾���N+1��
%   ״̬������5���������Ǳ�ʾλ�õ�x,y����ʾ��̬psi����ʾ�ٶȵ�v����ʾת��ǵ�delta
%   initialInput��(N+1)*2�ľ���
%   ���Ʊ�����2���������Ǳ�ʾ���ٶȵ�a�ͱ�ʾת��Ǳ仯�ʵ�omega
%   initialT�ǲ���ʱ���ĳ�ֵ
    opti = casadi.Opti();
    % ���״̬
    state = opti.variable(N+1,5)';
    x = state(1,:);
    y = state(2,:);
    pos = [x; y];
    psi = state(3,:);
    v = state(4,:);
    delta = state(5,:);
    % ��������
    input = opti.variable(N+1,2)';
    a = input(1,:);
    omega = input(2,:);
    % ����ʱ��
    T = opti.variable();
    % �ȷֳ�N��ʱ������
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
    OFFSET = VEHICLE.LENGTH/2 - VEHICLE.LB;   %����ԭ�㣨���������ģ��������������ĵľ���
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
                    VEHICLE.LF+VEHICLE.WB, VEHICLE.WIDTH/2.0]'; %��תƽ��֮ǰ�ľ��ζ��㣬��5*2�ľ���ֱ��ʾ���ϽǶ��㡢���ϽǶ��㡢���½Ƕ��㡢���½Ƕ��㣨����ʱ�룩����������ġ��Һ�������
                
    [obstacleListLength, ~] = size(obstacleList);
    obstacleVertexList = zeros(2,obstacleListLength*5);
    j = 1;
    for i = 1:1:obstacleListLength
        obstacleVertex = GetObstacleVertexMatrix(obstacleList(i,:), LENGTH, WIDTH);
        obstacleVertexList(:,j:j+4) = obstacleVertex;      
        j = j+5;
    end
    obstacleVertexListLength = obstacleListLength*5;
                
    % ״̬����
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
    % �߽�Լ��
    opti.subject_to(XLIM(1) < x < XLIM(2)-sqrt((WB+LF)^2+(WIDTH/2)^2)-CONFIG.WALL_WIDTH);
    opti.subject_to(YLIM(1) < y < YLIM(2)-sqrt((WB+LF)^2+(WIDTH/2)^2)-CONFIG.WALL_WIDTH);
    opti.subject_to(-pi <= state(3,:) <= pi);
    opti.subject_to(MIN_V <= state(4,:) <= MAX_V);
    opti.subject_to(MIN_DELTA <= state(5,:) <= MAX_DELTA);
    % ����������仯�ʵ�Լ��
    opti.subject_to(MIN_A <= input(1,:) <= MAX_A);
    opti.subject_to(MIN_OMEGA <= input(2,:) <= MAX_OMEGA);
    % ��ֵԼ��
    opti.subject_to(state(1:4,1) == [ego.start(1:3), 0]');
    % ��ֵԼ��
    opti.subject_to(state(1:4,N+1) == [ego.goal(1:3), 0]');
    input(1,N+1) = 0;
    input(2,N+1) = 0;
    opti.subject_to(t_f <= 80);
%     ��ײԼ��
%     ��֤���õ�ʱ������
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
                 length/2.0, width/2.0]'; %��תƽ��֮ǰ�ľ��Σ���5*2�ľ���ֱ��ʾ���ϽǶ��㡢���ϽǶ��㡢���½Ƕ��㡢���½Ƕ��㣨����ʱ�룩
    rotMatrix = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
    vertexMatrix = rotMatrix*recVertex+[pose(1)*ones(1,5); pose(2)*ones(1,5)];    
end

function result = CrossProduct(a,b)
    result = a(1)*b(2)-a(2)*b(1);
end
