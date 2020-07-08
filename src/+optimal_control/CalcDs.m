function [ds] = CalcDs(state,input,T,N,VEHICLE, CONFIG)
%SOLVEDS 根据路径求解d_s
%   state是5*(N+1)的矩阵，每列元素分别对应x,y,psi,v,kappa
%   input是2*N的矩阵，每列元素依次是a、sigma
%   T是每个时间区间的时长
%   N表示泊车时域被分成N个时间区间
%   VEHICLE描述了车辆的各参数
    epsilon = 1e-8;
    if strcmp(CONFIG.MODE,'REAL')
        threshold = 0.002;
        MIN_D = 0.001;
    elseif strcmp(CONFIG.MODE,'SIM')
        threshold = 0.02;
        MIN_D = 0.01;
    end
    LF = VEHICLE.LF;
    WB = VEHICLE.WB;
    LB = VEHICLE.LB;
    WIDTH = VEHICLE.WIDTH;
    LENGTH = VEHICLE.LENGTH;
    dSList = zeros(N,1);
    f = @(z,u)[z(4)*cos(z(3)); z(4)*sin(z(3)); z(4)*z(5); u(1); u(2)];
    for i = 1:1:N
%         tic
        v = state(4,i);
        a = input(1,i);
        sigma = input(2,i);
        if abs(v*T+a*T^2/2) < threshold
            dSList(i) = MIN_D;
            continue;
        end
        if v*T+a*T^2/2 >= 0
            currentPose = state(1:3,i);
            nextPose = state(1:3,i+1);
            kappa_c = state(5,i);
            kappa_n = state(5,i+1);
        else
            currentPose = state(1:3,i+1);
            nextPose = state(1:3,i);
            kappa_c = state(5,i+1);
            kappa_n = state(5,i);
            sigma = -sigma;
            v = -v;
            a = -a;
        end
        current = GetVertexMatrix(currentPose, VEHICLE);
        next = GetVertexMatrix(nextPose,VEHICLE);
        if abs(kappa_c) < epsilon && abs(kappa_n) < epsilon
            dSList(i) = 0;
        elseif kappa_c >= epsilon
            if sigma >= 0
                caseNum = 1;
            elseif abs(kappa_n) > kappa_c
                caseNum = 2;
            else
                caseNum = 3;
            end
        elseif kappa_c <= -epsilon
            if sigma <= 0
                caseNum = 2;
            elseif abs(kappa_n) > abs(kappa_c)
                caseNum = 1;
            else
                caseNum = 4;
            end
        else
            if kappa_n > epsilon
                caseNum = 1;
            else
                caseNum = 2;
            end
        end
        switch caseNum
            case 1
                t_M = abs(fsolve(@(t) atan((LF+WB)/(1/(sigma*t+kappa_c)+WIDTH/2))+...
                      a*sigma*t^3/3+(v*sigma+a*kappa_c)*t^2/2+v*kappa_c*t+currentPose(3)-...
                      atan2(next(2,4)-current(2,4),next(1,4)-current(1,4)), 0.2,optimoptions('fsolve','Display','off','TolFun',epsilon)));
                if t_M > T
                    dSList(i) = MIN_D;
                    continue;
                end
                k1 = f(state(:,i),        input(:,i));
                k2 = f(state(:,i)+t_M/2*k1, input(:,i));
                k3 = f(state(:,i)+t_M/2*k2, input(:,i));
                k4 = f(state(:,i)+t_M*k3,   input(:,i));
                M_state = state(:,i) + t_M/6*(k1+2*k2+2*k3+k4); 
                M_pose = M_state(1:3);
                E_4_M = [cos(M_pose(3)), -sin(M_pose(3)); sin(M_pose(3)), cos(M_pose(3))]*[LF+WB;-WIDTH/2]+M_pose(1:2);
                E_4_k = current(:,4);
                E_4_kp1 = next(:,4); % kp1 for k plus 1
                d = norm(E_4_kp1-E_4_k);
                e = abs(((E_4_kp1(2)-E_4_k(2))*E_4_M(1)-(E_4_kp1(1)-E_4_k(1))*E_4_M(2)-E_4_k(1)*E_4_kp1(2)+E_4_k(2)*E_4_kp1(1))/d);
                vector1 = next(:,4)-current(:,4);
                vector2 = current(:,4)-current(:,3);
                cosD = abs(vector1'*vector2/(d*LENGTH));                
                dSList(i) = e/cosD;
            case 2
                t_M = abs(fsolve(@(t) atan((LF+WB)/(1/(sigma*t+kappa_c)+WIDTH/2))+...
                      a*sigma*t^3/3+(v*sigma+a*kappa_c)*t^2/2+v*kappa_c*t+currentPose(3)-...
                      atan2(next(2,1)-current(2,1),next(1,1)-current(1,1)), 0.2,optimoptions('fsolve','Display','off','TolFun',epsilon)));
                if t_M > T
                    dSList(i) = MIN_D;
                    continue;
                end
                k1 = f(state(:,i),        input(:,i));
                k2 = f(state(:,i)+t_M/2*k1, input(:,i));
                k3 = f(state(:,i)+t_M/2*k2, input(:,i));
                k4 = f(state(:,i)+t_M*k3,   input(:,i));
                M_state = state(:,i) + t_M/6*(k1+2*k2+2*k3+k4); 
                M_pose = M_state(1:3);
                E_1_M = [cos(M_pose(3)), -sin(M_pose(3)); sin(M_pose(3)), cos(M_pose(3))]*[LF+WB;WIDTH/2]+M_pose(1:2);
                E_1_k = current(:,1);
                E_1_kp1 = next(:,1); % kp1 for k plus 1
                d = norm(E_1_kp1-E_1_k);
                e = abs(((E_1_kp1(2)-E_1_k(2))*E_1_M(1)-(E_1_kp1(1)-E_1_k(1))*E_1_M(2)-E_1_k(1)*E_1_kp1(2)+E_1_k(2)*E_1_kp1(1))/d);
                vector1 = next(:,1)-current(:,1);
                vector2 = current(:,1)-current(:,2);
                cosA = abs(vector1'*vector2/(d*LENGTH));                
                dSList(i) = e/cosA;   
            case 3
                t_M = abs(fsolve(@(t) atan(-LB/(1/(sigma*t+kappa_c)+WIDTH/2))+...
                      a*sigma*t^3/3+(v*sigma+a*kappa_c)*t^2/2+v*kappa_c*t+currentPose(3)-...
                      atan2(next(2,3)-current(2,3),next(1,3)-current(1,3)), 0.2,optimoptions('fsolve','Display','off','TolFun',epsilon)));
                if t_M > T
                    dSList(i) = MIN_D;
                    continue;
                end
                k1 = f(state(:,i),        input(:,i));
                k2 = f(state(:,i)+t_M/2*k1, input(:,i));
                k3 = f(state(:,i)+t_M/2*k2, input(:,i));
                k4 = f(state(:,i)+t_M*k3,   input(:,i));
                M_state = state(:,i) + t_M/6*(k1+2*k2+2*k3+k4); 
                M_pose = M_state(1:3);
                E_3_M = [cos(M_pose(3)), -sin(M_pose(3)); sin(M_pose(3)), cos(M_pose(3))]*[-LB;-WIDTH/2]+M_pose(1:2);
                E_3_k = current(:,3);
                E_3_kp1 = next(:,3); % kp1 for k plus 1
                d = norm(E_3_kp1-E_3_k);
                e = abs(((E_3_kp1(2)-E_3_k(2))*E_3_M(1)-(E_3_kp1(1)-E_3_k(1))*E_3_M(2)-E_3_k(1)*E_3_kp1(2)+E_3_k(2)*E_3_kp1(1))/d);
                vector1 = next(:,3)-current(:,3);
                vector2 = next(:,4)-next(:,3);
                cosC = abs(vector1'*vector2/(d*LENGTH));                
                dSList(i) = e/cosC;  
            case 4
                t_M = abs(fsolve(@(t) atan(-LB/(1/(sigma*t+kappa_c)+WIDTH/2))+...
                      a*sigma*t^3/3+(v*sigma+a*kappa_c)*t^2/2+v*kappa_c*t+currentPose(3)-...
                      atan2(next(2,2)-current(2,2),next(1,2)-current(1,2)), 0.2,optimoptions('fsolve','Display','off','TolFun',epsilon)));
                if t_M > T
                    dSList(i) = MIN_D;
                    continue;
                end
                k1 = f(state(:,i),        input(:,i));
                k2 = f(state(:,i)+t_M/2*k1, input(:,i));
                k3 = f(state(:,i)+t_M/2*k2, input(:,i));
                k4 = f(state(:,i)+t_M*k3,   input(:,i));
                M_state = state(:,i) + t_M/6*(k1+2*k2+2*k3+k4); 
                M_pose = M_state(1:3);
                E_2_M = [cos(M_pose(3)), -sin(M_pose(3)); sin(M_pose(3)), cos(M_pose(3))]*[-LB;WIDTH/2]+M_pose(1:2);
                E_2_k = current(:,2);
                E_2_kp1 = next(:,2); % kp1 for k plus 1
                d = norm(E_2_kp1-E_2_k);
                e = abs(((E_2_kp1(2)-E_2_k(2))*E_2_M(1)-(E_2_kp1(1)-E_2_k(1))*E_2_M(2)-E_2_k(1)*E_2_kp1(2)+E_2_k(2)*E_2_kp1(1))/d);
                vector1 = next(:,2)-current(:,2);
                vector2 = next(:,1)-next(:,2);
                cosB = abs(vector1'*vector2/(d*LENGTH));                
                dSList(i) = e/cosB;                  
            otherwise
                dSList(i) = MIN_D;
        end
    end
    ds = max(dSList);
end

function vertexMatrix = GetVertexMatrix(pose, VEHICLE)
    recVertex = [VEHICLE.LF+VEHICLE.WB, VEHICLE.WIDTH/2.0; 
                -VEHICLE.LB, VEHICLE.WIDTH/2.0; 
                -VEHICLE.LB, -VEHICLE.WIDTH/2.0; 
                VEHICLE.LF+VEHICLE.WB, -VEHICLE.WIDTH/2.0]'; %旋转平移之前的矩形，用6*2的矩阵分别表示右上角顶点、左上角顶点、左下角顶点、右下角顶点（即逆时针）、左后轮中心、右后轮中心
    rotMatrix = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
    vertexMatrix = rotMatrix*recVertex+[pose(1)*ones(1,4); pose(2)*ones(1,4)];        
end

