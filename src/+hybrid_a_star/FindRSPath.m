function [path, EXITFLAG] = FindRSPath(pose)
%FINDRSPATH 寻找原点到pose的Reeds-Shepp曲线
    EXITFLAG = false; path = [];
    % 遍历5类路径到达目标点，然后选取路径最短的一条
    [path1, EXITFLAG1] = CSC(pose(1),pose(2),pose(3));
    [path2, EXITFLAG2] = CCC(pose(1),pose(2),pose(3));
    [path3, EXITFLAG3] = CCCC(pose(1),pose(2),pose(3));
    [path4, EXITFLAG4] = CCSC(pose(1),pose(2),pose(3));
    [path5, EXITFLAG5] = CCSCC(pose(1),pose(2),pose(3));
    EXITFLAG_LIST = [EXITFLAG1, EXITFLAG2, EXITFLAG3, EXITFLAG4, EXITFLAG5];
    pathList = {path1, path2, path3, path4, path5};
    Lmin = inf;
    % 找出5条路径最短的曲线
    for i = 1:5
        if EXITFLAG_LIST(i) == true
            EXITFLAG = true;
            ele = pathList{i};
            if Lmin > ele.totalLength
                Lmin = ele.totalLength;
                path = ele;
            end
        end
    end
end

% 控制角度x取值范围是[-pi,pi]
function v = mod2pi(x)
    v = rem(x,2*pi);
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end

% formula 8.6
function [tau,omega] = tauOmega(u,v,xi,eta,phi)
    delta = mod2pi(u-v);
    A = sin(u)-sin(delta);
    B = cos(u)-cos(delta)-1;
    t1 = atan2(eta*A-xi*B,xi*A+eta*B);
    t2 = 2*(cos(delta)-cos(v)-cos(u))+3;
    if t2 < 0
        tau = mod2pi(t1+pi);
    else
        tau = mod2pi(t1);
    end
    omega = mod2pi(tau-u+v-phi);
end

% formula 8.1
function [EXITFLAG,t,u,v] = LpSpLp(x,y,phi)
    [t,u] = cart2pol(x-sin(phi),y-1+cos(phi)); % 将笛卡尔坐标转换为极坐标,返回theta和rho,论文返回的是[u,t],是因为cart2pol函数返回的值的顺序不同导致与原文不同，变量代表的含义还是一样，t代表弧度，u代表直行的距离
    if t >= 0 % 必须是左转,t>=0代表左转
        v = mod2pi(phi-t);
        if v >= 0 % 符号代表前进和后退
            EXITFLAG = true;
            return
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.2
function [EXITFLAG,t,u,v] = LpSpRp(x,y,phi)
    [t1,u1] = cart2pol(x+sin(phi),y-1-cos(phi));
    if u1^2 >= 4
        u = sqrt(u1^2-4);
        theta = atan2(2,u);
        t = mod2pi(t1+theta);
        v = mod2pi(t-phi);
        if t >= 0 && v >= 0 % 符号代表前进和后退
            EXITFLAG = true;
            return
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

function [path, EXITFLAG] = CSC(x,y,phi)
    Lmin = inf;
    [EXITFLAG,t,u,v] = LpSpLp(x,y,phi);
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(15,:),t,u,v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpSpLp(-x,y,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(15,:),-t,-u,-v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpSpLp(x,-y,-phi); % reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(16,:),t,u,v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpSpLp(-x,-y,phi); % timeflp + reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(16,:),-t,-u,-v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpSpRp(x,y,phi);
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(13,:),t,u,v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpSpRp(-x,y,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(13,:),-t,-u,-v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpSpRp(x,-y,-phi); % reflect 
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(14,:),t,u,v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpSpRp(-x,-y,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(14,:),-t,-u,-v,0,0,L);
        end
    end
    if Lmin < inf
        EXITFLAG = true;
    else
        type = repmat('N',[1,5]);
        path = hybrid_a_star.RSPath(type,0,0,0,0,0,0);
        EXITFLAG = false;
    end
end

% formula 8.3/8.4
function [EXITFLAG,t,u,v] = LpRmL(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,u1] = cart2pol(xi,eta);
    if u1 <= 4
        u = -2*asin(u1/4);
        t = mod2pi(theta+u/2+pi);
        v = mod2pi(phi-t+u);
        if t >= 0 && u <= 0
            EXITFLAG = true;
            return
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

function [path, EXITFLAG] = CCC(x,y,phi)
    Lmin = inf;
    [EXITFLAG,t,u,v] = LpRmL(x,y,phi);
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(1,:),t,u,v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmL(-x,y,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(1,:),-t,-u,-v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmL(x,-y,-phi); % reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(2,:),t,u,v,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmL(-x,-y,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(2,:),-t,-u,-v,0,0,L);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [EXITFLAG,t,u,v] = LpRmL(xb,yb,phi);
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(1,:),v,u,t,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmL(-xb,yb,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(1,:),-v,-u,-t,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmL(xb,-yb,-phi); % reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(2,:),v,u,t,0,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmL(-xb,-yb,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(2,:),-v,-u,-t,0,0,L);
        end
    end
    if Lmin < inf
        EXITFLAG = true;
    else
        EXITFLAG = false;
        type = repmat('N',[1,5]);
        path = hybrid_a_star.RSPath(type,0,0,0,0,0,0);
    end
end

% formula 8.7,tauOmega() is formula 8.6
function [EXITFLAG,t,u,v] = LpRupLumRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (2+sqrt(xi^2+eta^2))/4;
    if rho <= 1
        u = acos(rho);
        [t,v] = tauOmega(u,-u,xi,eta,phi);
        if t >= 0 && v <= 0 % 符号代表前进和后退
            EXITFLAG = true;
            return
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.8
function [EXITFLAG,t,u,v] = LpRumLumRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (20-xi^2-eta^2)/16;
    if rho >= 0 && rho <= 1
        u = -acos(rho);
        if u >= -pi/2
            [t,v] = tauOmega(u,u,xi,eta,phi);
            if t >=0 && v >=0
                EXITFLAG = true;
                return
            end
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

function [path, EXITFLAG] = CCCC(x,y,phi)
    Lmin = inf;
    [EXITFLAG,t,u,v] = LpRupLumRm(x,y,phi);
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(3,:),t,u,-u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRupLumRm(-x,y,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(3,:),-t,-u,u,-v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRupLumRm(x,-y,-phi); % reflect
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(4,:),t,u,-u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRupLumRm(-x,-y,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(4,:),-t,-u,u,-v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRumLumRp(x,y,phi);
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(3,:),t,u,u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRumLumRp(-x,y,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(3,:),-t,-u,-u,-v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRumLumRp(x,-y,-phi); % reflect
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(4,:),t,u,u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRumLumRp(-x,-y,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(4,:),-t,-u,-u,-v,0,L);
        end
    end
    if Lmin < inf
        EXITFLAG = true;
    else
        EXITFLAG = false;
        type = repmat('N',[1,5]);
        path = hybrid_a_star.RSPath(type,0,0,0,0,0,0);
    end
end

% formula 8.9
function [EXITFLAG,t,u,v] = LpRmSmLm(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,rho] = cart2pol(xi,eta);
    if rho >= 2
        r = sqrt(rho^2-4);
        u = 2-r;
        t = mod2pi(theta+atan2(r,-2));
        v = mod2pi(phi-pi/2-t);
        if t >= 0 && u <= 0 && v <= 0
            EXITFLAG = true;
            return
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.10
function [EXITFLAG,t,u,v] = LpRmSmRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [theta,rho] = cart2pol(-eta,xi);
    if rho >= 2
        t = theta;
        u = 2-rho;
        v = mod2pi(t+pi/2-phi);
        if t >= 0 && u <= 0 && v <= 0
            EXITFLAG = true;
            return
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

function [path, EXITFLAG] = CCSC(x,y,phi)
    Lmin = inf;
    [EXITFLAG,t,u,v] = LpRmSmLm(x,y,phi);
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(5,:),t,-pi/2,u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmLm(-x,y,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(5,:),-t,pi/2,-u,-v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmLm(x,-y,-phi); % reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(6,:),t,-pi/2,u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmLm(-x,-y,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(6,:),-t,pi/2,-u,-v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(x,y,phi);
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(9,:),t,-pi/2,u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(-x,y,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(9,:),-t,pi/2,-u,-v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(x,-y,-phi); % reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(10,:),t,-pi/2,u,v,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(-x,-y,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(10,:),-t,pi/2,-u,-v,0,L);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [EXITFLAG,t,u,v] = LpRmSmLm(xb,yb,phi);
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(7,:),v,u,-pi/2,t,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmLm(-xb,yb,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(7,:),-v,-u,pi/2,-t,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmLm(xb,-yb,-phi); % reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(8,:),v,u,-pi/2,t,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmLm(-xb,-yb,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(8,:),-v,-u,pi/2,-t,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(xb,yb,phi);
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(11,:),v,u,-pi/2,t,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(-xb,yb,-phi); % timeflip
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(11,:),-v,-u,pi/2,-t,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(xb,-yb,-phi); % reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(12,:),v,u,-pi/2,t,0,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSmRm(-xb,-yb,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+pi/2+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(12,:),-v,-u,pi/2,-t,0,L);
        end
    end
    if Lmin < inf
        EXITFLAG = true;
    else
        EXITFLAG = false;
        type = repmat('N',[1,5]);
        path = hybrid_a_star.RSPath(type,0,0,0,0,0,0);
    end
end

% formula 8.11
function [EXITFLAG,t,u,v] = LpRmSLmRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [~,rho] = cart2pol(xi,eta);
    if rho >= 2
        u = 4-sqrt(rho^2-4);
        if u <= 0
            t = mod2pi(atan2((4-u)*xi-2*eta,-2*xi+(u-4)*eta));
            v = mod2pi(t-phi);
            if t >= 0 && v >= 0
                EXITFLAG = true;
                return
            end
        end
    end
    EXITFLAG = false;
    t = 0;
    u = 0;
    v = 0;
end

function [path, EXITFLAG] = CCSCC(x,y,phi)
    Lmin = inf;
    [EXITFLAG,t,u,v] = LpRmSLmRp(x,y,phi);
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v)+pi;
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(17,:),t,-pi/2,u,-pi/2,v,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v)+pi;
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(17,:),-t,pi/2,-u,pi/2,-v,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSLmRp(x,y,phi); % reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v)+pi;
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(18,:),t,-pi/2,u,-pi/2,v,L);
        end
    end
    [EXITFLAG,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip + reflect
    if EXITFLAG
        L = abs(t)+abs(u)+abs(v)+pi;
        if Lmin > L
            Lmin = L;
            path = hybrid_a_star.RSPath(hybrid_a_star.RSPath.Types(18,:),-t,pi/2,-u,pi/2,-v,L);
        end
    end
    if Lmin < inf
        EXITFLAG = true;
    else
        EXITFLAG = false;
        type = repmat('N',[1,5]);
        path = hybrid_a_star.RSPath(type,0,0,0,0,0,0);
    end
end
