close all; clc;clear;
import casadi.*;

% MODE = 'REAL';
MODE = 'SIM';
scenario = 'backwards';
% scenario = 'parallel';
% scenario = 'diagonal';
if strcmp(MODE,'REAL')
    imgPath = '..\img\real\';
    dataPath = '..\data\real\';
elseif strcmp(MODE,'SIM')
    imgPath = '..\img\sim\';
    dataPath = '..\data\sim\';
end
costmap_path = strcat(dataPath,scenario,'_costmap.mat');
[VEHICLE, CONFIG, map, obstacleList, ego, EXITFLAG] = Init(scenario,MODE);
if EXITFLAG == 0
    fprintf('fail to initiate parking scenario\n');
    return;
end

tic
[costMap] = hybrid_a_star.GetAStarCostMap(map, obstacleList, ego, CONFIG,VEHICLE); 
save(costmap_path,'costMap');
% load(costmap_path);
toc

tic
[path, EXITFLAG] = hybrid_a_star.HybridAStar(obstacleList, costMap, ego, VEHICLE, CONFIG);
toc

% if EXITFLAG == true
%     figNum = 100;
%     figTitle = strcat(scenario,'(hybrid A star)');
%     timeInterval = 0.1;
%     plot_trajectory.PlotTraj(obstacleList, path, VEHICLE, CONFIG, imgPath, figNum, figTitle, timeInterval);
% end

Ts = CONFIG.T*CONFIG.SAMPLE;
if EXITFLAG == 1
    x = path(:,1);  y = path(:,2);  psi = path(:,3);  delta_f = path(:,4);
    kappa = tan(delta_f)/VEHICLE.WB;
    x0 = x(1:CONFIG.SAMPLE:end);
    y0 = y(1:CONFIG.SAMPLE:end);
    psi0 = psi(1:CONFIG.SAMPLE:end);
    v = zeros(length(x0),1);
    for i = 1:1:length(x0)-1
        v(i) = ((x0(i+1)-x0(i))*cos(psi0(i))+(y0(i+1)-y0(i))*sin(psi0(i)))/Ts;
    end
    [v0,a0] = optimal_control.SmoothV(v,VEHICLE.MAX_A,CONFIG.T);
    kappa0 = kappa(1:CONFIG.SAMPLE:end);
    sigma0 = [diff(kappa0)/Ts;0];
    delta0 = delta_f(1:CONFIG.SAMPLE:end);
    omega0 = [diff(delta0)/Ts;0];
    N = length(x0)-1;
else
    N = 40;
    x0 = zeros(N,1); y0 = zeros(N,1); psi0 = zeros(N,1); v0 = zeros(N,1); kappa0 = zeros(N,1); 
    a0 = zeros(N,1); sigma0 = zeros(N,1); delta0 = zeros(N,1); omega0 = zeros(N,1);
end
%%
% A unified motion planning method for parking an autonomous vehicle in the presence of irregularly placed obstacles
% initialState0 = [x0'; y0'; psi0'; v0'; delta0'];
% initialT0 = Ts;
% initialInput0 = [a0'; omega0'];
% disp('开始优化')
% tic
% [optimalState0, optimalInput0, optimalT0, EXITFLAG] = optimal_control.UnifiedMethod(CONFIG, VEHICLE, ego, obstacleList, N, initialState0, initialInput0, initialT0);
% toc
% if EXITFLAG == true
%     figNum = 100;
%     figTitle = strcat(scenario,'(unified motion planning method)');
%     timeInterval = optimalT0;
% %     plot_trajectory.PlotTraj(obstacleList, [optimalState0(1:3,:);optimalState0(5,:)]', VEHICLE, CONFIG, imgPath, figNum, figTitle, timeInterval);
%     plot_trajectory.PlotRegion(obstacleList, optimalState0', optimalInput0', 10, VEHICLE, CONFIG, imgPath, figNum, figTitle, timeInterval)
% end

%%
% 保证无碰撞的轨迹规划方法
initialState = [x0'; y0'; psi0'; v0'; kappa0'];
initialInput = [a0'; sigma0'];
initialT = Ts;
d_s = 0;
if strcmp(MODE,'SIM')
    epsilon = 0.001;
elseif strcmp(MODE,'REAL')
    epsilon = 0.00001;
end
disp('开始优化')
ite = 1;
tic
[optimalState, optimalInput, optimalT, optimalS, EXITFLAG] = optimal_control.GuaranteedCollisionFreeMethod(CONFIG, VEHICLE, ego, obstacleList, N, initialState, initialInput, initialT, d_s);

disp('计算安全参数')
d_s_new = optimal_control.CalcDs(optimalState,optimalInput,optimalT,N,VEHICLE,CONFIG)

while d_s_new-d_s > epsilon
    d_s = d_s_new;
    initialState = optimalState;
    initialInput = optimalInput;
    initialT = optimalT;
    [optimalState, optimalInput, optimalT, optimalS, EXITFLAG] = optimal_control.GuaranteedCollisionFreeMethod(CONFIG, VEHICLE, ego, obstacleList, N, initialState, initialInput, initialT, d_s);
    disp('计算安全参数')
    d_s_new = optimal_control.CalcDs(optimalState,optimalInput,optimalT,N,VEHICLE,CONFIG)
    ite = ite+1;
end
toc
% if EXITFLAG == true
%     figNum = 101;
%     figTitle = strcat(scenario,'(guaranteed collision free method)');
%     timeInterval = optimalT;
%     plot_trajectory.PlotTraj(obstacleList, [optimalState(1:3,:);atan(optimalState(5,:)*VEHICLE.WB)]', VEHICLE, CONFIG, imgPath, figNum, figTitle, timeInterval);
%     plot_trajectory.PlotRegion1(obstacleList, optimalState', optimalInput', 10, VEHICLE, CONFIG, imgPath, 200, figTitle, timeInterval)
% end

filename = '../data/control_cmd/data.csv';
writer = fopen(filename, 'w');
[~,n] = size(optimalState);
fprintf(writer, ['%f','\n'], optimalT);
optimalDelta = atan(VEHICLE.WB*optimalState(5,:));
optimalDeltaDot = optimalInput(2,:)*VEHICLE.WB./(1+(optimalState(5,:)));
optimalV = optimalState(4,:);
optimalA = optimalInput(1,:);
for i=1:n
    % [delta, delta_dot, v, a]
    fprintf(writer, ['%f',',','%f',',','%f',',','%f','\n'], optimalDelta(i),optimalDeltaDot(i),optimalV(i),optimalA(i));
end 
