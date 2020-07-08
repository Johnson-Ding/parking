close all; clear; clc;
% MODE = 'REAL';
MODE = 'SIM';
% scenario = 'backwards';
scenario = 'parallel';
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
if EXITFLAG == 1
    figNum = 2;
    figTitle = scenario;
    timeInterval = 0.05;
    plot_trajectory.PlotTraj(obstacleList, path, VEHICLE, CONFIG, imgPath, figNum, figTitle, timeInterval);
end
