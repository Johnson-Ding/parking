close all; clear; clc;
% MODE = 'REAL';
MODE = 'SIM';
% scenario = 'backwards';
scenario = 'parallel';
% scenario = 'diagonal';
[VEHICLE, CONFIG, map, obstacleList, ego, EXITFLAG] = Init(scenario,MODE);
if EXITFLAG == 0
    fprintf('fail to initiate parking scenario\n');
    return;
end
tic
[path, EXITFLAG] = a_star.AStar(map, obstacleList, ego); 
toc
show(map);
% grid on;
hold on;
if EXITFLAG == 0
    disp('A Star fails');
else
    plot(path(:,1),path(:,2),'r');
end

