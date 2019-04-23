%{
    Author  Dingyi Sun
    Date    April 9 2019
    The solver for the ploblem has changed into gtsam.
    Make sure to add the directory: gtsam_toobox 
%}

clc; clear; close all

import gtsam.*

fsize = 16; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

% nice colours
green = [0.2980 .6 0];
crimson = [220,20,60]/255; 
darkblue = [0 .2 .4];
Darkgrey = [.25 .25 .25];
darkgrey = [.35 .35 .35];
lightgrey = [.7 .7 .7];
Lightgrey = [.9 .9 .9];
VermillionRed = [156,31,46]/255;
DupontGray = [144,131,118]/255;
Azure = [53, 112, 188]/255;
purple = [178, 102, 255]/255;
orange = [255,110,0]/255;

%% Setup the map and ground truth trajectory
% map of landmarks
map = [1.3, 2.7, 1.0
       4.0, 2.6, 1.0
       0.1, 1.2, 1.0
       2.3, 1.1, 2.0
       1.7, 0.6, 2.0
       3.7, 3.1, 3.0
       2.4, 3.4, 3.0
       2.9, 2.0, 4.0
       1.2, 1.8, 1.0];
% robot group truth positions
x_gt = [0,0, 1.0
        0.5, 0.4, 1.0
        1, 0.6, 1.0
        1.3, 1.1, 2.0
        1.7, 1.6, 2.0
        2.1, 1.7, 5.0
        2.4, 1.9, 3.0
        2.5, 2.4, 3.0
        2.7, 2.7, 4.0
        2.9, 3.0, 4.0
        3.1, 2.9, 4.0
        3.4, 2.5, 4.0
        3.5, 2.1, 2.0
        3.5, 1.7, 2.0
        3.3, 1.6, 1.0
        2.8, 1.6, 1.0];
    

% plot the map and ground truth trajectory
h_leg = []; % legend handle
figure; 
hold on; grid on; axis auto
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
plot3(map(:,1), map(:,2), map(:,3), '*k', 'MarkerSize', 10)
plot3(map(:,1), map(:,2), map(:,3), 'sk', 'MarkerSize', 10)
h_leg{1} = plot3(x_gt(:,1), x_gt(:,2), x_gt(:,3), '--', 'color', [Darkgrey, 0.7], 'linewidth', 2);
plot3(x_gt(:,1), x_gt(:,2), x_gt(:,3), '.', 'color', [Darkgrey, 0.7], 'MarkerSize', 18)

%% Simulate noisy measuremest in form of range and bearing
sigma_range = 0.02;  % standard deviation along range
sigma_bearing = 0.001;  % standard deviation along bearing

Sigma_z = blkdiag(sigma_range^2, sigma_bearing^2, sigma_bearing^2);
Lz = chol(Sigma_z, 'lower');

z = cell(size(x_gt,1),2);         % measurements
z_max = 2.5;      % maximum sensor range in meters
% create a kd-tree structure to search within the sensor range
MdlKDT = KDTreeSearcher(map);
for i = 1:size(x_gt,1)
    Idx = rangesearch(MdlKDT, x_gt(i,:), z_max);
    map_gt = map(Idx{1},:)
    del = (map_gt - x_gt(i,:))';
    range = diag(sqrt(del'*del))';
    z{i,1} = [range;
        atan(del(2,:)./del(1,:));
        asin(del(3,:)./range)
        ] + (Lz * randn(size(map_gt,1),3)'); % landmark coordinates
    z{i,2} = Idx{1};        % correspondences
    
    for j = 1:size(z{i,1},2)
        line([x_gt(i,1), map_gt(j,1)], [x_gt(i,2), map_gt(j,2)], [x_gt(i,3), map_gt(j,3)],'Color',[green, 0.2],'LineStyle','-', 'linewidth', 1.5)
    end
end

%% Simulate noisy odometry measurements
sigma_ux = 0.1;
sigma_uy = 0.1;
sigma_uz = 0.05;
Sigma_u = blkdiag(sigma_ux^2, sigma_uy^2, sigma_uz^2);
Lu = chol(Sigma_u, 'lower');

u = diff(x_gt) + (Lu * randn(size(x_gt,1)-1,3)')';
x_init = [0, 0, 1.0];
Sigma_init = diag([1e-3,1e-3,1e-3]);
x_odom = [x_init; cumsum(u)+ x_init];
np = size(x_odom,1); % number of poses

% plot odometry trajectory
h_leg{2} = plot3(x_odom(:,1), x_odom(:,2), x_odom(:,3), ':', 'color', [VermillionRed, 0.7], 'linewidth', 2);
plot3(x_odom(:,1), x_odom(:,2), x_odom(:,3), '.', 'color', [VermillionRed, 0.7], 'MarkerSize', 18)

%%  gtsam setup
% Create a factor graph container

graph = NonlinearFactorGraph;
initials = Values();
isam2 = ISAM2();

initModel = noiseModel.Diagonal.Sigmas([1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3]);
odomModel = noiseModel.Diagonal.Sigmas([0.1; 0.1;0.05; 1e-3; 1e-3; 1e-3]);
obsModel = noiseModel.Diagonal.Sigmas(0.02);   
obsinitModel = noiseModel.Diagonal.Sigmas([0.1; 0.1;0.02]);   
% Initial pose
r = Rot3(eye(3));
t = Point3(x_init(1), x_init(2), x_init(3));
graph.add(PriorFactorPose3(symbol('x', 1), Pose3(r, t), initModel));
initials.insert(symbol('x', 1), Pose3(r, t));
isam2.update(graph, initials)
%%
seen_landmarks = [];    % list of seen landmarks

% Initial measurement
if ~isempty(z{1,2})
    graph = NonlinearFactorGraph;
    initials = Values();
    for j = 1:length(z{1,2})
        range = z{1,1}(1,j);
        if any(seen_landmarks == z{1,2}(j)) % old landmark, add a loop-closure
            % find the landmark id stored in A
            l_id = find(seen_landmarks == z{1,2}(j));
            graph.add(RangeFactorPosePoint3(symbol('x', 1),symbol('l', l_id),range,obsModel));
        else % new landmark, exploration;
            % append landmark id
            seen_landmarks = [seen_landmarks, z{1,2}(j)];
            l_id = length(seen_landmarks);
            pt = Point3(x_odom(1,1) + z{1,1}(1,j)*cos(z{1,1}(3,j))*cos(z{1,1}(2,j))...
                , x_odom(1,2) + z{1,1}(1,j)*cos(z{1,1}(3,j))*sin(z{1,1}(2,j))...
                , x_odom(1,3) + z{1,1}(1,j)*sin(z{1,1}(3,j)) );
            graph.add(PriorFactorPoint3(symbol('l', l_id),pt,obsinitModel));
            initials.insert(symbol('l', l_id), pt);
        end
    end
end
isam2.update(graph, initials)
results{1} = isam2.calculateBestEstimate();
graph_step{1} = isam2.getFactorsUnsafe();
maiginals = [];
marginals = Marginals(graph_step{1}, results{1});
poses = getAll3Dposes(results{1}, marginals);

for i = 2:np
    
    graph = NonlinearFactorGraph;
    initials = Values();
    r = Rot3.RzRyRx(0.0, 0.0, 0.0);
    t = Point3(u(i-1,1), u(i-1,2), u(i-1,3));
    graph.add(BetweenFactorPose3(symbol('x', i-1), symbol('x', i), Pose3(r,t), odomModel));
    if ~isempty(z{i,2})
        for j = 1:length(z{i,2})
            range = z{i,1}(1,j);
            if any(seen_landmarks == z{i,2}(j)) % old landmark, add a loop-closure
                % find the landmark id stored in A
                l_id = find(seen_landmarks == z{i,2}(j));
                graph.add(RangeFactorPosePoint3(symbol('x', i),symbol('l', l_id),range,obsModel));
            else % new landmark, exploration;
                % append landmark id
                seen_landmarks = [seen_landmarks, z{i,2}(j)];
                l_id = length(seen_landmarks);
                pt = Point3(poses.p{i-1}(1) + u(i-1,1) + z{i,1}(1,j)*cos(z{i,1}(3,j))*cos(z{i,1}(2,j))...
                    , poses.p{i-1}(2) + u(i-1,2) + z{i,1}(1,j)*cos(z{i,1}(3,j))*sin(z{i,1}(2,j))...
                    ,  poses.p{i-1}(3) + u(i-1,3) + z{i,1}(1,j)*sin(z{i,1}(3,j)));
                graph.add(PriorFactorPoint3(symbol('l', l_id),pt,obsinitModel));
                initials.insert(symbol('l', l_id), pt);
            end
        end
    end
    t = Point3(poses.p{i-1}(1) + u(i-1,1), poses.p{i-1}(2) + u(i-1,2), poses.p{i-1}(3) + u(i-1,3));
    initials.insert(symbol('x', i), Pose3(r, t));
    isam2.update(graph, initials)
    tic
    results{i} = isam2.calculateBestEstimate();
    toc
    graph_step{i} = isam2.getFactorsUnsafe();
    maiginals = [];
    marginals = Marginals(graph_step{i}, results{i});
    poses = getAll3Dposes(results{i}, marginals);
    hold off
    plot3DTrajectory(results{i}, [], [], [], marginals);
    plot3DPoints(results{i}, [], marginals);
end


%% Plot results
import gtsam.*
% SLAM trajectory
index = 16;
graph_step{index}.print('\nFactor Graph:\n'); 
results{index}.print('Final Result: \n');
% Calculate marginal covariances for all poses
maiginals = [];
marginals = Marginals(graph_step{index}, results{index});

figure()
hold on; grid on; axis auto
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
plot3(map(:,1), map(:,2), map(:,3), '*k', 'MarkerSize', 10)
plot3(map(:,1), map(:,2), map(:,3), 'sk', 'MarkerSize', 10)
h_leg{1} = plot3(x_gt(:,1), x_gt(:,2), x_gt(:,3), '--', 'color', [Darkgrey, 0.7], 'linewidth', 2);
plot3(x_gt(:,1), x_gt(:,2), x_gt(:,3), '.', 'color', [Darkgrey, 0.7], 'MarkerSize', 18)
plot3DTrajectory(results{index}, [], [], [], marginals);
plot3DPoints(results{index}, [], marginals);
% points = get2Dpoints(results{index}, marginals);
% poses = get2Dposes(results{index}, marginals);

axis equal

