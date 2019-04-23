%{
    Author  Dingyi Sun
    Date    April 9 2019
    The solver for the ploblem has changed into gtsam.
    Make sure to add the directory: gtsam_toobox 
%}


import gtsam.*

fsize = 16; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

%% Parse the data from the front end 
global measurement_path
global steps

[gt, Measurements, Transform] = Data_Parsing_old(measurement_path)
np = length(Transform);
%%  gtsam setup
% Create a factor graph container
import gtsam.*

graph = NonlinearFactorGraph;
initials = Values();
isam2 = ISAM2();

motion_noise = 0.002;
initModel = noiseModel.Diagonal.Sigmas([1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3]);
odomModel = noiseModel.Diagonal.Sigmas([motion_noise;motion_noise; motion_noise; 1e-3; 1e-3; 1e-3]);
sensor_noise = 0.5;
obsModel = noiseModel.Diagonal.Sigmas(sensor_noise);

obsinitModel = noiseModel.Diagonal.Sigmas([0.5;0.5;2]);   
% Initial pose

r = Rot3(gt{1}.R);
t = Point3(gt{1}.p);
graph.add(PriorFactorPose3(symbol('x', 0), Pose3(r, t), initModel));
initials.insert(symbol('x', 0), Pose3(r, t));

seen_landmarks = [];    % list of seen landmarks
isam2.update(graph, initials)
results{1} = isam2.calculateBestEstimate();
graph_step{1} = isam2.getFactorsUnsafe();
maiginals = [];
marginals = Marginals(graph_step{1}, results{1});
last_pose = getLast3Dpose(results{1}, marginals);
% all_points = getAll3Dpoints(results{1}, marginals);

% main loop
f = 1
index = steps;
for i = 1:index
    graph = NonlinearFactorGraph;
    initials = Values();
    r = Rot3(Transform{i}.R);
    t = Point3(Transform{i}.p);
    
    % current pose
    graph.add(BetweenFactorPose3(symbol('x', i-1), symbol('x', i), Pose3(r,t), odomModel));
    
    % consecutive rigid body transformation
    r = Rot3(last_pose.R*Transform{i}.R);
    t = Point3(last_pose.R*Transform{i}.p+last_pose.p);
    
    initials.insert(symbol('x', i), Pose3(r, t));
    
    
    % optimize
    isam2.update(graph, initials);
    results{i+1} = isam2.calculateBestEstimate();
    graph_step{i+1} = isam2.getFactorsUnsafe();
    maiginals = [];
    marginals = Marginals(graph_step{i+1}, results{i+1});
    curr_pose = getLast3Dpose(results{i+1}, marginals);
    graph = NonlinearFactorGraph;
    initials = Values();
    
    try
        all_points = getAll3Dpoints(results{i+1}, marginals);
        landmark_num = length(all_points.p);
        
    catch
        all_points = [];
        landmark_num = length(all_points);
        
    end
    
    
    candidate_points = DiscriminativePointReduction(all_points,curr_pose);
    if isempty(Measurements{i})
        last_pose = curr_pose;
    else
    [Maha_mat,L] = da_nn(Measurements{i}, candidate_points, curr_pose,sensor_noise);
    
    for j = 1:length(L)
        range = Measurements{i}.range(:,j);
        if range > 30
            continue;
        end
        
        if L(j) ~= 0
            l_id = L(j);
            graph.add(RangeFactorPosePoint3(symbol('x', i),symbol('l', l_id),range,obsModel));
        else % new landmark, exploration;
            l_id = landmark_num + 1;         

            displacement = getGlobalDisplacement(Measurements{i}.p(:,j), curr_pose);
            
            pt = Point3(displacement.p(1), displacement.p(2), displacement.p(3));
            graph.add(PriorFactorPoint3(symbol('l', l_id),pt,obsinitModel));
            initials.insert(symbol('l', l_id), pt);
            landmark_num = l_id;
        end
    end
    
    isam2.update(graph, initials);
    results{i+1} = isam2.calculateBestEstimate();
    graph_step{i+1} = isam2.getFactorsUnsafe();
    maiginals = [];
    marginals = Marginals(graph_step{i+1}, results{i+1});

    last_pose = getLast3Dpose(results{i+1}, marginals);
    end
       
    i


end
%
% video = VideoWriter('ISAM.mp4','MPEG-4')
% video.FrameRate = 2;
% open(video)
% writeVideo(video,F)
% close(video)



%% Plot results (VS ground truth trajectory)
% import gtsam.*
% SLAM trajectory

graph_step{index}.print('\nFactor Graph:\n'); 
results{index}.print('Final Result: \n');
% Calculate marginal covariances for all poses
maiginals = [];
marginals = Marginals(graph_step{index}, results{index});

figure()
hold on; grid on; axis auto
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')

all_poses = getAll3Dposes(results{index}, marginals);
plot3DTrajectory(results{index}, [], [], [], marginals);
plot3DPoints(results{index}, [], marginals);

for i = 1:index
    r = Rot3(gt{i}.R);
    t = Point3(gt{i}.p);
    gtpose = Pose3(r,t);
    plot3(gtpose.x , gtpose.y , gtpose.z ,'rx');
    plotPose3(gtpose, [], 1);
end

axis equal


