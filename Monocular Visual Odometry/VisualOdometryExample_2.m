%% Monocular Visual Odometry
% Visual odometry is the process of determining the location and orientation
% of a camera by analyzing a sequence of images. Visual odometry is used in
% a variety of applications, such as mobile robots, self-driving cars, and 
% unmanned aerial vehicles. This example shows you how to estimate the
% trajectory of a single calibrated camera from a sequence of images. 

% Copyright 2016 The MathWorks, Inc. 

K = [707 0 602; 0 707 183; 0 0 1]';
cameraParams = cameraParameters('IntrinsicMatrix', K);

%% Create a View Set Containing the First View of the Sequence
% Use a |viewSet| object to store and manage the image points and the
% camera pose associated with each view, as well as point matches between
% pairs of views. Once you populate a |viewSet| object, you can use it to
% find point tracks across multiple views and retrieve the camera poses to
% be used by |triangulateMultiview| and |bundleAdjustment| functions.

% Create an empty viewSet object to manage the data associated with each view.
vSet = viewSet;

% Read and display the first image.
Irgb = readimage(images, 1);
player = vision.VideoPlayer('Position', [20, 400, 650, 510]);
step(player, Irgb);

%%
% Convert to gray scale and undistort. In this example, undistortion has no 
% effect, because the images are synthetic, with no lens distortion. However, 
% for real images, undistortion is necessary.

prevI = undistortImage(rgb2gray(Irgb), cameraParams); 

% Detect features. 
prevPoints = detectSURFFeatures(prevI, 'MetricThreshold', 5000);

% Select a subset of features, uniformly distributed throughout the image.
numPoints = 300;
prevPoints = selectUniform(prevPoints, numPoints, size(prevI));

% Extract features. Using 'Upright' features improves matching quality if 
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(prevI, prevPoints, 'Upright', true);

deg_2_rad = pi/180;
rot1 = [1 0 0;0 cos(-90*deg_2_rad) -sin(-90*deg_2_rad);0 sin(90*deg_2_rad) cos(-90*deg_2_rad)];
rot2 = [cos(90*deg_2_rad) 0 sin(90*deg_2_rad); 0 1 0;-sin(90*deg_2_rad) 0 cos(90*deg_2_rad)];
rot = rot1*rot2;
% Add the first view. Place the camera associated with the first view
% at the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', eye(3),...
    'Location', [0 0 0]);

%% Plot Initial Camera Pose
% Create two graphical camera objects representing the estimated and the
% actual camera poses based on ground truth data from the New Tsukuba
% dataset.

% Setup axes.
figure
axis([-220, 100, -140, 20, -50, 300]);

% Set Y-axis to be vertical pointing down.
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);

grid on
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
hold on

% Plot estimated camera pose. 
cameraSize = 7;
camEstimated = plotCamera('Size', cameraSize, 'Location',...
    vSet.Views.Location{1}, 'Orientation', vSet.Views.Orientation{1},...
    'Color', 'g', 'Opacity', 0);

% Plot actual camera pose.
camActual = plotCamera('Size', cameraSize, 'Location', ...
    groundTruthPoses.Location{1}, 'Orientation', ...
    groundTruthPoses.Orientation{1}, 'Color', 'b', 'Opacity', 0);

% Initialize camera trajectories.
trajectoryEstimated = plot3(0, 0, 0, 'g-');
trajectoryActual    = plot3(0, 0, 0, 'b-');

legend('Estimated Trajectory', 'Actual Trajectory');
title('Camera Trajectory');

%% Estimate the Pose of the Second View
% Detect and extract features from the second view, and match them to the
% first view using <matlab:edit('helperDetectAndMatchFeatures.m') helperDetectAndMatchFeatures>. 
% Estimate the pose of the second view relative to the first view using 
% <matlab:edit('helperEstimateRelativePose.m') helperEstimateRelativePose>,
% and add it to the |viewSet|.

% Read and display the image.
viewId = 2;
Irgb = readimage(images, viewId);
step(player, Irgb);

% Convert to gray scale and undistort.
I = undistortImage(rgb2gray(Irgb), cameraParams);

% Match features between the previous and the current image.
[currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
    prevFeatures, I);

% Estimate the pose of the current view relative to the previous view.
[orient, loc, inlierIdx] = helperEstimateRelativePose(...
    prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), cameraParams);

% Exclude epipolar outliers.
indexPairs = indexPairs(inlierIdx, :);
    
% Add the current view to the view set.
vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
    'Location', loc);
% Store the point matches between the previous and the current views.
vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);

%%
% The location of the second view relative to the first view can only be
% recovered up to an unknown scale factor. Compute the scale factor from 
% the ground truth using <matlab:edit('helperNormalizeViewSet.m') helperNormalizeViewSet>,
% simulating an external sensor, which would be used in a typical monocular
% visual odometry system.

vSet = helperNormalizeViewSet(vSet, groundTruthPoses);

%%
% Update camera trajectory plots using 
% <matlab:edit('helperUpdateCameraPlots.m') helperUpdateCameraPlots> and
% <matlab:edit('helperUpdateCameraTrajectories.m') helperUpdateCameraTrajectories>.

helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(vSet), ...
    groundTruthPoses);

helperUpdateCameraTrajectories(viewId, trajectoryEstimated, trajectoryActual,...
    poses(vSet), groundTruthPoses);

prevI = I;
prevFeatures = currFeatures;
prevPoints   = currPoints;

%% Bootstrap Estimating Camera Trajectory Using Global Bundle Adjustment
% Find 3D-to-2D correspondences between world points triangulated from the 
% previous two views and image points from the current view. Use 
% <matlab:edit('helperFindEpipolarInliers.m') helperFindEpipolarInliers> 
% to find the matches that satisfy the epipolar constraint, and then use
% <matlab:edit('helperFind3Dto2DCorrespondences.m') helperFind3Dto2DCorrespondences>
% to triangulate 3-D points from the previous two views and find the
% corresponding 2-D points in the current view.
%
% Compute the world camera pose for the current view by solving the 
% perspective-n-point (PnP) problem using |estimateWorldCameraPose|. For 
% the first 15 views, use global bundle adjustment to refine the entire
% trajectory. Using global bundle adjustment for a limited number of views
% bootstraps estimating the rest of the camera trajectory, and it is not
% prohibitively expensive.
i = 1;
for viewId = 3:numel(images.Files)
    % Read and display the next image
    Irgb = readimage(images, viewId);
    step(player, Irgb);
    Q(i) = im2frame(Irgb);
    % Convert to gray scale and undistort.
    I = undistortImage(rgb2gray(Irgb), cameraParams);
    
    % Match points between the previous and the current image.
    [currPoints, currFeatures, indexPairs] = helperDetectAndMatchFeatures(...
        prevFeatures, I);
      
    % Eliminate outliers from feature matches.
    inlierIdx = helperFindEpipolarInliers(prevPoints(indexPairs(:,1)),...
        currPoints(indexPairs(:, 2)), cameraParams);
    indexPairs = indexPairs(inlierIdx, :);
    
    % Triangulate points from the previous two views, and find the 
    % corresponding points in the current view.
    [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences(vSet,...
        cameraParams, indexPairs, currPoints);
    
    % Since RANSAC involves a stochastic process, it may sometimes not
    % reach the desired confidence level and exceed maximum number of
    % trials. Disable the warning when that happens since the outcomes are
    % still valid.
    warningstate = warning('off','vision:ransac:maxTrialsReached');
    
    % Estimate the world camera pose for the current view.
    [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
        cameraParams, 'Confidence', 99.99, 'MaxReprojectionError', 0.8);
    
    % Restore the original warning state
    warning(warningstate)
    
    % Add the current view to the view set.
    vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
        'Location', loc);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);    
    
    tracks = findTracks(vSet); % Find point tracks spanning multiple views.
        
    camPoses = poses(vSet);    % Get camera poses for all views.
    
    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine camera poses using bundle adjustment.
    [~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, ...
        cameraParams, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
        'RelativeTolerance', 1e-9, 'MaxIterations', 300);
        
    vSet = updateView(vSet, camPoses); % Update view set.
    
    % Bundle adjustment can move the entire set of cameras. Normalize the
    % view set to place the first camera at the origin looking along the
    % Z-axes and adjust the scale to match that of the ground truth.
    vSet = helperNormalizeViewSet(vSet, groundTruthPoses);
    
    % Update camera trajectory plot.
    helperUpdateCameraPlots(viewId, camEstimated, camActual, poses(vSet), ...
        groundTruthPoses);
    helperUpdateCameraTrajectories(viewId, trajectoryEstimated, ...
        trajectoryActual, poses(vSet), groundTruthPoses);

    i = i + 1;
    prevI = I;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;  

end

transform = [0 0 1 0;-1 0 0 0;0 -1 0 0;0 0 0 1];
SE3 = [];
saver = [pose{starter}(1:3,1:3),saver(1:3,4);0 0 0 1];
for i = 1:height(camPoses)
    SE3{i} = saver*transform*[camPoses.Orientation{i},camPoses.Location{i}' ;0 0 0 1]*(transform^-1);
    if i>1
        Relative{i} = SE3{i-1}^(-1)*SE3{i};
    end
end

SE3_total{counter} = SE3;
Relative_total{counter} = Relative;
j = [];
p = [];
counter2 = 1;
for k = 1:length(SE3_total)
    for q = 1:length(SE3)
        j(1:3,counter2) = SE3_total{k}{q}(1:3,4);
        p(1:3,counter2) = pose{counter2}(1:3,4);
        counter2 = counter2 + 1;
    end
end 

saver = SE3{length(SE3)};
%% Plot of Ground Truth Pose and Pose Obtained From Monocular Visual Odometry
figure
k = norm(j,1);
scatter3(j(1,:),j(2,:),j(3,:))
hold on
scatter3(p(1,:),p(2,:),p(3,:),'r')
axis([-k, k, -k, k, -k, k])
grid on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
hold on
counter = counter + 1;
%% References
%
% [1] Martin Peris Martorell, Atsuto Maki, Sarah Martull, Yasuhiro Ohkawa,
%     Kazuhiro Fukui, "Towards a Simulation Driven Stereo Vision System". 
%     Proceedings of ICPR, pp.1038-1042, 2012.
% 
% [2] Sarah Martull, Martin Peris Martorell, Kazuhiro Fukui, "Realistic CG 
%     Stereo Image Dataset with Ground Truth Disparity Maps", Proceedings of
%     ICPR workshop TrakMark2012, pp.40-42, 2012.
%
% [3] M.I.A. Lourakis and A.A. Argyros (2009). "SBA: A Software Package for
%     Generic Sparse Bundle Adjustment". ACM Transactions on Mathematical
%     Software (ACM) 36 (1): 1-30.
%
% [4] R. Hartley, A. Zisserman, "Multiple View Geometry in Computer
%     Vision," Cambridge University Press, 2003.
%
% [5] B. Triggs; P. McLauchlan; R. Hartley; A. Fitzgibbon (1999). "Bundle
%     Adjustment: A Modern Synthesis". Proceedings of the International
%     Workshop on Vision Algorithms. Springer-Verlag. pp. 298-372.
%
% [6] X.-S. Gao, X.-R. Hou, J. Tang, and H.-F. Cheng, "Complete Solution 
%     Classification for the Perspective-Three-Point Problem," IEEE Trans. 
%     Pattern Analysis and Machine Intelligence, vol. 25, no. 8, pp. 930-943, 
%     2003.
