clc; clear; close all

import gtsam.*


index = 2301;



fsize = 16; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');
% nice colours
colours.green = [0.2980 .6 0];
colours.crimson = [220,20,60]/255; 
colours.darkblue = [0 .2 .4];
colours.Darkgrey = [.25 .25 .25];
colours.darkgrey = [.35 .35 .35];
colours.lightgrey = [.7 .7 .7];
colours.Lightgrey = [.9 .9 .9];
colours.VermillionRed = [156,31,46]/255;
colours.DupontGray = [144,131,118]/255;
colours.Azure = [53, 112, 188]/255;
colours.purple = [178, 102, 255]/255;
colours.orange = [255,110,0]/255;

%load soft association result
load('\\engin-labs.m.storage.umich.edu\dysun\windat.V2\Desktop\TODO\GTSAM_semantic_whole\result\soft_association_result.mat')
all_poses_S = all_poses;
all_points_S = all_points;
%load hard association result 
load('\\engin-labs.m.storage.umich.edu\dysun\windat.V2\Desktop\TODO\GTSAM_semantic_whole\result\hard_association_result.mat')
all_poses_H = all_poses;
all_points_H = all_points;
%load ground truth
load('SE3_FINAL.mat');
j = 1;
while j < index + 1
    gt.R{j} = pose{j}(1:3,1:3);
    gt.p{j} = pose{j}(1:3,4);
    j = j + 1;
end
%load odometry
load('Global_total.mat');
j = 1;
while j < index + 1
    odometry.R{j} = SE3_Final{j}(1:3,1:3);
    odometry.p{j} = SE3_Final{j}(1:3,4);
    j = j + 1;
end 
%% error calculation
len = 0;
for i = 2:index
Relative_gt = [
    gt.R{i-1} gt.p{i-1}
    0 0 0 1]\[
    gt.R{i} gt.p{i}
    0 0 0 1];
d_l = sqrt(Relative_gt(1:3,4)'*Relative_gt(1:3,4));
len = len + d_l
end

isam_norm_R = 0
t_norm_R = 0
R_norm_R = 0

isam_norm_H = 0
t_norm_H = 0
R_norm_H = 0

isam_norm_S = 0
t_norm_S = 0
R_norm_S = 0

previous_R = [
    odometry.R{1} odometry.p{1}
    0 0 0 1];
previous_H = [
    all_poses_H.R{1} all_poses_H.p{1}
    0 0 0 1];
previous_S = [
    all_poses_S.R{1} all_poses_S.p{1}
    0 0 0 1];
previous_G = [
    gt.R{1} gt.p{1}
    0 0 0 1];
for i = 1:index
    SE3_ISAM_R = [
        odometry.R{i} odometry.p{i}
        0 0 0 1];
    SE3_ISAM_H = [
        all_poses_H.R{i} all_poses_H.p{i}
        0 0 0 1];
    SE3_ISAM_S = [
        all_poses_S.R{i} all_poses_S.p{i}
        0 0 0 1];
    SE3_gt = [
        gt.R{i} gt.p{i}
        0 0 0 1];
    isam_norm_R = isam_norm_R + SE3norm(SE3_ISAM_R,SE3_gt);
    isam_norm_H = isam_norm_H + SE3norm(SE3_ISAM_H,SE3_gt);
    isam_norm_S = isam_norm_S + SE3norm(SE3_ISAM_S,SE3_gt);
    if i > 1
        current_R = SE3_ISAM_R;
        current_H = SE3_ISAM_H;
        current_S = SE3_ISAM_S;
        current_G = SE3_gt;
        
        [tError_R RError_R] = KITTI_norm(previous_R, current_R,previous_G, current_G);
        t_norm_R = t_norm_R + tError_R;
        R_norm_R = R_norm_R + RError_R;
        
        [tError_H RError_H] = KITTI_norm(previous_H, current_H,previous_G, current_G);
        t_norm_H = t_norm_H + tError_H;
        R_norm_H = R_norm_H + RError_H;
        
        [tError_S RError_S] = KITTI_norm(previous_S, current_S,previous_G, current_G);
        t_norm_S = t_norm_S + tError_S;
        R_norm_S = R_norm_S + RError_S;
        
        previous_R = current_R;
        previous_H = current_H;
        previous_S = current_S;
        previous_G = current_G;
    end

i
end
isam_norm_R = isam_norm_R/len 
t_norm_R = t_norm_R/len
R_norm_R = R_norm_R/len

isam_norm_H = isam_norm_H/len 
t_norm_H = t_norm_H/len
R_norm_H = R_norm_H/len

isam_norm_S = isam_norm_S/len
t_norm_S = t_norm_S/len
R_norm_S = R_norm_S/len
%% plot
figure()
hold on; grid on; axis auto
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')

for i = 1:index
    r = Rot3(gt.R{i});
    t = Point3(gt.p{i});
    gtpose = Pose3(r,t);
    plot3(gtpose.x , gtpose.y , gtpose.z ,'Color',colours.VermillionRed,'Marker','x','MarkerSize',10,'MarkerFaceColor',colours.VermillionRed);
    plotPose3(gtpose, [], 0.1);
end


for i = 1:index
    r = Rot3(all_poses_H.R{i});
    t = Point3(all_poses_H.p{i});
    gtpose = Pose3(r,t);
    plot3(gtpose.x , gtpose.y , gtpose.z ,'Color',colours.orange,'Marker','o','MarkerSize',10,'MarkerFaceColor',colours.orange);
    plotPose3(gtpose, [], 1);
    
    r = Rot3(all_poses_S.R{i});
    t = Point3(all_poses_S.p{i});
    gtpose = Pose3(r,t);
    plot3(gtpose.x , gtpose.y , gtpose.z ,'Color',colours.darkblue,'Marker','o','MarkerSize',10,'MarkerFaceColor',colours.darkblue);
    plotPose3(gtpose, [], 0.1);
end

axis equal

