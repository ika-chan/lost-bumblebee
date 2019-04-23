clc; clear; close all

global measurement_path
global steps

%% Set these parameters
% direct address to the folder contains observations
measurement_path = ""
% the range should be [100, 2301]
steps = 200;
%%
steps = min(max(steps,100),2301);

%% Comment or uncomment these lines
% uncomment to run the graph SLAM based on soft association
% ISAM2_back_end_SS

% uncomment to run the graph SLAM based on hard association
ISAM2_back_end_HS

%% toy examples

% uncomment to run a 2D graph SLAM solved by Nonlinear Least Squares
% gtsam_slam_nls

% uncomment to run a 2D graph SLAM solved by ISAM2
% gtsam_slam_ISAM2_2D

% uncomment to run a 3D graph SLAM solved by ISAM2
% gtsam_slam_ISAM2_3D