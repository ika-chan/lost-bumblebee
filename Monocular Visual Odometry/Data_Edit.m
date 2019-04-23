
starter = 1;
count = 1;
SE3_multiply = pose{starter};
groundTruth_trans = [];
groundTruth_rot = [];
num = [];
if starter == 1
    saver = eye(4);
end
for i = starter:starter+100
   poser{count} = ((SE3_multiply)^(-1))*pose{i} ;
   groundTruth_trans{count} = [-poser{count}(2,4),-poser{count}(3,4),poser{count}(1,4)];
   groundTruth_rot{count} = poser{count}(1:3,1:3);
   count = count + 1;   
end
num = starter:starter+100;
T = table(num',groundTruth_trans',groundTruth_rot');
groundTruthPoses = T;
images = imageDatastore('\\engin-labs.m.storage.umich.edu\pwestra\windat.v2\Desktop\Monocular Visual Odometry\image01');
groundTruthPoses.Properties.VariableNames = {'ViewId','Location','Orientation'};